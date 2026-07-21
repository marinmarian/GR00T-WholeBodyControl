"""
Inspire RH56DFTP hand driver over direct Modbus TCP.

Vendored from prosus-robotics/teleop
(xr_teleoperate/teleop/robot_control/robot_hand_inspire_modbus.py) — the
low-level InspireHandModbusTCP class only; the xr_teleoperate-specific
Inspire_Controller_FTP multiprocess wrapper is intentionally omitted.
Changes: logging_mp -> stdlib logging, dropped multiprocessing import.

Hands: left 192.168.123.210, right 192.168.123.211, TCP port 6000.
6 DOF per hand, order [little, ring, middle, index, thumb_bend, thumb_rot],
angle range 0-1000 (1000 = open, 0 = closed).
"""


from pymodbus.client import ModbusTcpClient
import numpy as np
import threading
import time
import logging
logger_mp = logging.getLogger(__name__)

Inspire_Num_Motors = 6

REG_ANGLE_SET = 1486
REG_ANGLE_ACT = 1546
REG_FORCE_ACT = 1582
REG_SPEED_SET = 1522
REG_FORCE_SET = 1498
REG_CURRENT_ACT = 1594
REG_GESTURE_FORCE_CLB = 1009
REG_CLEAR_ERROR = 1004
REG_SAVE = 1005

MODBUS_DEVICE_ID = 255

# Named gesture presets (angle 0-1000 per DOF; 0 = closed, 1000 = open).
# Ported from correlllab/rh56_controller (rh56_driver.py gesture library).
# DOF order: [little, ring, middle, index, thumb_bend, thumb_rotation].
GESTURES = {
    "open":  [1000, 1000, 1000, 1000, 1000, 1000],
    "close": [0, 0, 0, 0, 0, 0],
    "pinch": [1000, 1000, 0, 0, 1000, 0],
    "point": [0, 0, 0, 1000, 1000, 1000],
}

# Which tactile region senses contact for each DOF (the closing fingers).
# DOF order: [little, ring, middle, index, thumb_bend, thumb_rotation].
# Thumb bend + rotation share the thumb pad; palm has no dedicated DOF.
DOF_TACTILE_REGION = {
    0: "little_finger", 1: "ring_finger", 2: "middle_finger",
    3: "index_finger", 4: "thumb", 5: "thumb",
}

# Force-to-Newton calibration, ported from correlllab/rh56_controller
# (grasp_executor._FORCE_CALIB). Per-DOF (a, b) such that F_newtons = a*raw + b,
# clamped >= 0. NOTE: these coefficients were fit on the RH56DFX (force-only)
# hands; the DFTP force scale may differ, so treat the Newton values as
# approximate until re-calibrated on this hardware.
FORCE_CALIB = {
    0: (0.006452,  0.018),   # little (pinky)
    1: (0.006452,  0.018),   # ring
    2: (0.006452,  0.018),   # middle
    3: (0.007478, -0.414),   # index
    4: (0.012547,  0.384),   # thumb bend
    5: (0.012547,  0.384),   # thumb rotation (yaw)
}

# --- Tactile sensors (RH56DFTP only) ---------------------------------------
# Register map from the RH56DFTP User Manual V1.0.0, section 2.6.20.
# The ModbusTCP gateway uses the documented byte start-address as the register
# address, and each holding register holds one 16-bit tactile point (0-4095).
# So `count` (number of registers) == bytes / 2 for each region.
#
# Each region is itself subdivided into pads laid out as (rows, cols), filled
# row-major (data point 1 = row 1 col 1, point 2 = row 1 col 2, ...), except the
# palm which is filled column-major bottom-to-top (see _PALM_COLUMN_MAJOR).
# Layout: each pad is (name, rows, cols); points consumed = rows * cols.
TACTILE_REGIONS = {
    # finger: tip 3x3, nail 12x8, pad 10x8  => 9 + 96 + 80 = 185 points (370 bytes)
    "little_finger": {"base": 3000, "pads": [("tip", 3, 3), ("nail", 12, 8), ("pad", 10, 8)]},
    "ring_finger":   {"base": 3370, "pads": [("tip", 3, 3), ("nail", 12, 8), ("pad", 10, 8)]},
    "middle_finger": {"base": 3740, "pads": [("tip", 3, 3), ("nail", 12, 8), ("pad", 10, 8)]},
    "index_finger":  {"base": 4110, "pads": [("tip", 3, 3), ("nail", 12, 8), ("pad", 10, 8)]},
    # thumb: tip 3x3, nail 12x8, middle 3x3, pad 12x8 => 9 + 96 + 9 + 96 = 210 points (420 bytes)
    "thumb":         {"base": 4480, "pads": [("tip", 3, 3), ("nail", 12, 8), ("middle", 3, 3), ("pad", 12, 8)]},
    # palm: 8x14 = 112 points (224 bytes)
    "palm":          {"base": 4900, "pads": [("palm", 8, 14)]},
}

# Total tactile points = 185*4 + 210 + 112 = 1062
TACTILE_TOTAL_POINTS = sum(
    rows * cols for region in TACTILE_REGIONS.values() for _, rows, cols in region["pads"]
)

# Max 16-bit registers per Modbus read request (spec limit is 125; stay safe).
MODBUS_MAX_READ = 120

DEFAULT_LEFT_IP  = "192.168.123.210"
DEFAULT_RIGHT_IP = "192.168.123.211"
MODBUS_PORT = 6000


class InspireHandModbusTCP:
    def __init__(self, ip, port=MODBUS_PORT, label="hand"):
        self.ip = ip
        self.port = port
        self.label = label
        self.client = None
        self.connected = False
        self.force_limits = [1000] * Inspire_Num_Motors  # last-written FORCE_SET values

    def connect(self):
        try:
            self.client = ModbusTcpClient(self.ip, port=self.port, timeout=1)
            if self.client.connect():
                self.connected = True
                logger_mp.info(f"[{self.label}] Connected to {self.ip}:{self.port}")
                return True
            else:
                logger_mp.warning(f"[{self.label}] Failed to connect to {self.ip}:{self.port}")
                return False
        except Exception as e:
            logger_mp.warning(f"[{self.label}] Connection error: {e}")
            return False

    def reconnect(self):
        if self.client:
            try:
                self.client.close()
            except:
                pass
        self.connected = False
        return self.connect()

    def read_angles(self):
        if not self.connected:
            return None
        try:
            r = self.client.read_holding_registers(REG_ANGLE_ACT, count=6, device_id=MODBUS_DEVICE_ID)
            if not r.isError():
                return list(r.registers)
        except Exception:
            self.connected = False
        return None

    def _read_registers(self, start, count):
        """Read `count` consecutive 16-bit holding registers starting at `start`,
        splitting into <=MODBUS_MAX_READ chunks to respect the Modbus PDU limit.
        Returns a flat list of ints, or None on any error."""
        if not self.connected:
            return None
        values = []
        offset = 0
        try:
            while offset < count:
                n = min(MODBUS_MAX_READ, count - offset)
                r = self.client.read_holding_registers(start + offset, count=n, device_id=MODBUS_DEVICE_ID)
                if r.isError():
                    return None
                values.extend(r.registers)
                offset += n
            return values
        except Exception:
            self.connected = False
            return None

    def read_tactile_raw(self):
        """Read every tactile region and return a dict of region_name -> flat
        list of 16-bit values (0-4095). Returns None if any region read fails."""
        out = {}
        for name, region in TACTILE_REGIONS.items():
            count = sum(rows * cols for _, rows, cols in region["pads"])
            vals = self._read_registers(region["base"], count)
            if vals is None:
                return None
            out[name] = vals
        return out

    def read_tactile(self):
        """Read tactile data and reshape into named pad arrays.
        Returns {region_name: {pad_name: np.ndarray(rows, cols)}}, or None on error."""
        raw = self.read_tactile_raw()
        if raw is None:
            return None
        out = {}
        for name, region in TACTILE_REGIONS.items():
            vals = raw[name]
            pads = {}
            idx = 0
            for pad_name, rows, cols in region["pads"]:
                n = rows * cols
                chunk = np.array(vals[idx:idx + n], dtype=np.int32)
                if name == "palm":
                    # Palm is filled column-major, bottom row first (manual 2.6.20):
                    # point 1 -> (row 8, col 1), point 2 -> (row 7, col 1), ...
                    grid = chunk.reshape(cols, rows).T[::-1, :]
                else:
                    grid = chunk.reshape(rows, cols)
                pads[pad_name] = grid
                idx += n
            out[name] = pads
        return out

    def write_angles(self, angles):
        if not self.connected:
            return False
        try:
            r = self.client.write_registers(REG_ANGLE_SET, values=angles, device_id=MODBUS_DEVICE_ID)
            return not r.isError()
        except Exception:
            self.connected = False
            return False

    # --- Force / speed / diagnostics -------------------------------------
    # Ported from correlllab/rh56_controller (rh56_hand.py), reusing the same
    # registers over Modbus TCP instead of their serial frame protocol.

    @staticmethod
    def _to_signed16(v):
        return v - 0x10000 if v >= 0x8000 else v

    def read_forces(self):
        """Read actual contact force per DOF (FORCE_ACT, signed, unit: grams).
        Range -4000..4000. Returns a list of 6 ints, or None on error."""
        regs = self._read_registers(REG_FORCE_ACT, Inspire_Num_Motors)
        if regs is None:
            return None
        return [self._to_signed16(v) for v in regs]

    @staticmethod
    def forces_to_newtons(raw_forces):
        """Convert raw FORCE_ACT readings to Newtons via FORCE_CALIB
        (F = a*raw + b, clamped >= 0). Ported from correlllab/rh56_controller.
        Returns a list of 6 floats (approximate — see FORCE_CALIB caveat)."""
        out = []
        for i, raw in enumerate(raw_forces):
            if i in FORCE_CALIB:
                a, b = FORCE_CALIB[i]
                out.append(round(max(0.0, a * raw + b), 4))
            else:
                out.append(round(max(0.0, raw / 1000.0 * 9.81), 4))  # fallback
        return out

    @staticmethod
    def newton_to_raw(dof, force_n):
        """Inverse calibration: target Newtons -> raw force units, clamped
        [0, 1000]. Useful for setting force thresholds in Newtons."""
        if dof in FORCE_CALIB:
            a, b = FORCE_CALIB[dof]
            return int(np.clip((force_n - b) / a, 0, 1000))
        return int(np.clip(force_n * 1000 / 9.81, 0, 1000))

    def read_forces_newtons(self):
        """Read actual contact force per DOF, converted to Newtons. Returns a
        list of 6 floats, or None on error."""
        raw = self.read_forces()
        if raw is None:
            return None
        return self.forces_to_newtons(raw)

    def read_currents(self):
        """Read actuator current per DOF (CURRENT, unit: mA, 0-2000)."""
        return self._read_registers(REG_CURRENT_ACT, Inspire_Num_Motors)

    def write_force_limits(self, thresholds):
        """Set the firmware force-control threshold per DOF (FORCE_SET, unit: g,
        range 0-3000). A finger closing toward its target stops once FORCE_ACT
        reaches this value."""
        if len(thresholds) != Inspire_Num_Motors:
            raise ValueError("Need 6 force thresholds")
        vals = [int(np.clip(t, 0, 3000)) for t in thresholds]
        if not self.connected:
            return False
        try:
            r = self.client.write_registers(REG_FORCE_SET, values=vals, device_id=MODBUS_DEVICE_ID)
            if r.isError():
                return False
            self.force_limits = vals
            return True
        except Exception:
            self.connected = False
            return False

    def write_speed(self, speeds):
        """Set movement speed per DOF (SPEED_SET, 0-1000)."""
        if len(speeds) != Inspire_Num_Motors:
            raise ValueError("Need 6 speed values")
        vals = [int(np.clip(s, 0, 1000)) for s in speeds]
        if not self.connected:
            return False
        try:
            r = self.client.write_registers(REG_SPEED_SET, values=vals, device_id=MODBUS_DEVICE_ID)
            return not r.isError()
        except Exception:
            self.connected = False
            return False

    def _write_single(self, address, value):
        if not self.connected:
            return False
        try:
            r = self.client.write_register(address, value, device_id=MODBUS_DEVICE_ID)
            return not r.isError()
        except Exception:
            self.connected = False
            return False

    def clear_errors(self):
        """Clear clearable actuator errors (locked-rotor, overcurrent, comms...)."""
        return self._write_single(REG_CLEAR_ERROR, 1)

    def save_parameters(self):
        """Persist current parameters (e.g. force limits, speed) to the hand's flash."""
        return self._write_single(REG_SAVE, 1)

    def calibrate_force(self, gesture_id=1):
        """Trigger the firmware force-sensor calibration routine (~6-15 s).
        Keep the hand open and untouched during calibration."""
        if not (1 <= gesture_id <= 255):
            raise ValueError("gesture_id must be 1-255")
        return self._write_single(REG_GESTURE_FORCE_CLB, gesture_id)

    # --- Higher-level motion / skills ------------------------------------

    def set_gesture(self, name):
        """Move to a named preset from GESTURES (e.g. 'open', 'pinch')."""
        if name not in GESTURES:
            raise ValueError(f"unknown gesture '{name}'; options: {list(GESTURES)}")
        return self.write_angles(GESTURES[name])

    def smooth_angle_set(self, target_angles, steps=30, delay=0.05):
        """Interpolate from the current angles to target over `steps` writes."""
        if len(target_angles) != Inspire_Num_Motors:
            raise ValueError("Need 6 angle values")
        current = self.read_angles()
        if current is None:
            return False
        current = np.array(current, dtype=float)
        target = np.array(target_angles, dtype=float)
        for i in range(1, steps + 1):
            if i == steps:
                nxt = target
            else:
                nxt = current + (target - current) * (i / steps)
            self.write_angles([int(np.clip(v, 0, 1000)) for v in nxt])
            time.sleep(delay)
        return True

    def adaptive_force_control(self, target_forces, target_angles, step_size=50,
                               max_iterations=20, speed=None, settle=0.10):
        """Closed-loop grasp: step fingers toward target_angles while holding the
        firmware force limit at target_forces, stopping each finger once its
        measured force reaches the threshold. Generator yielding per-iteration
        dicts, then a final {'done': True, ...}. Ported from
        correlllab/rh56_controller rh56_hand.adaptive_force_control_iter."""
        if len(target_forces) != Inspire_Num_Motors or len(target_angles) != Inspire_Num_Motors:
            raise ValueError("Need 6 values for both forces and angles")

        if speed is not None:
            self.write_speed([speed] * Inspire_Num_Motors)

        target_forces_arr = np.array(target_forces)
        target_angles_arr = np.array(target_angles, dtype=float)
        step_size_arr = np.array(step_size)
        current_angles = np.array(self.read_angles() or [1000] * Inspire_Num_Motors, dtype=float)
        stop_reason = "max_iterations"

        for iteration in range(max_iterations):
            # Re-assert the firmware force limit so protection stays in place.
            if not self.write_force_limits(target_forces):
                continue
            time.sleep(settle)

            # Step angles toward target, clipped to step_size per iteration.
            step = np.clip(target_angles_arr - current_angles, -step_size_arr, step_size_arr)
            next_angles = current_angles + step
            self.write_angles([int(np.clip(v, 0, 1000)) for v in np.round(next_angles)])
            time.sleep(settle)

            # Read back actual angles (firmware may have stopped a motor at the
            # force threshold, so actual < commanded there).
            readback = self.read_angles()
            current_angles = np.array(readback if readback else next_angles, dtype=float)

            current_forces = self.read_forces()
            if current_forces is None:
                continue

            yield {
                "iteration": iteration + 1,
                "forces": list(current_forces),
                "angles": current_angles.tolist(),
            }

            # Done when every active finger has reached its force threshold.
            all_done = all(
                target_forces_arr[i] == 0 or current_forces[i] >= target_forces_arr[i]
                for i in range(Inspire_Num_Motors)
            )
            if all_done:
                stop_reason = "force_reached"
                break

        yield {
            "done": True,
            "stop_reason": stop_reason,
            "final_forces": self.read_forces(),
            "final_angles": self.read_angles(),
        }

    def read_tactile_peaks(self):
        """Return {region: peak int} (max taxel value per region), or None."""
        tac = self.read_tactile()
        if tac is None:
            return None
        return {r: max(int(g.max()) for g in pads.values()) for r, pads in tac.items()}

    def tactile_grasp(self, margin=30, step_size=40, max_iterations=40, speed=200,
                      close_to=0, settle=0.12, baseline_samples=5,
                      gate_dofs=(0, 1, 2, 3, 4)):
        """Tactile-guided grasp: close the fingers incrementally and freeze each
        one the moment its tactile pad detects contact (peak rises `margin`
        above its zeroed resting baseline). DOFs not in `gate_dofs` (e.g. thumb
        rotation) just drive toward `close_to`. This uses the touch array as the
        stop signal instead of the noisy force sensor, so it is gentle by
        construction. Generator yielding per-iteration state, then a final
        {'done': True, ...}.

        Args:
            margin: tactile rise above baseline that counts as contact.
            step_size: max angle decrement per iteration (closing).
            speed: finger speed 0-1000 (keep low to avoid overshoot).
            close_to: angle to close toward if no contact (0 = fully closed).
            baseline_samples: tactile reads averaged for the resting baseline.
            gate_dofs: DOFs stopped on contact (default: the 4 fingers + thumb bend).
        """
        self.write_speed([speed] * Inspire_Num_Motors)

        # Zero the baseline: peak per region at rest (use the max seen so
        # resting noise does not later read as contact).
        baseline = {}
        for _ in range(max(1, baseline_samples)):
            peaks = self.read_tactile_peaks()
            if peaks is None:
                continue
            for r, v in peaks.items():
                baseline[r] = max(baseline.get(r, 0), v)
            time.sleep(0.02)

        current = np.array(self.read_angles() or [1000] * Inspire_Num_Motors, dtype=float)
        contact = [False] * Inspire_Num_Motors
        stop_reason = "max_iterations"

        for iteration in range(max_iterations):
            peaks = self.read_tactile_peaks()
            if peaks is None:
                continue

            # Update contact flags for gated DOFs.
            for dof in gate_dofs:
                region = DOF_TACTILE_REGION[dof]
                if peaks.get(region, 0) - baseline.get(region, 0) >= margin:
                    contact[dof] = True

            # Step each DOF: frozen on contact, else close toward close_to.
            target = current.copy()
            for dof in range(Inspire_Num_Motors):
                if dof in gate_dofs and contact[dof]:
                    continue  # freeze finger that has made contact
                delta = np.clip(close_to - current[dof], -step_size, step_size)
                target[dof] = current[dof] + delta
            self.write_angles([int(np.clip(v, 0, 1000)) for v in np.round(target)])
            time.sleep(settle)

            readback = self.read_angles()
            current = np.array(readback if readback else target, dtype=float)

            yield {
                "iteration": iteration + 1,
                "peaks": peaks,
                "baseline": dict(baseline),
                "contact": list(contact),
                "angles": current.tolist(),
            }

            # Done when every gated finger has contact, or all reached close_to.
            gated_done = all(contact[d] for d in gate_dofs)
            reached = all(abs(current[d] - close_to) <= step_size for d in range(Inspire_Num_Motors))
            if gated_done:
                stop_reason = "all_contacted"
                break
            if reached:
                stop_reason = "fully_closed"
                break

        yield {
            "done": True,
            "stop_reason": stop_reason,
            "contact": list(contact),
            "final_peaks": self.read_tactile_peaks(),
            "final_angles": self.read_angles(),
        }

    def hybrid_grasp(self, margin=30, force_margin=100, force_ceiling=1000,
                     step_size=40, max_iterations=40, speed=200, close_to=0,
                     settle=0.12, baseline_samples=5, gate_dofs=(0, 1, 2, 3, 4)):
        """Hybrid grasp: stop each finger when EITHER its tactile pad detects
        contact (peak rises `margin` above baseline) OR its measured force rises
        `force_margin` above baseline. Tactile triggers earlier/gentler where a
        taxel exists; force covers the surfaces the sparse tactile array misses.
        A firmware force ceiling (`force_ceiling`, abs grams) is set as a
        hardware backstop. Generator yielding per-iteration state, then final.

        Args:
            margin: tactile rise above baseline = contact.
            force_margin: force rise above baseline = contact.
            force_ceiling: firmware FORCE_SET cap (hardware safety net), grams.
            (other args as tactile_grasp)
        """
        self.write_speed([speed] * Inspire_Num_Motors)
        # Hardware backstop: firmware stops a finger if it ever exceeds this push.
        self.write_force_limits([force_ceiling] * Inspire_Num_Motors)

        # Zero both baselines at rest (both signals have nonzero resting offsets).
        t_base = {}
        f_base = [0] * Inspire_Num_Motors
        n = 0
        for _ in range(max(1, baseline_samples)):
            peaks = self.read_tactile_peaks()
            forces = self.read_forces()
            if peaks is None or forces is None:
                continue
            for r, v in peaks.items():
                t_base[r] = max(t_base.get(r, 0), v)
            f_base = [max(f_base[i], forces[i]) for i in range(Inspire_Num_Motors)]
            n += 1
            time.sleep(0.02)

        current = np.array(self.read_angles() or [1000] * Inspire_Num_Motors, dtype=float)
        contact = [False] * Inspire_Num_Motors
        reason = [None] * Inspire_Num_Motors
        stop_reason = "max_iterations"

        for iteration in range(max_iterations):
            peaks = self.read_tactile_peaks()
            forces = self.read_forces()
            if peaks is None or forces is None:
                continue

            for dof in gate_dofs:
                if contact[dof]:
                    continue
                region = DOF_TACTILE_REGION[dof]
                touch = peaks.get(region, 0) - t_base.get(region, 0) >= margin
                pushed = forces[dof] - f_base[dof] >= force_margin
                if touch or pushed:
                    contact[dof] = True
                    reason[dof] = "touch" if touch else "force"

            target = current.copy()
            for dof in range(Inspire_Num_Motors):
                if dof in gate_dofs and contact[dof]:
                    continue
                delta = np.clip(close_to - current[dof], -step_size, step_size)
                target[dof] = current[dof] + delta
            self.write_angles([int(np.clip(v, 0, 1000)) for v in np.round(target)])
            time.sleep(settle)

            readback = self.read_angles()
            current = np.array(readback if readback else target, dtype=float)

            yield {
                "iteration": iteration + 1,
                "peaks": peaks,
                "forces": list(forces),
                "tactile_baseline": dict(t_base),
                "force_baseline": list(f_base),
                "contact": list(contact),
                "reason": list(reason),
                "angles": current.tolist(),
            }

            gated_done = all(contact[d] for d in gate_dofs)
            reached = all(abs(current[d] - close_to) <= step_size for d in range(Inspire_Num_Motors))
            if gated_done:
                stop_reason = "all_contacted"
                break
            if reached:
                stop_reason = "fully_closed"
                break

        yield {
            "done": True,
            "stop_reason": stop_reason,
            "contact": list(contact),
            "reason": list(reason),
            "final_forces": self.read_forces(),
            "final_angles": self.read_angles(),
        }

    def close(self):
        if self.client:
            try:
                self.client.close()
            except:
                pass
        self.connected = False

