"""Unit tests for the hand-tracking retargeter (#35). Synthetic joints, no headset.

    cd ~/GR00T-WholeBodyControl && .venv_teleop/bin/python tools/tests/test_hand_tracking.py
"""
import math
import numpy as np
from gear_sonic.utils.teleop import hand_tracking as ht


def synth_hand(finger_flex_deg=0.0, thumb_flex_deg=0.0, scale=1.0):
    """26x7 joint array of a hand whose finger joints each bend by finger_flex_deg (per joint).
    finger_flex_deg may be a dict {index|middle|ring|little: deg} for per-finger poses."""
    j = np.zeros((26, 7)); j[:, 6] = 1.0
    j[ht.WRIST, :3] = (0, 0, 0)
    j[ht.PALM, :3] = (0.04, 0, 0)

    def chain(base, direction, lengths, per_joint_deg):
        p = np.array(base, dtype=float); ang = 0.0; pts = [p.copy()]
        for i, L in enumerate(lengths):
            if i > 0:
                ang += math.radians(per_joint_deg)
            d = np.array([math.cos(ang) * direction[0] - math.sin(ang) * direction[1],
                          math.sin(ang) * direction[0] + math.cos(ang) * direction[1], 0.0])
            p = p + L * scale * d; pts.append(p.copy())
        return pts

    for k, (name, idx) in enumerate(ht.FINGERS.items()):
        base = (0.03, 0.02 - 0.013 * k, 0)
        flex = finger_flex_deg.get(name, 0.0) if isinstance(finger_flex_deg, dict) else finger_flex_deg
        pts = chain(base, (1, 0), [0.05, 0.04, 0.025, 0.02], flex)   # meta, prox, inter, dist, tip
        for jj, p in zip(idx, pts):
            j[jj, :3] = p
    pts = chain((0.01, 0.03, 0), (0.7, 0.7), [0.04, 0.03, 0.025], thumb_flex_deg)  # meta, prox, dist, tip
    for jj, p in zip(ht.THUMB, pts):
        j[jj, :3] = p
    if thumb_flex_deg >= 60:   # "closed": thumb wraps across the palm, tip next to the little proximal
        j[ht.THUMB[3], :3] = j[ht.LITTLE[1], :3] + np.array([0.0, -0.01, 0.0]) * scale
    return j


# straight fingers -> 0
c = ht.hand_curls(synth_hand(0, 0)); assert c is not None
assert c["fingers"] == 0.0 and c["thumb"] == 0.0, c
# 70 deg per joint -> 210 deg total > FINGER_CLOSED_DEG -> 1.0 ; thumb tip across the palm -> ratio small -> 1.0
c = ht.hand_curls(synth_hand(70, 60)); assert c["fingers"] == 1.0 and c["thumb"] == 1.0, c
# mid flexion -> in between, monotonic, scale-invariant
c1 = ht.hand_curls(synth_hand(30, 20)); c2 = ht.hand_curls(synth_hand(45, 30))
assert 0.0 < c1["fingers"] < c2["fingers"] < 1.0, (c1, c2)
assert c1["thumb"] == 0.0 and c2["thumb"] == 0.0                       # thumb away from the palm stays open
# real PICO numbers from the 2026-09-21 probe: relaxed 26-53 deg -> 0, fist 142-190 -> ~1
assert ht._normalize(53.0, ht.FINGER_OPEN_DEG, ht.FINGER_CLOSED_DEG) == 0.0
assert ht._normalize(142.0, ht.FINGER_OPEN_DEG, ht.FINGER_CLOSED_DEG) >= 0.9
assert ht._normalize(169.0, ht.FINGER_OPEN_DEG, ht.FINGER_CLOSED_DEG) == 1.0
# thumb ratio from the probe: open hand 1.4-1.6 -> 0, natural fist 0.8-1.0 -> mostly closed, folded 0.55 -> 1
assert ht._normalize(1.4, ht.THUMB_OPEN_RATIO, ht.THUMB_CLOSED_RATIO) == 0.0
assert ht._normalize(0.9, ht.THUMB_OPEN_RATIO, ht.THUMB_CLOSED_RATIO) >= 0.85
assert 0.5 < ht._normalize(1.0, ht.THUMB_OPEN_RATIO, ht.THUMB_CLOSED_RATIO) < 0.9
assert ht._normalize(0.55, ht.THUMB_OPEN_RATIO, ht.THUMB_CLOSED_RATIO) == 1.0
assert abs(ht.hand_curls(synth_hand(30, 20, scale=0.5))["fingers"] - c1["fingers"]) < 1e-9
# untracked hand: zeros -> None (no NaN), wrong shape -> None, NaN -> None
assert ht.hand_curls(np.zeros((26, 7))) is None
assert ht.hand_curls(np.zeros((25, 7))) is None
bad = synth_hand(); bad[5, 0] = float("nan"); assert ht.hand_curls(bad) is None

# ---- provider: hand / hold / controller fallback with a fake clock
state = {"t": 0.0, "hand": synth_hand(70, 60), "active": 1, "ctrl": (False, 0.0, 0.2, 0.0, 0.3)}
prov = ht.HandTrackingInputs(lambda: state["ctrl"], lambda side: (state["hand"], state["active"]),
                             alpha=1.0, hold_s=0.5, clock=lambda: state["t"])
menu, lt, rt, lg, rg = prov()
assert (lt, rt, lg, rg) == (1.0, 1.0, 1.0, 1.0) and prov.sources == {"left": "hand", "right": "hand"}, (lt, rt, lg, rg)
# tracking lost: hold the last value for hold_s, then fall back to the controllers
state["active"] = 0; state["t"] = 0.3
_, lt, rt, lg, rg = prov(); assert (lt, rt) == (1.0, 1.0) and prov.sources["left"] == "hold"
state["t"] = 1.0
_, lt, rt, lg, rg = prov(); assert (lt, rt, lg, rg) == (0.0, 0.2, 0.0, 0.3) and prov.sources["left"] == "controller", (lt, rt, lg, rg)
# tracking back -> hand again; reader exception -> treated as not tracked
state["active"] = 1; state["t"] = 1.1
assert prov()[1] == 1.0 and prov.sources["left"] == "hand"
def boom(side): raise RuntimeError("sdk down")
prov2 = ht.HandTrackingInputs(lambda: (False, 0.5, 0.5, 0.1, 0.1), boom, clock=lambda: 0.0)
assert prov2()[1:] == (0.5, 0.5, 0.1, 0.1)
# smoothing: alpha 0.5 from open to fist gives 0.5 after one step, then snaps to 1 above SNAP_HIGH
state3 = {"hand": synth_hand(0, 0)}
prov3 = ht.HandTrackingInputs(lambda: (False, 0, 0, 0, 0), lambda s: (state3["hand"], 1), alpha=0.5, clock=lambda: 0.0)
prov3(); state3["hand"] = synth_hand(70, 60)
assert abs(prov3()[1] - 0.5) < 1e-9
assert abs(prov3()[1] - 0.75) < 1e-9
assert abs(prov3()[1] - 0.875) < 1e-9   # still below SNAP_HIGH
assert prov3()[1] == 1.0                  # 0.9375 >= SNAP_HIGH -> snapped to 1.0

# ---- per-finger targets (phase 2): only the index bent -> only the index DOF closes
one = synth_hand({"index": 70}, 0)
c = ht.hand_curls(one); assert c["index"] == 1.0 and c["middle"] == 0.0 and c["little"] == 0.0, c
st = {"hand": one}
p4 = ht.HandTrackingInputs(lambda: (False, 0, 0, 0, 0), lambda s: (st["hand"], 1), alpha=1.0, clock=lambda: 0.0)
p4()
ft = p4.finger_targets()
assert ft["left"] == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], ft          # [little, ring, middle, index, thumb_bend, thumb_rot]
assert ft["right"] == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
# side on its controller -> None (bridge falls back to trigger mapping)
p5 = ht.HandTrackingInputs(lambda: (False, 0.3, 0, 0, 0), lambda s: (np.zeros((26, 7)), 0), clock=lambda: 0.0)
p5(); assert p5.finger_targets() == {"left": None, "right": None}
# bridge mapping of closures -> angles, and per-finger force rule (pure static methods)
from gear_sonic.utils.teleop.inspire import inspire_bridge as ib
ang = ib.InspireBridge._targets_from_closures([0.0, 0.0, 0.0, 1.0, 0.5, None])
assert ang == [ib.OPEN, ib.OPEN, ib.OPEN, 0, ib.OPEN * 0.5, ib.THUMB_ROT_REST], ang
base = [100] * 6
lim = ib.InspireBridge._force_set_for_inputs(base, 0.0, 0.0, closures=[0.0, 0.0, 0.0, 1.0, 0.0, None])
cruise = ib.InspireBridge._force_set_from_baseline(base)
assert lim[3] == ib.FORCE_SET_MAX_G and lim[0] == cruise[0] and lim[4] == cruise[4], lim
lim2 = ib.InspireBridge._force_set_for_inputs(base, 1.0, 1.0)      # trigger path unchanged
assert all(lim2[i] == ib.FORCE_SET_MAX_G for i in range(5)), lim2

# ---- thumb rotation: the synthetic hand is planar (thumb in the palm plane) -> angle 90 -> closure 0
flat = synth_hand(0, 0)
assert abs(ht.thumb_rotation_angle_deg(flat) - 90.0) < 1e-6
assert ht.hand_curls(flat)["thumb_rot"] == 0.0
# lift the thumb distal joint out of the palm plane along the normal -> small angle -> closure 1
opp = synth_hand(0, 0); n = ht.palm_normal(opp[:, :3])
opp[ht.THUMB[2], :3] = opp[ht.THUMB[1], :3] + 0.03 * n
assert ht.thumb_rotation_angle_deg(opp) < 1e-6
assert ht.hand_curls(opp)["thumb_rot"] == 1.0
st6 = {"hand": opp}
p6 = ht.HandTrackingInputs(lambda: (False, 0, 0, 0, 0), lambda s: (st6["hand"], 1), alpha=1.0, clock=lambda: 0.0)
p6(); assert p6.finger_targets()["left"][5] == 1.0
import os
os.environ["HAND_TRACKING_THUMB_ROT"] = "0"
p7 = ht.HandTrackingInputs(lambda: (False, 0, 0, 0, 0), lambda s: (st6["hand"], 1), alpha=1.0, clock=lambda: 0.0)
p7(); assert p7.finger_targets()["left"][5] is None
del os.environ["HAND_TRACKING_THUMB_ROT"]
print("HAND_TRACKING_TEST_OK")
