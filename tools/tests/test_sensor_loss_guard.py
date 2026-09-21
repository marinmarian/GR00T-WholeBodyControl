"""Unit test for the sensor-loss guard (g1-vr-teleop #26). No robot, no sockets: fake clock only.

    cd ~/GR00T-WholeBodyControl && .venv_inference/bin/python tools/tests/test_sensor_loss_guard.py
"""
from gear_sonic.scripts.run_vla_inference import SensorLossGuard

g = SensorLossGuard(threshold_s=1.0)
# unarmed: no valid observation ever -> never fires, however long we wait
assert g.update(now=100.0, paused=False) is None
# armed by the first valid observation
g.observation_valid(0.0)
assert g.update(0.5, paused=False) is None            # fresh
assert g.update(1.0, paused=False) is None            # at the threshold, not past it
assert g.update(1.3, paused=False) == "pause"         # past it -> pause once
assert abs(g.outage_s - 1.3) < 1e-9
assert g.update(1.5, paused=True) is None             # still dead: quiet
assert g.update(5.0, paused=True) is None
# camera back: one 'recovered' message, then quiet, still paused
g.observation_valid(5.2)
assert g.update(5.3, paused=True) == "recovered"
assert g.update(5.4, paused=True) is None
# operator resumes with p -> guard re-armed; sensors fine -> nothing
g.operator_resumed()
assert g.update(5.5, paused=False) is None
# operator pause with p while sensors die: the guard does not interfere while paused
assert g.update(9.0, paused=True) is None
# ...but resuming with a still-dead sensor re-trips on the next tick
g.operator_resumed()
assert g.update(9.1, paused=False) == "pause"
# auto-resume variant
a = SensorLossGuard(threshold_s=1.0, auto_resume=True)
a.observation_valid(0.0)
assert a.update(2.0, paused=False) == "pause"
a.observation_valid(3.0)
assert a.update(3.1, paused=True) == "resume"
assert a.update(3.2, paused=False) is None
# disabled
d = SensorLossGuard(threshold_s=0.0); d.observation_valid(0.0)
assert d.update(100.0, paused=False) is None
print("SENSOR_LOSS_GUARD_TEST_OK")
