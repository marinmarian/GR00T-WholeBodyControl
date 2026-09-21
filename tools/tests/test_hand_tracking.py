"""Unit tests for the hand-tracking retargeter (#35). Synthetic joints, no headset.

    cd ~/GR00T-WholeBodyControl && .venv_teleop/bin/python tools/tests/test_hand_tracking.py
"""
import math
import numpy as np
from gear_sonic.utils.teleop import hand_tracking as ht


def synth_hand(finger_flex_deg=0.0, thumb_flex_deg=0.0, scale=1.0):
    """26x7 joint array of a hand whose finger joints each bend by finger_flex_deg (per joint)."""
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
        pts = chain(base, (1, 0), [0.05, 0.04, 0.025, 0.02], finger_flex_deg)   # meta, prox, inter, dist, tip
        for jj, p in zip(idx, pts):
            j[jj, :3] = p
    pts = chain((0.01, 0.03, 0), (0.7, 0.7), [0.04, 0.03, 0.025], thumb_flex_deg)  # meta, prox, dist, tip
    for jj, p in zip(ht.THUMB, pts):
        j[jj, :3] = p
    return j


# straight fingers -> 0
c = ht.hand_curls(synth_hand(0, 0)); assert c is not None
assert c["fingers"] == 0.0 and c["thumb"] == 0.0, c
# 70 deg per joint -> 210 deg total > FINGER_CLOSED_DEG -> 1.0 ; thumb 60/joint -> 120 > 100 -> 1.0
c = ht.hand_curls(synth_hand(70, 60)); assert c["fingers"] == 1.0 and c["thumb"] == 1.0, c
# mid flexion -> in between, monotonic, scale-invariant
c1 = ht.hand_curls(synth_hand(30, 20)); c2 = ht.hand_curls(synth_hand(45, 30))
assert 0.0 < c1["fingers"] < c2["fingers"] < 1.0, (c1, c2)
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
print("HAND_TRACKING_TEST_OK")
