# Experiment log

Newest first. Datasets and checkpoints referenced here are described in `DATA_AND_MODELS.md`. Open work that
follows from these sessions is tracked in the milestone
[Restocking VLA: from first closed loop to reliable placement](https://github.com/prosus-robotics/g1-vr-teleop/milestone/1);
the teleop/demo phase that produced the recordings is the closed milestone
[Bartending demo, AI House (Aug 2026)](https://github.com/prosus-robotics/g1-vr-teleop/milestone/2).

## 2026-09-23 — first closed-loop attempts recorded as LeRobot episodes (#25)

**Setup.** `RECORD=1 DATASET=policy_2026-09-23 tools/vla-inference.sh up` (branch `dagger-interventions`): the usual
VLA stack (local server, `best34`, training prompt) plus the ZMQ exporter in the wbc-marin container and
`record_keys_zmq.py` as the keys pane. No headset, no streamer: the exporter labels frames `stream_mode` 6 (POLICY) on
its own when it sees VLA tokens on the `pose` topic and no `manager_state`. Three attempts, `c` / `c` per attempt.

**Result.** 3 episodes, 7,899 frames (614 / 3,093 / 4,192 = 12 s / 62 s / 84 s), all frames POLICY, `action.motion_token`
non-zero on every frame (|token| mean 0.11–0.13), policy hand action populated, Inspire hand state on every frame (one
262 ms stale fallback in episode 1), clean 20 ms grid with no gap > 40 ms, both videos 640x480 @ 50 fps with frame counts
equal to the parquet lengths, all 14 files in `s3://darwin-robot-data/raw/policy_2026-09-23/` (resync dry-run: in sync).
Exporter log clean (no traceback, no discard, no recovery). First live confirmation of the recording half of #25; the
DAgger half (interventions from the PICO, `DAGGER=1`) is implemented but not yet run on the robot.

**Found on the way.** The same morning a `tictactoe_test` teleop session recorded nothing: a discard on an empty
episode buffer raised inside lerobot and killed the exporter, so every later gesture went nowhere. Fixed in the
exporter (empty discard = back to IDLE; any exception in a tick is logged and recovered from) and the exporter pane is
now kept in `logs/exporter-last-run.log` by both launchers.

## 2026-09-21 — policy server moved onto the Thor (mjolnir), latency benchmark (#19, #27)

**Setup.** Isaac-GR00T @ `51d4c89` installed on mjolnir (Jetson AGX Thor, JetPack 7.1 / CUDA 13.0, Python 3.12) with
the JetPack 7.1 dependency set upstream shipped at `e574928` (torch 2.10 cu130, flash-attn 2.8.4 from the Jetson AI
Lab index), see `rig/thor/`. Both checkpoints pulled from S3 (`MANIFEST.sha256` OK). Benchmark = `rig/thor/bench_policy.py`:
`Gr00tPolicy.get_action` in-process, bf16 eager, one real two-camera observation (`synthetic_obs_ep0_f100.npz`,
640x480 head + side view), 3 warm-up + 20 timed calls, robot stack down (GPU otherwise idle).

**Result.**

| measurement (restocking_best34, Thor, bf16 eager) | value |
|---|---|
| `Gr00tPolicy.get_action`, 20 calls in-process | **median 145 ms**, p95 165 ms, min 142 ms, max 165 ms |
| round trip through `run_gr00t_server.py` on loopback (`policy_smoke_test.py --n 10`) | 0.145–0.166 s; first call after start 0.77 s (warm-up) |
| peak GPU memory | 6.0 GiB allocated / 6.1 GiB reserved (unified 122 GB) |
| server start to ZMQ bind (backbone cached) | 56 s |
| sanity on the real frame | motion token \|max\| 0.36 (bound 1.25), MSE vs recorded next-40 0.0009 (recorded vs own mean 0.0006) |

Budget is 0.4 s per 40-step chunk (2.5 Hz), target < 0.3 s: **under budget by ~2.5×**, so the local server becomes the default. Compare darwin-gpu (RTX PRO 6000):
0.25 s round trip on the bench, 0.40–0.54 s in the real runs because two raw frames crossed the office uplink per request.
Locally the frames never leave the Thor, so the run-time number should sit at the bench value.

**Decision.** `tools/vla-inference.sh` now starts the server on mjolnir by default (`POLICY_MODE=local`, pane `serve`
replaces the tunnel; the inference pane waits for the port because the client reads the camera keys from the server once
at start-up). EC2 stays as `POLICY_MODE=ec2`. #20 (JPEG frames over the tunnel) is only needed for the fallback path.

**Verified on the robot the same afternoon.** Marin ran the closed loop with the local server (`POLICY_MODE=local`,
`best34`): it worked, the deploy's TensorRT controller and the 3B policy coexist on the Thor GPU without visible
trouble. In-run chunk latency was not captured (the tmux session was gone before the log could be read); expect ~0.15 s
per the bench and record it next time. `nvpmodel -q` / `jetson_clocks --show` still to log (MAXN configured). #27 closed.

## 2026-09-11 — first closed-loop VLA runs on the G1 (restocking)

**Setup.** Real robot on the hoist, then feet on the ground at the table. Policy server on darwin-gpu (SSH tunnel,
5550), SONIC deploy + VLA client + Inspire bridge on mjolnir (`tools/vla-inference.sh`). Both cameras live
(side OBSBOT + head D430i). Prompt `put bottles with red cap in red bottle holder`. Both checkpoints were used during
the session — `restocking_best34` first, `restocking_v2` after ~13:20 — but the phone videos were not labelled per
policy, so the outcomes below are pooled.

**Runs.** 8 videos, `s3://darwin-robot-data/videos/2026-09-11-vla-runs/` (times are local, from the file metadata):

| video | len | scene | what happened |
|---|---|---|---|
| IMG_0014 | 53 s | Coke far left | reach, grasp (~30 s), lift, carry toward the holders |
| IMG_0018 | 79 s | Coke | early grasp, then ~40 s hovering with the bottle raised, moves toward holders only at the end |
| IMG_0020 2 | 70 s | Coke near holders | clean grasp (13 s), long hover above the red holders, lowers toward them in the last 10 s |
| IMG_0020 | 122 s | Coke | grasp, ~1 min hover, bottle brought down near the holders around 100 s |
| IMG_0022 | 73 s | Coke + Fanta distractor | ignores the Fanta, grasps the Coke (~45 s), holds it up |
| IMG_0024 | 84 s | Coke + Fanta | reaches; bottle reset by hand around 55 s; a Coke ends up in a red holder (reset or robot — unclear) |
| IMG_0029 | 98 s | Coke | grasp, high lift; at ~67 s a Coke stands in a red holder with the hand above it — the closest thing to a full success |
| IMG_0033 | 107 s | Coke + Fanta | grasps the Coke (~47 s), lifts, holds it high for the last 30 s without placing |

**Result.** Approach and grasp of the red-capped bottle worked in essentially every attempt, including with the
Fanta present (correctly ignored). Placement did not: after lifting, the robot **hovers with the bottle raised**
for tens of seconds, and the **wrist motors overheat and fault** before the placement motion completes (Marin's
observation at the robot; the hover is the cause, the wrist fault the symptom). No clean, unassisted placement was
recorded.

**Measurements.**
- Policy round trip during the runs: 0.40–0.54 s per 40-step chunk (bench value with the same server: 0.25 s).
  Each request carries two raw 640x480 frames (~1.8 MB) through the office uplink; roughly half of each chunk is
  discarded as stale by the latency compensation.
- C++ loop healthy throughout: LowState age 4–5 ms, policy 2.2 ms, motor command 4 ms.
- Camera timestamps: side vs head skew ≤ 53 ms.
- Live side-camera view is farther/wider than in the recordings (room and window visible, table smaller) —
  see `~/Desktop/restocking_eval/live/compare_training_top_vs_live_bottom.jpg` on Marin's Mac.

**Problems hit and fixed during the session** (all now in the RUNBOOK):
1. IGMP querier not running → deploy died at `k` with `LowState or IMUState is not available`.
2. The VLA client's `k` was a start/stop toggle keyed on its own memory; after a deploy restart it sent STOP → replaced by `k` = start, `x` = stop.
3. Head camera (D430i) dropped off g1's USB bus → no `head_view` → every observation skipped. Fixed by unplug/replug.
4. Restoring the head-camera push while the loop was live made the robot start moving unprompted — pause (`p`) before restoring any sensor.

**Hypotheses for the hover, in test order.** (a) Side-camera viewpoint mismatch — placement needs the fine
alignment learned from the recorded viewpoint. (b) Latency — 0.4–0.5 s chunks make chunked policies hesitant.
(c) The demonstrations themselves contain long pauses before placement.

**Next** (each item is an issue in milestone 1): systemd unit for the IGMP querier (#18); pause-after-sensor-loss and
head-camera watchdog (#26); wrist temperature readout, cool-down, gains (#21); benchmark the model on the Thor (#19) and
then either a local policy server (#27) or JPEG frames over the tunnel (#20) — done 2026-09-21, see above; scored comparison of the two checkpoints
(#23); record every attempt as an episode (#25); wrist cameras and mounts (#28, #29) and a rigid head-camera mount (#30).

## 2026-09-10 — open-loop evaluation and second model

Trained `restocking_best34` (34 hand-picked episodes) with the same recipe as `restocking_v2`; mean train loss
0.102 vs 0.105. Open-loop eval of both on shared and held-out episodes — numbers in `DATA_AND_MODELS.md`. Outcome:
best34 fits its training episodes slightly better, is worse on low-quality takes it never saw; no winner offline.

## 2026-09-09/10 — first fine-tune

`restocking_v2` on the 52 cleaned episodes, 10k steps on a single RTX PRO 6000 (81 min). The first attempt filled
the root disk with 25 GB full checkpoints at step 5000 and a resume attempt hung the instance; rerun from scratch
with weights-only checkpoints. Open-loop MSE 0.0047 (all dims) on 5 training episodes.

## 2026-09-08 — data collection and cleaning

69 teleop episodes recorded on mjolnir (`restocking`), 17 flagged discarded during collection. Cleaned to 52
episodes with `process_dataset.py` (discarded episodes + stale SMPL frames removed).
