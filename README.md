# g1-vr-teleop

Prosus Robotics work on the **Unitree G1 humanoid with Inspire RH56 hands**: SONIC whole-body teleoperation
over a PICO VR headset, episode recording, and — since September 2026 — closed-loop **VLA inference** with a
fine-tuned NVIDIA GR00T N1.7 policy.

The repository is an *overlay* on [NVlabs GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl):
it holds only the files we patch or add, at the same paths, so it applies over an upstream checkout. Provenance and
the exact upstream base are in [`VENDOR.md`](VENDOR.md).

## What is here

| | |
|---|---|
| [`RUNBOOK.md`](RUNBOOK.md) | How to run everything on the rig: teleop + recording, the **VLA inference** operator guide (pre-flight, key sequence, gotchas), and the **DAgger** stack (record policy attempts, intervene from the PICO; experimental). Start here. |
| [`CUSTOM_SETUP.md`](CUSTOM_SETUP.md) | How the rig was built: network, hotspot, Inspire hands, PICO/Quest paths, IK stack, head camera. |
| [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md) | Experiment log — what we ran on the robot and what happened. |
| [`docs/DATA_AND_MODELS.md`](docs/DATA_AND_MODELS.md) | Registry of datasets and trained checkpoints (episode lists, training configs, eval numbers, S3 paths). |
| `gear_sonic/`, `decoupled_wbc/` | Patched upstream code: Inspire hand bridge + state recording, ZMQ exporter, VLA inference client, hand bridge for the policy, policy relay / intervention modes in the PICO streamer (`gear_sonic/utils/teleop/policy_relay.py`). |
| `tools/` | Launchers (`sonic-teleop.sh`, `vla-inference.sh`), keyboard publishers, head-camera sender for the PICO. |
| [`rig/`](rig/README.md) | Host-side scripts for the operator Jetson (mjolnir); [`rig/thor/`](rig/thor/README.md) installs Isaac-GR00T on the Thor and serves the policy locally; [`rig/gpu/`](rig/gpu/README.md) holds the GPU-box scripts that train, evaluate and (as fallback) serve the policy. |

## The restocking task in one paragraph

We teleoperated the G1 to put red-capped bottles into a red holder (~70 episodes), cleaned the data, and fine-tuned
GR00T N1.7 on it using both the head camera and a side camera; a second model was trained on the 34 best episodes.
The policy runs on a cloud GPU; the robot's Jetson streams images and joint state to it and executes the returned
motion tokens and hand commands through the SONIC controller. The full loop ran on the real robot on 2026-09-11:
grasping works, placement is blocked by a long hover that overheats the wrist motors — see the experiment log.

## Work tracking

Issues are grouped into [milestones](https://github.com/prosus-robotics/g1-vr-teleop/milestones), one per phase:

| milestone | status | what |
|---|---|---|
| [Bartending demo, AI House (Aug 2026)](https://github.com/prosus-robotics/g1-vr-teleop/milestone/2) | closed | SONIC VR teleop with Inspire hands for the demo: cameras, Sonic Fast validation, bottle handover, recording, venue; plus the PRs that brought the code into this repo (#1–#14). |
| [Restocking VLA: from first closed loop to reliable placement](https://github.com/prosus-robotics/g1-vr-teleop/milestone/1) | open | GR00T N1.7 restocking policy: deployment PRs (#15–#17, done) and the open work — infra/safety fixes, latency, wrist thermal, scored model comparison, recording attempts, wrist/head camera hardware. |

New issues for an effort get its milestone when created; finished work is recorded in the PRs and in `docs/EXPERIMENTS.md`, not as retroactive issues.

## Machines

- **mjolnir** — Jetson AGX Thor on the robot network, operator machine (teleop, cameras, deploy, VLA client).
- **g1** — the robot's onboard PC (head camera push).
- **darwin-gpu** — EC2 GPU instance: fine-tuning and evaluation; fallback GR00T policy server (reached through an SSH tunnel). Since 2026-09-21 the policy server runs on mjolnir by default.
- Bucket `s3://darwin-robot-data` — raw and processed datasets, checkpoints, run videos.
