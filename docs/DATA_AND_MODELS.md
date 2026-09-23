# Data and models — G1 restocking task

Task prompt (used verbatim for training and inference): **`put bottles with red cap in red bottle holder`**.
Embodiment tag `UNITREE_G1_SONIC`. Everything lives in `s3://darwin-robot-data` (eu-central-1); local copies noted.

## Cameras — read this first

The dataset's camera names do **not** follow upstream's meaning:

| key in the dataset | what it actually is | at inference |
|---|---|---|
| `observation.images.ego_view` | **side camera** (OBSBOT Tiny 2 Lite on mjolnir), overall view of the table, 640x480 @ 50 fps | sender's primary device |
| `observation.images.head_view` | **G1 head camera** (RealSense D430i, left IR, grayscale replicated to 3 channels), 640x480 | g1 RTP push → sender `--second-device`, `--add-head-camera` at recording |

Models were trained on **both** views (`gear_sonic`-side modality config `rig/gpu/g1_sonic_two_cam.py`, video keys
`[head_view, ego_view]`). The side camera must therefore be present and placed as during recording at inference time.

## Datasets (LeRobot v2.1, 50 fps)

| name | episodes | frames | S3 | produced by |
|---|---|---|---|---|
| `raw/restocking` | 69 (17 flagged discarded during collection) | 156,342 | `s3://darwin-robot-data/raw/restocking/` | `run_data_exporter.py` on mjolnir, 2026-09-08 (SONIC teleop, PICO, Inspire hand state via dump port) |
| `restocking_cleaned` | 52 | 126,791 | `s3://darwin-robot-data/processed/restocking_cleaned/` | `gear_sonic/scripts/process_dataset.py --dataset-path restocking --output-path restocking_cleaned` (removes the 17 discarded episodes + 2,658 stale-SMPL frames in 47 episodes) |
| `restocking_best34` | 34 | 80,895 | `s3://darwin-robot-data/processed/restocking_best34/` | same, plus `--exclude-episodes 1 2 9 12 16 22 24 26 30 31 32 33 37 38 42 47 54 55` (raw indices; hand-picked best-quality takes) |
| `raw/policy_2026-09-23` | 3 | 7,899 | `s3://darwin-robot-data/raw/policy_2026-09-23/` | `RECORD=1 tools/vla-inference.sh` on mjolnir, 2026-09-23: closed-loop attempts of `restocking_best34` (policy actions, `teleop.stream_mode` 6 on every frame; not demonstrations — for evaluation/DAgger, #25) |

Discarded during collection (raw indices): `6 13 14 15 41 48 51 56 57 60 61 62 63 64 65 67 68`.

**best34 = raw episodes** `0 3 4 5 7 8 10 11 17 18 19 20 21 23 25 27 28 29 34 35 36 39 40 43 44 45 46 49 50 52 53 58 59 66`.

Index mapping: a cleaned dataset renumbers survivors contiguously from 0, so *cleaned index = position of the raw index
among the kept episodes* (e.g. raw 10 → cleaned 9 → best34 6). Episode lengths are identical across the three
datasets for the same raw episode, which is the quickest way to cross-check a mapping.

Both processed datasets include `meta/stats.json` and `meta/relative_stats.json` (generated with
`gr00t/data/stats.py --embodiment-tag UNITREE_G1_SONIC`), so Isaac-GR00T can train/eval on them directly.
`meta/episodes_stats.jsonl` from the raw recording is intentionally absent (stale after frame removal; not used).

Known data quirks (present in recordings and at inference alike): the right index finger reads fully closed at its
open end-stop (closure 0.25 at rest); index fingers stall at ~⅓ travel when closing (measured closure ≈ 0.9).

Local copies: darwin-gpu `~/datasets/{restocking_cleaned,restocking_best34}`; Marin's Mac `~/Desktop/restocking{,_cleaned,_best34}`.

## Models

Both are full fine-tunes of `nvidia/GR00T-N1.7-3B` (backbone `nvidia/Cosmos-Reason2-2B`, gated on HF) with
Isaac-GR00T @ `51d4c89` via `rig/gpu/finetune_restocking.sh`: single RTX PRO 6000 (96 GB), 10,000 steps,
global batch 32, lr 1e-4, warmup 5 %, weight decay 1e-5, colour jitter defaults, two-camera modality config,
~80 min each, 41 GB VRAM. Checkpoint format: HF/safetensors, 12.6 GB, weights only.

| model | data | mean train loss | final loss | S3 | darwin-gpu | mjolnir |
|---|---|---|---|---|---|---|
| `restocking_v2` | `restocking_cleaned` (52 ep) | 0.105 | 0.042 | `s3://darwin-robot-data/models/gr00t-n1.7-restocking/restocking_v2/checkpoint-10000/` | `~/checkpoints/restocking_v2/restocking_v2/checkpoint-10000` | `~/checkpoints/restocking_v2/checkpoint-10000` |
| `restocking_best34` | `restocking_best34` (34 ep) | 0.102 | 0.038 | `s3://darwin-robot-data/models/gr00t-n1.7-restocking/restocking_best34/checkpoint-10000/` | `~/checkpoints/restocking_best34/restocking_best34/checkpoint-10000` | `~/checkpoints/restocking_best34/checkpoint-10000` |

Each S3 checkpoint folder carries a `MANIFEST.sha256` (sha256sum format) — verify with `sha256sum -c MANIFEST.sha256` after download
(`rig/thor/s3_pull_checkpoints.py` does both; the mjolnir copies were pulled and verified 2026-09-21).
Loading a checkpoint also fetches the gated `nvidia/Cosmos-Reason2-2B` backbone from Hugging Face on first use
(~4.6 GB into `HF_HOME`), so the serving machine needs an HF token with access (`~/.hf_token` on darwin-gpu and mjolnir).
The checkpoint's `processor_config.json` embeds the two-camera video keys, but `run_gr00t_server.py` still needs
`--modality-config-path rig/gpu/g1_sonic_two_cam.py` to *advertise* them to clients (see `rig/gpu/serve_policy.sh` and
`rig/thor/serve_policy.sh`).

### Inference latency (one 40-step chunk, two 640x480 views, bf16 eager)

| where | GPU | bench | in the runs |
|---|---|---|---|
| darwin-gpu (EC2, via SSH tunnel) | RTX PRO 6000 Blackwell 96 GB | 0.25 s round trip | 0.40–0.54 s (raw frames over the office uplink) |
| mjolnir (local, loopback) | Jetson AGX Thor | 0.145 s median / 0.165 s p95 in-process; 0.15–0.17 s round trip over loopback (2026-09-21) | not yet run on the robot |

### Open-loop evaluation (2026-09-10, `gr00t/eval/open_loop_eval.py`, 2000 steps/episode, execution horizon 16)

Unnormalised action MSE / MAE, averaged over 5 episodes. "Shared" = raw episodes 0, 10, 25, 40, 53 (in both training sets);
"held-out" = raw 1, 2, 9, 12, 16 (in v2's training set only, rated lower quality).

| episodes | dims | v2 | best34 |
|---|---|---|---|
| shared | all 78 (motion token + hands) | 0.0043 / 0.0200 | 0.0034 / 0.0177 |
| shared | hands (14) | 0.0182 / 0.0209 | 0.0161 / 0.0181 |
| held-out | all 78 | 0.0034 / 0.0193 | 0.0080 / 0.0369 |
| held-out | hands | 0.0121 / 0.0158 | 0.0337 / 0.0327 |

Reading: best34 fits the shared episodes ~15 % better; it is ~2× worse on the low-quality takes it never saw (dominated
by raw episode 12), which is not a fair generalisation test since v2 trained on them. Open-loop cannot pick a winner;
see `docs/EXPERIMENTS.md` for the robot runs. Plots: Marin's Mac `~/Desktop/restocking_eval/`.

## Tic-tac-toe (milestone 3, in preparation)

Second task on the same embodiment and recipe. The VLA learns one skill, "put a piece of my colour in
cell X", and a game orchestrator (#43) chooses X. Prompt template (recording and inference, byte for byte):
**`put a white piece in the {cell} cell`**, defined in `gear_sonic/utils/tictactoe/cells.py`. Digits in the
KEYS pane set the prompt for the next episode (#40); board index = digit − 1:

| digit | cell | digit | cell | digit | cell |
|---|---|---|---|---|---|
| 1 | top left | 2 | top center | 3 | top right |
| 4 | middle left | 5 | center | 6 | middle right |
| 7 | bottom left | 8 | bottom center | 9 | bottom right |

The piece colour in the template is provisional until the board and pieces exist (#38): it must separate
from the human's colour in the OBSBOT view *and* in the grayscale IR head view. Datasets: none yet; the
recording plan is #44. Merging sessions with several prompts needs `process_dataset.py` from #41 or later —
older versions relabel episodes when the sessions list the prompts in a different order.

## Run videos

Phone videos of the closed-loop runs: `s3://darwin-robot-data/videos/2026-09-11-vla-runs/` (8 × 4K/60 fps, 4.9 GB).
