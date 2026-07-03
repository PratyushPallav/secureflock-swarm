# Physically-Grounded Trust for Byzantine-Resilient Swarm Coordination

Simulation code accompanying the paper:

> **Physically-Grounded Trust for Byzantine-Resilient Swarm Coordination: Defeating Colluding Agents in Simulated Search-and-Rescue.** Pratyush Pallav. *(Under review, Frontiers in Robotics and AI.)*

This repository contains the ARGoS controller, loop functions, experiment
configurations, parameter-sweep scripts, and plotting code needed to reproduce
the results in the paper. A colluding in-swarm adversary broadcasts a shared
false goal; each uninformed robot filters neighbours' goal claims by whether
the claimed **bearing** agrees with the bearing of an independently sensed
physical beacon, before trust-weighted aggregation. Because the beacon is
external to the swarm's communication, collusion cannot forge it.

## What's here

```
controllers/footbot_secureflock/    per-robot controller (bearing filter + trust aggregation)
loop_functions/secureflock_loop_functions/  scenario setup, role assignment, logging
experiments/                        ARGoS .argos configuration files
scripts/                            parameter-sweep drivers (bash)
plotting/                           figure-generation scripts (Python)
data/                               sweep output CSVs land here (generated; git-ignored)
```

## Requirements

- **ARGoS3** v3.0.0-beta59 (with `footbot` and QT-OpenGL plugins)
- Ubuntu 22.04, GCC/CMake
- Python 3 with `pandas numpy matplotlib` (`pip install -r plotting/requirements.txt`)

## Build

```bash
mkdir build && cd build
cmake ..
make
cd ..
```

This produces the controller and loop-function shared libraries under `build/`.

## Reproduce the results

Each sweep writes a CSV into `data/`; the matching plotting script renders the
figure. The sweeps edit `experiments/secureflock_single.argos` in place, so
restore it with `git checkout experiments/secureflock_single.argos` between runs.

| Figure | Sweep | Plot |
|---|---|---|
| Coverage comparison (headline) | `scripts/sweep_compare_coverage.sh` | `plotting/plot_coverage.py` |
| Parameter sensitivity | `scripts/sweep_sensitivity.sh 0.80` | `plotting/plot_sensitivity.py` |
| Same-bearing blind spot (limitation) | `scripts/sweep_bearing_limitation.sh` | `plotting/plot_bearing_limitation.py` |

Example:

```bash
bash scripts/sweep_compare_coverage.sh        # ~90 runs -> data/sweep_compare_coverage.csv
python3 plotting/plot_coverage.py             # -> coverage_comparison.png
```

The `secureflock_comms.argos` (partial sensing), `secureflock_noise.argos`
(packet loss + localisation noise), and multi-survivor rescue experiments are
run analogously; see the paper's Methods for the exact parameter values.

## Key parameters

Set in the controller's `<trust>` node and the `<secureflock>` loop-function node:

| Parameter | Meaning |
|---|---|
| `ground` | enable beacon-bearing grounding |
| `ground_fraction` | fraction of naive robots that can sense the beacon (−1 = all) |
| `agree` | agreement-cone width (`0.0` = 90°, `0.5` = 60°) |
| `tolerance` | trust-update tolerance (σ) |
| `method` | `trust` or `wmsr` (baseline) |
| `attacker` | `follow` (embedded, adaptive) or `static` |
| `malicious_fraction` | fraction of colluding adversaries |
| `informed_fraction` | fraction of informed scouts |
| `false_x`, `false_y` | broadcast phantom location |
| `packet_loss`, `pos_noise` | robustness stressors |

## Logging

Each run appends to `secureflock_log.csv`. Columns (single-survivor):
`tick, cov_s0, cohesion, phantom, n_naive, naive_at_surv, naive_at_phantom, naive_est_err`.
`naive_est_err` (field 8) is the belief metric used throughout the paper
(0 = belief on the true goal; 8 = full flip to the phantom).

## Citation

```bibtex
@article{pallav2026grounded,
  author  = {Pallav, Pratyush},
  title   = {Physically-Grounded Trust for Byzantine-Resilient Swarm
             Coordination: Defeating Colluding Agents in Simulated
             Search-and-Rescue},
  journal = {Frontiers in Robotics and AI},
  year    = {2026},
  note    = {Under review}
}
```

## License

Released under the MIT License (see `LICENSE`). The controller and loop
functions derive from the `argos3-examples` template; original license and
attribution headers in those files are retained.
