# Place your ARGoS experiment configs here

Copy from `~/argos3-examples/experiments/`:

- `secureflock_single.argos`   (single-goal threat; used by the sweeps)
- `secureflock.argos`          (base / multi-survivor)
- `secureflock_comms.argos`    (partial-sensing / comms-necessity)
- `secureflock_noise.argos`    (packet loss + localisation noise)

Then delete this PLACEHOLDER.md.

Note: the sweep scripts in `scripts/` edit `secureflock_single.argos`
in place via `sed`. Commit a clean copy first so you can `git checkout`
it back after a sweep.
