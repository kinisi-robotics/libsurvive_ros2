# libsurvive patch series

CMake fetches upstream [cntools/libsurvive](https://github.com/cntools/libsurvive)
at the pinned commit in `CMakeLists.txt` and applies these patches in order
(`git apply`, after resetting the checkout to the pin, so the step is idempotent).

| Patch | Origin | Why we carry it |
|---|---|---|
| `0001-kalman-cap-process-noise-dt…` | upstream `bf27b41` | Bounds the Kalman process-noise growth on IMU gaps (t^7 blow-up → NaN filter). Upstream says it fixes tracker freezes on blackout recovery. Cherry-picked as low-risk hardening; not shown to be *our* wedge. |
| `0002-variance_measure_add-skip-non-finite…` | upstream `d63f79f` | A non-finite optical sample used to `assert()` and kill the process mid-config-write, corrupting the calibration file. |
| `0003-kinisi-freeze-calibration…` | ours | Three changes, see below. |

## 0003 — record-mode calibration freeze, OOTX conflicts, light relock

* **`disable-calibrate` freezes the calibration for real.** Previously the stock
  library still rewrote the config file on every start and on every lighthouse
  filter nudge (1e-8 pose changes, `unlock_count` reset to 0), and an OOTX packet
  from a *foreign* base station on one of our channels replaced that slot's id
  and calibration data and cleared its position, after which the poser re-solved
  every lighthouse. With the patch: `config_save()` is a no-op, lighthouse pose
  updates that differ from the stored pose are dropped, and a foreign OOTX id is
  recorded in `BaseStationData::ootx_conflict_id/count` (surfaced as an ERROR on
  that lighthouse's diagnostic row) instead of being applied.
* **Light relock self-heal** (`--light-relock-timeout <s>`, default off in the
  library, 3 s from the ROS node). A live LH2 device that reports IMU but no
  lightcap data for that long gets the lightcap mode switch re-sent — the same
  USB feature report a fresh process sends at startup, which is the only thing
  known to recover the tracker after a total-occlusion wedge. Counted in
  `SurviveObject::stats.light_relocks`. `--light-relock-force-interval <s>` is a
  test knob that relocks periodically so the cost of a relock during tracking can
  be measured.

Regenerate after editing the series: keep a branch on the pinned commit and run
`git format-patch <pin>..<branch> -o patches/`.
