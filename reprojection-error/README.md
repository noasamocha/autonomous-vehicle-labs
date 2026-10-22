# Reprojection Error Measurement and Calibration Comparisons

A LiDAR–camera calibration fits its own observations, but the real question is how well it projects **new** data. In this lab I implemented the core math of an offline reprojection-error tool. I then used the tool to test hypotheses about which calibration data generalizes.

## Method (`reprojection.py`)

Every candidate calibration is scored on a fixed, held-out measurement set: 76 Captures and 304 board corners, recorded separately on the same rig. For each corner:

1. **Transform** the LiDAR-frame corner into the camera optical frame: `p_C = R · p_L + t`. `R` comes from the candidate's Rodrigues vector, and the transform already goes LiDAR → camera, so it is not inverted.
2. **Project** it with the camera intrinsics: `[u_h, v_h, w_h]ᵀ = K · p_C`, pixel = `(u_h / w_h, v_h / w_h)`. The observed corners are already undistorted.
3. **Error:** the Euclidean pixel distance between the projected and the observed corner.
4. **Pool** the errors into one RMS: `sqrt(Σ eᵢ² / N)`. Every corner counts once.

`reprojection.py` implements these four functions, vectorized with NumPy. The data loading, CLI and tests around it were supplied by the course.

## Experiments

All candidates use the same recording, solver settings and measurement set. Every metric is in pixels over the same 76 Captures.

### 1. Number of placements

Hypothesis: more board placements give a lower error.

| Calibration | Placements | RMS | Mean | Median | Max |
| --- | --- | --- | --- | --- | --- |
| T6 | 6 | 24.81 | 21.39 | 19.14 | 84.07 |
| T4 | 4 | 36.34 | 32.02 | 30.04 | 126.98 |
| T2 | 2 | 57.74 | 35.61 | 19.49 | 259.28 |
| T1 | 1 | **20.81** | 14.97 | 9.84 | 87.32 |

- **From 2 to 6 placements, the error drops as expected.**
- **T1 is surprisingly the best.** Its single board is far away, which may help with the rotation, and 40 of the 76 measurement boards are also 3.5 m or further. Still, one placement does not constrain the translation reliably: T1's camera position was noticeably off from T6's.
- **Overfitting.** LCTK reported the lowest *fit* errors for T1 (0.12 px) and T2 (4.5 px), and even marked T2 as well constrained. Yet T2 had the worst measured RMS. A small error on a calibration's own data says little about new data: with so few placements, the solver fits its own Captures.

### 2. Board distance

Hypothesis: far boards give a lower error, by constraining rotation better.

Each set has 3 placements with similar board angles.

| Set | Distance | RMS | Mean | Median | Max |
| --- | --- | --- | --- | --- | --- |
| Near | ~2.1 m | 52.10 | 46.08 | 44.35 | 179.45 |
| Far | 4.0–4.3 m | **25.20** | 14.70 | 7.10 | 121.49 |

The far set roughly halves the RMS. The two sets also differ slightly in lateral spread, and half of the measurement boards are far too, so distance is likely, but not proven, to be the only cause.

### 3. Number of Captures per placement

Hypothesis: more Captures from the same placements average out noise.

| Pair | Captures | RMS |
| --- | --- | --- |
| A: three near positions | 3 → 9 | 52.10 → 43.62 |
| B: three near positions + far right | 4 → 11 | 52.10 → **19.10** |

- **Pair A** isolates the Capture count, and shows only a small gain.
- **Pair B** improves much more, but not from the count alone. With one far Capture, the solver's per-Capture errors showed the far Capture as an outlier (43 px against 2–13 px for the near ones), and the result was identical to the near-only set. A second, agreeing far Capture made the far position count. So the gain came mainly from distance coverage, through an interaction between Capture count and coverage.

## Limitations

- **This is projection error, not ground truth.** The RMS measures projection quality on the measurement data, not the true calibration error, and no candidate serves as ground truth.
- **One calibration per condition.** Each condition is a single calibration, so small differences may be noise.
- **Noisy board poses.** The board was hand-held, which adds noise. At far range the LiDAR sees fewer points on the board, so its pose estimate is noisier.
