# Empirical uncertainty calibration

Aegis can replace its radial Gaussian approximation with a deterministic empirical residual model. The input is a compact CSV exported from a tracker replay:

```text
track_id,horizon_s,predicted_x_m,predicted_y_m,observed_x_m,observed_y_m
```

Each row compares a prediction made at the start of a replay window with the observation at `horizon_s`. Keep train/evaluation runs separate when reporting generalization; the built-in fit report measures in-sample coverage and does not claim out-of-sample validity.

Fit and serialize an artifact:

```bash
python -m aegis_rt calibrate recorded_residuals.csv \
  --output calibration/robot-v1.json \
  --coverage 0.95 \
  --source "warehouse tracker replay 2026-10-01"
```

The artifact stores sorted radial residuals per horizon, the selected quantile, sample count, empirical coverage, and absolute calibration error. Sorting and nearest-bin selection make replay deterministic. Add-one smoothing prevents a finite dataset from producing an unsupported zero-risk estimate.

Compare it with the baseline model:

```bash
python -m aegis_rt compare-predictors \
  scenarios/warehouse_crossing.json calibration/robot-v1.json
```

To use the artifact in a scenario, add a path relative to the scenario file:

```json
{
  "uncertainty_calibration": "../calibration/robot-v1.json"
}
```

`datasets/synthetic_residuals.csv` exists only to exercise the workflow. It is explicitly synthetic and must not be presented as hardware or field evidence. Production artifacts need dataset provenance, held-out coverage checks, drift monitoring, and review under the deployment safety process.
