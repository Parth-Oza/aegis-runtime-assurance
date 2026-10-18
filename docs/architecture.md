# Architecture and safety argument

## System boundary

Aegis accepts a robot state, an upstream control command, tracked dynamic obstacles, and a safety configuration. It returns an applied command plus decision evidence. The upstream planner is intentionally treated as a black box.

## Decision path

1. Roll out the commanded acceleration and yaw rate through a bounded unicycle model.
2. Propagate each obstacle with constant velocity while growing positional uncertainty quadratically from acceleration uncertainty.
3. Convert separation, geometry, safety margin, and combined uncertainty into a conservative radial overlap probability.
4. Allow the command only when both maximum collision probability and geometric clearance pass the configured envelope.
5. Otherwise evaluate a deterministic lattice of braking and steering alternatives.
6. Select the safe alternative with minimum normalized command deviation, progress loss, and tail-risk cost.
7. Emit the original and applied risk traces as auditable evidence.

## Control-barrier fallback

The optional `cbf` controller uses the squared-distance barrier

`h(x, o) = ||p_robot - p_obstacle||² - (r_robot + r_obstacle + margin)²`.

For every prediction step and tracked obstacle it enforces the discrete exponential condition

`h[k+1] >= (1 - alpha * dt) * h[k]`, with `alpha = 4.0 s^-1`.

The sampled optimizer is deterministic: it evaluates 11 bounded acceleration values and 17 bounded yaw rates, rejects candidates that violate the barrier or the probabilistic envelope, then minimizes the same deviation/progress/tail-risk objective as the lattice controller. Its assumptions are planar circular geometry, constant-velocity obstacle means, a constant candidate command over the horizon, and sufficiently small `dt` for the discrete condition. This is a research fallback candidate, not a continuous-time QP implementation or a certification claim.

Every intervention reports `fallback_controller`, `fallback_status`, and normalized `command_deviation`. When no sampled control meets both constraints, status is `infeasible` and the supervisor returns a clearly labeled minimum-risk emergency command rather than misrepresenting it as safe.

## Current assumptions

- Planar motion and circular footprints
- Constant command over the finite horizon
- Constant-velocity obstacle means
- Isotropic Gaussian position uncertainty
- A configured delay before the override takes effect

These assumptions are visible rather than hidden. A production integration should replace them with calibrated vehicle and tracker models while keeping the supervisor interface stable.

When a scenario references an empirical calibration artifact, collision probability uses the survival function of recorded radial residuals from the nearest prediction-horizon bin. The baseline Gaussian model remains the default so existing scenarios and integrations are unchanged. Evidence labels every risk sample with the active uncertainty model.

## Failure behavior

If no candidate satisfies the risk budget, Aegis chooses the minimum-risk candidate and labels the decision as an emergency fallback. This distinction is important: the system never describes an unsafe candidate as safe simply because it was the best available option.

## Extension interfaces

The next architectural split should introduce protocols for `DynamicsModel`, `ObstaclePredictor`, `RiskMetric`, and `FallbackController`. That permits ROS 2, learned prediction, reachability libraries, and certified braking controllers without changing evidence generation.

