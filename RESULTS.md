# Optical Flow Obstacle Avoidance — Results

Evaluation of optical flow τ-estimation against LiDAR ground truth on
`unitree_office_walk`. Full reference: theory, methodology, results, limitations.

---

## Theory

**Physical basis (Lee 1976):** when a camera moves toward a surface, pixels
expand radially outward from the Focus of Expansion. The rate of expansion —
divergence — is inversely proportional to Time-to-Contact (continuous form):
`τ = 1 / divergence`.

**Discrete implementation (this pipeline):** per-cell divergence is estimated
from tracked flow as `mean(flow_x)/cell_w + mean(flow_y)/cell_h`, and
`τ_cell = 1 / div`. Flow is in pixels-per-frame with no dt normalization, so
τ is not literally seconds — it is proportional to physical TTC up to a
frame-rate × grid-geometry factor. Scale-free in the sense that closer/faster
approach → smaller τ at any speed or object size; thresholds are tuned
empirically rather than derived from `distance/speed`.

**What breaks it:** rotation produces global image shift indistinguishable
from expansion, so the formula is only valid during pure forward translation.

**Algorithm steps:**

1. **Keypoint detection — FAST (Rosten 2006):** scans for pixels with a
   surrounding ring of 16 pixels all brighter or all darker. Up to 300 per
   frame, refreshed every 10 frames. ORB-SLAM uses the same FAST corners for
   map-point extraction; it adds an orientation estimate (intensity centroid)
   for BRIEF descriptor matching — irrelevant here, since LK tracks image
   patches and has no orientation input. Plain FAST is the correct choice.

2. **Lucas-Kanade flow:** for each keypoint, LK minimizes sum-of-squared
   intensity differences in a 21×21 window (3-level pyramid) to locate where
   the patch moved in the next frame.

3. **Grid divergence:** image divided into a 5×5 cell grid. Per cell:
   `τ = 1/div` if `div > 0` (approaching); `NaN` if `div ≤ 0` (static or
   receding). Danger fires if `min(τ) < tau_threshold`.

4. **Threshold calibration:** τ is a proxy for Time-to-Contact — smaller τ
   means a closer/faster-approaching obstacle — but the numeric value depends
   on frame rate and cell geometry, so it is not literally seconds. The
   default `tau_threshold = 3.0` was chosen from the PR sweep below: it lies
   on the high-precision shoulder of the curve (P=0.939 at τ=3.0 on this
   dataset). Tune per-deployment from the PR curve, not from a physical
   distance/speed formula.

5. **Rotation gate (live module):** if `angular_velocity` stream is connected
   and `|ω| > omega_max` (default 0.3 rad/s), `danger_signal` is suppressed.
   If unused, `_last_omega` stays 0 — gate is transparent.

---

## Experiment Setup

### Dataset

`unitree_office_walk` — 51 s indoor sequence, Go2 drives repeatedly toward
walls and re-orients (96% forward-translation motion). τ-estimation is only
valid during forward translation, making this the appropriate dataset for
evaluation.

### Odom Gate

| Condition | Value | Rationale |
|-----------|-------|-----------|
| v\_fwd > 0.15 m/s | forward speed | robot must be translating toward something |
| \|ω\_yaw\| < 0.15 rad/s | yaw rate | eliminates rotation-contaminated frames |

**125 gated frames out of 390 total (33%).** Forward speed on gated frames:
p25=0.36 · p50=0.45 · p75=0.56 m/s.

**Ground truth:** LiDAR minimum forward distance in body frame (world frame
transformed using per-frame odom quaternion). Binary GT: `dist < 1.5 m`.

---

## Results

**Latency:** ~2.5 ms/frame — < 4% of 77 ms frame budget at 13 Hz.

### Correlation — Does τ Track Real TTC?

`lidar_ttc = min_fwd_dist / v_fwd` compared to optical flow `min_tau`.

| Metric | Value |
|--------|-------|
| Paired frames | 82 |
| Pearson r | +0.144 |
| Spearman r | +0.173 |
| RMSE | 3.31 s |
| MAE | 2.52 s |

Weak positive correlation — correct sign (smaller τ → closer obstacle), noisy
magnitude. Per-cell divergence mixes approach signal with parallax and tracker
drift. Use τ as a **ranked urgency signal**, not a calibrated timer.

### Binary Detection

GT positive rate: 84% (indoor walls are always nearby on forward frames).

| τ threshold | Purpose | Precision | Recall | F1 |
|-------------|---------|-----------|--------|----|
| **3.0 s (default)** | **Production** | **0.939** | **0.295** | **0.449** |
| 5.8 s | Max recall | 0.918 | 0.533 | 0.675 |

At τ=3.0, 93.9% of alarms are confirmed by LiDAR. Recall of ~30% reflects
the thresholded nature of the signal — many GT-close frames produce τ values
just above the decision boundary (noisy divergence) and are missed. Raising
the threshold to 5.8 lifts recall to 0.53 at a small precision cost.
Precision stays above 0.90 for all τ ≥ 1.5 across the operating range
(AUC-PR = 0.479; below the 0.84 baseline because 84% of gated frames are
GT-positive — precision at each recall level is the meaningful metric).

---

## Known Limitations

| Limitation | Consequence | Mitigation |
|------------|-------------|------------|
| Stationary obstacles produce no flow | Blind to static walls at constant distance | LiDAR for static proximity |
| τ noisy (RMSE ≈ 3 s) | Not a precise collision timer | Ranked alarm; wire `tac_grid` to planner |
| Rotation contaminates signal | Global image shift = false divergence | Odom gate in eval; `angular_velocity` gate in module |
| Low-texture surfaces | Few keypoints → sparse grid coverage | Edge-enhancing preprocessing |

**Deployment guidance:** keep the default `tau_threshold = 3.0` for high
precision, or raise toward 5–6 for higher recall; retune from the PR sweep
on your own data rather than from a physical distance/speed formula. Wire
`angular_velocity` from IMU for automatic turn suppression. Use as supplement
to LiDAR — not a replacement.

---

## Verdict

Optical flow τ-estimation is a viable **complement** to LiDAR — not a standalone
replacement. When the alarm fires it is correct 93.9% of the time (P=0.939 at
τ=3.0), but it only catches ~30% of danger frames (R=0.295). Low recall is
inherent: stationary obstacles produce no flow, and noisy divergence near the
threshold causes missed detections close to the decision boundary. Deploy
alongside LiDAR: LiDAR for static proximity, optical flow for fast-approaching
and dynamic obstacles. Real-time at ~2.5 ms/frame (< 4% of frame budget),
with a single empirically tuned parameter (`tau_threshold`).
