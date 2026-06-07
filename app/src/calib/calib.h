/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure-math accelerometer calibration: 12-parameter affine fit by least
 * squares over static poses of known gravity direction.
 *
 * No hardware dependencies — the fit, the apply step, and the solver are all
 * stateless and host-testable. See app/README.md "IMU calibration".
 */

#ifndef APP_CALIB_H_
#define APP_CALIB_H_

#include <stdbool.h>

/*
 * Model:
 *   a_corrected = A · [a_raw_x, a_raw_y, a_raw_z, 1]ᵀ        (A is 3×4)
 *
 * The 3×4 affine map folds bias (column 3), scale + axis non-orthogonality
 * (the 3×3 block), and the IMU-to-platform mounting rotation into one map.
 * A magnitude-only ("ellipsoid") fit cannot recover the mounting rotation —
 * a rotation preserves |a| — which is why each observation carries a *known*
 * gravity direction (established here by an external laser reference).
 *
 * The fit decouples by output row: each of the three corrected components is
 * an independent 4-parameter linear least-squares problem that shares the
 * same normal-equations matrix N = Σ xₚ xₚᵀ. So the whole solve is one 4×4
 * elimination with three right-hand sides — trivial on the M4F.
 */

/* Minimum observations for a solvable (rank-4) fit. The standard procedure
 * uses six (±x, ±y, ±z up); six is also what makes the fit well-conditioned. */
#define CALIB_MIN_POSES 4

/** Least-squares accumulator. Holds only the normal-equation moments, so its
 *  size is independent of how many observations are added. */
struct calib_fit {
	double N[4][4]; /* Σ xₚ xₚᵀ                              */
	double b[3][4]; /* Σ xₚ g_k,ₚ  — one RHS column per axis  */
	double gg[3];   /* Σ g_k,ₚ²    — for the residual         */
	unsigned int n; /* observation count                     */
};

/** Fitted calibration applied in the IMU read path. */
struct calib {
	float A[3][4];
	bool valid; /* false ⇒ treat as uncalibrated (pass-through) */
};

/** Zero the accumulator to start a fresh calibration session. */
void calib_fit_reset(struct calib_fit *f);

/** Add one static observation: the raw averaged accel `raw` (m/s²) measured
 *  at a pose whose true gravity reading is `target` (e.g. (+g,0,0) for the
 *  IMU +x axis pointing up). */
void calib_fit_add(struct calib_fit *f, const float raw[3],
		   const float target[3]);

/**
 * @brief Solve the accumulated normal equations for the affine map.
 *
 * @param f          accumulator with ≥ CALIB_MIN_POSES observations.
 * @param out        receives the fitted map (valid=true) on success.
 * @param resid_rms  if non-NULL, receives the RMS accel residual (m/s²) of
 *                   the fit over all added observations — the headline
 *                   "how good is this calibration" number.
 * @return 0 on success, -EINVAL if too few observations, -EIO if the pose
 *         set is degenerate (singular normal matrix).
 */
int calib_fit_solve(const struct calib_fit *f, struct calib *out,
		    float *resid_rms);

/** Set `out` to the identity map (pass-through), valid=true. */
void calib_identity(struct calib *out);

/** Apply the calibration: a_out = A · [a_in, 1]. In-place safe
 *  (a_out may alias a_in). A pass-through copy if !c->valid. */
void calib_apply(const struct calib *c, const float a_in[3], float a_out[3]);

#endif /* APP_CALIB_H_ */
