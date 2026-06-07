/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the calib component (pure math, no devices).
 * Runs on native_sim. Invoke with `west twister -T app/tests`.
 */

#include <math.h>

#include <zephyr/ztest.h>

#include "calib.h"

#define G 9.81f

/* The six standard calibration targets: each IMU axis ±g up. */
static const float targets[6][3] = {
	{ +G, 0.0f, 0.0f }, { -G, 0.0f, 0.0f },
	{ 0.0f, +G, 0.0f }, { 0.0f, -G, 0.0f },
	{ 0.0f, 0.0f, +G }, { 0.0f, 0.0f, -G },
};

/* Synthesize what a miscalibrated sensor reads for a true gravity vector:
 *   raw = M · g_true + bias
 * with M a scale + cross-axis (misalignment) block. The affine fit should
 * recover A ≈ [M⁻¹ | −M⁻¹·bias], i.e. apply(A, raw) ≈ g_true. */
static void synth_raw(const float M[3][3], const float bias[3],
		      const float g_true[3], float raw[3])
{
	for (int i = 0; i < 3; i++) {
		raw[i] = bias[i] + M[i][0] * g_true[0] + M[i][1] * g_true[1] +
			 M[i][2] * g_true[2];
	}
}

/* --- recovery on clean data ------------------------------------------- */

ZTEST(calib, test_identity_data_gives_identity_map)
{
	struct calib_fit f;
	struct calib c;
	float rms = -1.0f;

	calib_fit_reset(&f);
	for (int p = 0; p < 6; p++) {
		calib_fit_add(&f, targets[p], targets[p]); /* raw == target */
	}

	zassert_ok(calib_fit_solve(&f, &c, &rms), "solve should succeed");
	zassert_true(c.valid, "map should be valid");
	zassert_within(rms, 0.0f, 1.0e-3f, "perfect data → ~0 residual");

	const float expect[3][4] = {
		{ 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 },
	};
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			zassert_within(c.A[i][j], expect[i][j], 1.0e-4f,
				       "A[%d][%d] should be identity", i, j);
		}
	}
}

ZTEST(calib, test_recovers_bias_scale_and_misalignment)
{
	/* A sensor with per-axis scale error, cross-axis leakage, and bias. */
	const float M[3][3] = {
		{ 1.03f, 0.02f, -0.01f },
		{ -0.015f, 0.97f, 0.025f },
		{ 0.008f, -0.02f, 1.01f },
	};
	const float bias[3] = { 0.35f, -0.20f, 0.15f }; /* ~0.4 m/s² scale */

	struct calib_fit f;
	struct calib c;
	float rms = -1.0f;

	calib_fit_reset(&f);
	for (int p = 0; p < 6; p++) {
		float raw[3];

		synth_raw(M, bias, targets[p], raw);
		calib_fit_add(&f, raw, targets[p]);
	}

	zassert_ok(calib_fit_solve(&f, &c, &rms), "solve should succeed");
	zassert_within(rms, 0.0f, 1.0e-3f, "consistent data → ~0 residual");

	/* Applying the fitted map to each raw reading must recover g_true. */
	for (int p = 0; p < 6; p++) {
		float raw[3], corrected[3];

		synth_raw(M, bias, targets[p], raw);
		calib_apply(&c, raw, corrected);
		for (int k = 0; k < 3; k++) {
			zassert_within(corrected[k], targets[p][k], 1.0e-3f,
				       "pose %d axis %d not recovered", p, k);
		}
	}
}

/* --- residual reflects measurement noise ------------------------------ */

ZTEST(calib, test_residual_reflects_noise)
{
	const float bias[3] = { 0.1f, 0.0f, -0.05f };
	const float M[3][3] = {
		{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f },
	};
	/* Deterministic ±0.05 m/s² perturbation, zero-mean across the set. */
	const float noise[6][3] = {
		{ +0.05f, -0.03f, +0.02f }, { -0.04f, +0.05f, -0.01f },
		{ +0.02f, +0.01f, -0.05f }, { -0.05f, -0.02f, +0.04f },
		{ +0.03f, +0.04f, +0.01f }, { -0.01f, -0.05f, -0.01f },
	};

	struct calib_fit f;
	struct calib c;
	float rms = -1.0f;

	calib_fit_reset(&f);
	for (int p = 0; p < 6; p++) {
		float raw[3];

		synth_raw(M, bias, targets[p], raw);
		for (int k = 0; k < 3; k++) {
			raw[k] += noise[p][k];
		}
		calib_fit_add(&f, raw, targets[p]);
	}

	zassert_ok(calib_fit_solve(&f, &c, &rms), "solve should succeed");
	/* Residual should be on the order of the injected noise (~0.03 RMS),
	 * not zero and not wildly amplified. */
	zassert_true(rms > 0.005f && rms < 0.08f,
		     "residual %.4f outside expected noise band", (double)rms);
}

/* --- error paths ------------------------------------------------------ */

ZTEST(calib, test_too_few_poses_rejected)
{
	struct calib_fit f;
	struct calib c;

	calib_fit_reset(&f);
	calib_fit_add(&f, targets[0], targets[0]);
	calib_fit_add(&f, targets[1], targets[1]);

	zassert_equal(calib_fit_solve(&f, &c, NULL), -EINVAL,
		      "fewer than CALIB_MIN_POSES should be rejected");
}

ZTEST(calib, test_degenerate_pose_set_rejected)
{
	struct calib_fit f;
	struct calib c;

	/* Six observations all at the same direction: raw spans only one
	 * direction + the constant, so N is rank-deficient. */
	calib_fit_reset(&f);
	for (int p = 0; p < 6; p++) {
		calib_fit_add(&f, targets[0], targets[0]);
	}

	zassert_equal(calib_fit_solve(&f, &c, NULL), -EIO,
		      "rank-deficient pose set should be rejected");
}

/* --- apply helpers ---------------------------------------------------- */

ZTEST(calib, test_apply_is_in_place_safe)
{
	struct calib c;

	calib_identity(&c);
	c.A[0][1] = 0.5f; /* introduce cross-coupling so order matters */

	float a[3] = { 1.0f, 2.0f, 3.0f };
	float ref[3];

	calib_apply(&c, a, ref);   /* into separate buffer */
	calib_apply(&c, a, a);     /* in place */

	for (int k = 0; k < 3; k++) {
		zassert_within(a[k], ref[k], 1.0e-6f,
			       "in-place apply differs at axis %d", k);
	}
}

ZTEST(calib, test_invalid_map_is_passthrough)
{
	struct calib c = { .valid = false };
	float a_in[3] = { 1.5f, -2.5f, 9.0f };
	float a_out[3];

	calib_apply(&c, a_in, a_out);
	for (int k = 0; k < 3; k++) {
		zassert_equal(a_out[k], a_in[k], "invalid map must pass through");
	}
}

ZTEST_SUITE(calib, NULL, NULL, NULL, NULL, NULL);
