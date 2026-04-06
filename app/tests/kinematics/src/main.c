/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the kinematics component (pure math, no devices).
 * Runs on native_sim. Invoke with `west twister -T app/tests`.
 */

#include <math.h>

#include <zephyr/ztest.h>

#include "kinematics.h"

#define G 9.81f

/* --- kin_compute_tilt -------------------------------------------------- */

ZTEST(kinematics, test_level_pose_zero_tilt)
{
	float a[3] = { -G, 0.0f, 0.0f };
	float tilt, az;

	kin_compute_tilt(a, &tilt, &az);
	zassert_within(tilt, 0.0f, 0.01f, "expected tilt 0° at level");
}

ZTEST(kinematics, test_sideways_pose_90_deg_tilt)
{
	/* Gravity points along +y_imu (platform tilted right edge down) */
	float a[3] = { 0.0f, -G, 0.0f };
	float tilt, az;

	kin_compute_tilt(a, &tilt, &az);
	zassert_within(tilt, 90.0f, 0.01f, "tilt should be 90°");
	zassert_within(az, 0.0f, 0.01f, "azimuth right should be 0°");
}

ZTEST(kinematics, test_upside_down_180_deg_tilt)
{
	float a[3] = { +G, 0.0f, 0.0f };
	float tilt, az;

	kin_compute_tilt(a, &tilt, &az);
	zassert_within(tilt, 180.0f, 0.01f, "tilt at upside-down should be 180°");
}

ZTEST(kinematics, test_azimuth_quadrants)
{
	/* azimuth = 0 → tilt toward +y_imu (right) */
	float a_right[3] = { -8.5f, -4.905f, 0.0f };
	/* azimuth = ±180 → tilt toward -y_imu (left) */
	float a_left[3] = { -8.5f, +4.905f, 0.0f };
	/* azimuth = +90 → tilt toward -z_imu */
	float a_pz[3] = { -8.5f, 0.0f, +4.905f };
	/* azimuth = -90 → tilt toward +z_imu */
	float a_nz[3] = { -8.5f, 0.0f, -4.905f };
	float tilt, az;

	kin_compute_tilt(a_right, &tilt, &az);
	zassert_within(az, 0.0f, 0.5f);

	kin_compute_tilt(a_left, &tilt, &az);
	zassert_true(fabsf(az) > 179.0f, "left should be ±180°, got %f", (double)az);

	kin_compute_tilt(a_pz, &tilt, &az);
	zassert_within(az, 90.0f, 0.5f);

	kin_compute_tilt(a_nz, &tilt, &az);
	zassert_within(az, -90.0f, 0.5f);
}

/* --- kin_alpha_from_target / kin_alpha_from_accel round-trip ----------- */

static void check_round_trip(float tilt_deg, float az_deg)
{
	float t = tilt_deg * KIN_RAD_PER_DEG;
	float p = az_deg * KIN_RAD_PER_DEG;

	float alpha1, alpha2;

	kin_alpha_from_target(t, p, &alpha1, &alpha2);

	/* Forward kinematics: gravity_imu(α₁, α₂) */
	float g_x = cosf(alpha1) * cosf(alpha2);
	float g_y = sinf(alpha2) * cosf(alpha1);
	float g_z = sinf(alpha1);
	float a[3] = { -g_x * G, -g_y * G, -g_z * G };

	/* Inverse: recover (α₁, α₂) from accel */
	float a1_back, a2_back;

	kin_alpha_from_accel(a, &a1_back, &a2_back);
	zassert_within(a1_back, alpha1, 1.0e-4f,
		       "α₁ round-trip at θ=%.1f φ=%.1f failed: got %f, want %f",
		       (double)tilt_deg, (double)az_deg,
		       (double)a1_back, (double)alpha1);
	zassert_within(a2_back, alpha2, 1.0e-4f,
		       "α₂ round-trip at θ=%.1f φ=%.1f failed: got %f, want %f",
		       (double)tilt_deg, (double)az_deg,
		       (double)a2_back, (double)alpha2);

	/* Tilt round-trip via compute_tilt */
	float t_back, p_back;

	kin_compute_tilt(a, &t_back, &p_back);
	zassert_within(t_back, tilt_deg, 0.01f,
		       "tilt round-trip at θ=%.1f φ=%.1f failed: got %f",
		       (double)tilt_deg, (double)az_deg, (double)t_back);
	if (tilt_deg > 0.5f) {
		/* Azimuth is a circular quantity: ±180° are equivalent. */
		float diff = fabsf(p_back - az_deg);

		if (diff > 180.0f) {
			diff = 360.0f - diff;
		}
		zassert_true(diff < 0.5f,
			     "az round-trip at θ=%.1f φ=%.1f failed: got %f (diff=%f)",
			     (double)tilt_deg, (double)az_deg,
			     (double)p_back, (double)diff);
	}
}

ZTEST(kinematics, test_ik_round_trip_30_45)   { check_round_trip(30.0f,  45.0f); }
ZTEST(kinematics, test_ik_round_trip_30_n135) { check_round_trip(30.0f, -135.0f); }
ZTEST(kinematics, test_ik_round_trip_45_0)    { check_round_trip(45.0f,    0.0f); }
ZTEST(kinematics, test_ik_round_trip_15_n90)  { check_round_trip(15.0f,  -90.0f); }
ZTEST(kinematics, test_ik_round_trip_5_180)   { check_round_trip( 5.0f,  180.0f); }

/* --- kin_target_accel ------------------------------------------------- */

ZTEST(kinematics, test_target_accel_at_level)
{
	float a[3];

	kin_target_accel(0.0f, 0.0f, G, a);
	zassert_within(a[0], -G,    0.01f);
	zassert_within(a[1],  0.0f, 0.01f);
	zassert_within(a[2],  0.0f, 0.01f);
}

ZTEST(kinematics, test_target_accel_tilt_right)
{
	/* θ=30°, φ=0° → tilt right; expect a_y = -sin(30°)·g = -4.905 */
	float a[3];

	kin_target_accel(30.0f * KIN_RAD_PER_DEG, 0.0f, G, a);
	zassert_within(a[0], -cosf(30.0f * KIN_RAD_PER_DEG) * G, 0.01f);
	zassert_within(a[1], -sinf(30.0f * KIN_RAD_PER_DEG) * G, 0.01f);
	zassert_within(a[2],  0.0f,                              0.01f);
}

/* --- kin_scurve_velocity ---------------------------------------------- */

ZTEST(kinematics, test_scurve_starts_at_v_min)
{
	uint32_t v = kin_scurve_velocity(0, 1000, 500, 8000, 200);

	zassert_equal(v, 500U);
}

ZTEST(kinematics, test_scurve_reaches_v_max_in_cruise)
{
	uint32_t v = kin_scurve_velocity(500, 1000, 500, 8000, 200);

	zassert_equal(v, 8000U);
}

ZTEST(kinematics, test_scurve_decelerates_at_end)
{
	uint32_t v = kin_scurve_velocity(999, 1000, 500, 8000, 200);

	zassert_within(v, 500U, 50U);
}

ZTEST(kinematics, test_scurve_triangular_for_short_move)
{
	/* total < 2*n_ramp → triangular profile: ramp clamps to total/2 each
	 * way and the velocity hits v_max for a single sample at the apex
	 * (idx == n_acc), then immediately decelerates. */
	uint32_t v_start  = kin_scurve_velocity(0,  100, 500, 8000, 200);
	uint32_t v_quart  = kin_scurve_velocity(25, 100, 500, 8000, 200);
	uint32_t v_apex   = kin_scurve_velocity(50, 100, 500, 8000, 200);
	uint32_t v_end    = kin_scurve_velocity(99, 100, 500, 8000, 200);

	zassert_equal(v_start, 500U);
	zassert_true(v_quart > 500U && v_quart < 8000U,
		     "ramp midpoint should be between v_min and v_max, got %u",
		     v_quart);
	zassert_equal(v_apex, 8000U, "triangular apex should hit v_max");
	zassert_within(v_end, 500U, 50U);
}

ZTEST(kinematics, test_scurve_monotonic_in_accel_phase)
{
	uint32_t prev = 0;

	for (int i = 0; i < 200; i++) {
		uint32_t v = kin_scurve_velocity(i, 1000, 500, 8000, 200);

		zassert_true(v >= prev,
			     "non-monotonic at idx=%d: prev=%u v=%u",
			     i, prev, v);
		prev = v;
	}
}

ZTEST_SUITE(kinematics, NULL, NULL, NULL, NULL, NULL);
