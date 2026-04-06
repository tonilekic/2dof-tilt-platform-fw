/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include "kinematics.h"

void kin_compute_tilt(const float a[3], float *tilt_deg, float *azimuth_deg)
{
	float lat = sqrtf(a[1] * a[1] + a[2] * a[2]);

	/* Two-axis tilt: atan2 of lateral magnitude vs vertical. Equivalent to
	 * arccos(-a_x / |a|) but uniformly conditioned across [0, π] — arccos
	 * has a vertical tangent at ±1 that amplifies accel noise into tilt
	 * error near level (θ→0) and inverted (θ→π). */
	*tilt_deg = atan2f(lat, -a[0]) * KIN_DEG_PER_RAD;
	*azimuth_deg = atan2f(a[2], -a[1]) * KIN_DEG_PER_RAD;
}

void kin_alpha_from_accel(const float a[3], float *alpha1_rad,
			  float *alpha2_rad)
{
	float g_x = -a[0], g_y = -a[1], g_z = -a[2];
	float n = sqrtf(g_x * g_x + g_y * g_y + g_z * g_z);

	if (n < 1.0e-3f) {
		*alpha1_rad = 0.0f;
		*alpha2_rad = 0.0f;
		return;
	}

	float gz = g_z / n;

	if (gz > 1.0f) {
		gz = 1.0f;
	} else if (gz < -1.0f) {
		gz = -1.0f;
	}

	*alpha1_rad = asinf(gz);
	*alpha2_rad = atan2f(g_y, g_x);
}

void kin_alpha_from_target(float tilt_rad, float az_rad,
			   float *alpha1_rad, float *alpha2_rad)
{
	float arg = -sinf(tilt_rad) * sinf(az_rad);

	if (arg > 1.0f) {
		arg = 1.0f;
	} else if (arg < -1.0f) {
		arg = -1.0f;
	}

	*alpha1_rad = asinf(arg);
	*alpha2_rad = atan2f(sinf(tilt_rad) * cosf(az_rad), cosf(tilt_rad));
}

void kin_target_accel(float tilt_rad, float az_rad, float g,
		      float a_target[3])
{
	a_target[0] = -cosf(tilt_rad) * g;
	a_target[1] = -sinf(tilt_rad) * cosf(az_rad) * g;
	a_target[2] =  sinf(tilt_rad) * sinf(az_rad) * g;
}

uint32_t kin_scurve_velocity(int32_t idx, int32_t total,
			     uint32_t v_min, uint32_t v_max,
			     int32_t n_ramp)
{
	int32_t n_acc = (n_ramp < total / 2) ? n_ramp : total / 2;
	int32_t n_dec = n_acc;
	int32_t n_cruise = total - n_acc - n_dec;
	float frac;

	if (n_acc <= 0) {
		return v_min;
	}

	if (idx < n_acc) {
		frac = (float)idx / (float)n_acc;
	} else if (idx < n_acc + n_cruise) {
		return v_max;
	} else {
		frac = 1.0f - (float)(idx - n_acc - n_cruise) / (float)n_dec;
	}

	float s = frac * frac * (3.0f - 2.0f * frac);

	return (uint32_t)((float)v_min + (float)(v_max - v_min) * s);
}
