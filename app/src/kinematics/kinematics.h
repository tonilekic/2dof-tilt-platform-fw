/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure-math kinematics for the 2-DOF gimbal: tilt/azimuth from accel,
 * forward & inverse kinematics, S-curve velocity profile.
 *
 * No hardware dependencies — all functions are stateless and host-testable.
 * See app/README.md "Coordinate systems" and "2-DOF gimbal kinematics" for
 * derivations.
 */

#ifndef APP_KINEMATICS_H_
#define APP_KINEMATICS_H_

#include <stdint.h>

#define KIN_PI               3.14159265358979323846f
#define KIN_DEG_PER_RAD      (180.0f / KIN_PI)
#define KIN_RAD_PER_DEG      (KIN_PI / 180.0f)

/**
 * @brief Convert a measured accel vector to spherical (tilt, azimuth).
 *
 * Mounting: -x_imu points up at level → at-rest accel = (-g, 0, 0).
 *   tilt    θ = arctan2(sqrt(a_y² + a_z²), -a_x)   ∈ [0°, 180°]
 *   azimuth φ = arctan2(a_z, -a_y)                 ∈ (-180°, 180°]
 * Two-axis tilt form: numerically uniform across [0, π], unlike the
 * algebraically equivalent arccos(-a_x/|a|) which amplifies noise near
 * θ=0 and θ=π. φ is undefined at θ=0; arctan2 returns 0 there.
 */
void kin_compute_tilt(const float a[3], float *tilt_deg, float *azimuth_deg);

/**
 * @brief Inverse kinematics: motor angles deviation from a measured accel.
 *
 * 2-DOF gimbal model:
 *   gravity_imu(α₁, α₂) = (cos α₁ cos α₂, sin α₂ cos α₁, sin α₁) · g
 * α₁ = motor1 (outer), α₂ = motor2 (inner) deviation from level pose, in rad.
 *
 * At α₁ = ±90° the model is gimbal-locked in α₂ (atan2(0,0) → 0); a single
 * IK move from there won't reach level, but a second iteration resolves α₂.
 */
void kin_alpha_from_accel(const float a[3], float *alpha1_rad, float *alpha2_rad);

/**
 * @brief Forward IK: target motor angles for a desired (tilt, azimuth).
 *
 *   α₁_target = arcsin(-sin θ sin φ)
 *   α₂_target = arctan2(sin θ cos φ, cos θ)
 */
void kin_alpha_from_target(float tilt_rad, float az_rad,
			   float *alpha1_rad, float *alpha2_rad);

/**
 * @brief Target proper-acceleration vector (m/s²) for a desired (tilt, az).
 *
 *   a_target = -g · (cos θ, sin θ cos φ, -sin θ sin φ)
 */
void kin_target_accel(float tilt_rad, float az_rad, float g,
		      float a_target[3]);

/**
 * @brief Smoothstep S-curve velocity (steps/s) at step `idx` of a `total`-
 * step move.
 *
 * Acceleration and deceleration each span min(n_ramp, total/2) steps and
 * are mirror images. If `total` is too short for a cruise phase the profile
 * is triangular. Velocity follows smoothstep s(x) = 3x² - 2x³ giving
 * continuous-jerk envelopes (no kinks at v_min/v_max boundaries).
 */
uint32_t kin_scurve_velocity(int32_t idx, int32_t total,
			     uint32_t v_min, uint32_t v_max,
			     int32_t n_ramp);

#endif /* APP_KINEMATICS_H_ */
