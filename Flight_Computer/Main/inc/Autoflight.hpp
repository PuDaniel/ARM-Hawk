/*
 * Autoflight.hpp
 *
 *  Created on: Sep 30, 2018
 *      Author: Daniel
 */

#ifndef AUTOFLIGHT_HPP_
#define AUTOFLIGHT_HPP_

// Built-in modules
#include "stm32f7xx.h"
#include <cstdint>

// Own modules
#include "System.hpp"
#include "ArmHawk_math.hpp"

// Elevator trim values as seen on T16SZ transmitter screen (trim step = 7) for zero and full flaps
#define SAS_ELEVATOR_ZERO_TRIM			-3
#define SAS_ELEVATOR_FLAP_TRIM			-28

// The maximum control surface deflections, the Stability Augmentation System can command
#define SAS_AILERON_TRAVEL_LIMIT		1.0f
#define SAS_ELEVATOR_TRAVEL_LIMIT		0.70f
#define SAS_RUDDER_TRAVEL_LIMIT			0.50f

// Limitation parameters for SAS
#define SAS_STALLPROT_DELTA_T			(1.0f / 20.0f)
#define SAS_ALPHA_PROT					(12.0f * PI / 180.0f)
#define SAS_STALLPROT_ALPHA_TARGET		(6.0f * PI / 180.0f)
#define SAS_STALLPROT_GAMMA_TARGET		(30.0f * PI / 180.0f)
#define SAS_STALLPROT_MIN_THROTTLE		0.5f
#define SAS_STALLPROT_BANK_LIMIT		(45.0f * PI / 180.0f)

// SAS stall protection operational modes
#define SAS_STALL_MODE_STBY				0
#define SAS_STALL_MODE_ALPHA_HOLD		1
#define SAS_STALL_MODE_GAMMA_HOLD		2

/*
 * Provides feedback control to all aileron, elevator, throttle and rudder
 * for improving the airplane's dynamics
 */
class StabilityAugmentationSystem
{
	// Define linearized system models for self test
	Filter<4, 4, float> G_phi_deltaA;
	Filter<4, 4, float> G_beta_deltaR;
	Filter<3, 3, float> G_P_deltaA;
	Filter<4, 4, float> G_R_deltaR;

	// Roll damper proportional gain
	float k_P;

	// Bank controller proportional gain
	float k_phi;

	// Maximum allowed roll rate at current airspeed [rad/s]
	float P_max;

	// Yaw damper proportional gain
	float k_R;

	// Yaw damper washout filter time constant
	float T_R;

	// Yaw damper washout filter
	Filter<2, 2, float> H_R;

	// Turn coordinator proportional gain
	float k_beta;

	// Turn coordinator integrator time constant
	float T_beta;

	// Turn coordinator integrator (velocity form)
	Filter<2, 1, float> H_beta;
	float delta_R_beta;

	// Maximum allowed sideslip angle [rad]
	float beta_max;

	// Static elevator trim to account for CG shifts and flaps settings
	float elevator_trim;

	// Stall protection state space system and reference states (+derivations)
	static const Matrix<3, 3> p_A[10];
	static const Vector<2> p_a[10];
	static const float p_B[10];
	static const Vector<3> p_dx[10];

	// Freezes throttle at 'SAS_STALLPROT_THROTTLE' after stall prot; disabled by commanding higher throttle level
	char throttle_freeze;

	Matrix<3, 3> A;
	Vector<2> a;
	float B;
	Vector<3> x_0;
	Vector<3> dx;

public:
	// Disables the self test and enables the throttle controller once airborne
	char ground_lock;

	// Bypass bank controller in special situations (1 = active)
	char bank_bypass;

	// Commanded bank angle for bank controller [rad]
	float phi_cmd;

	// Commanded sideslip for turn coordinator [rad]
	float beta_cmd;

	// Flight path angle (computed from phi, theta, alpha) [rad]
	float gamma;

	// Stall protection operational mode
	char stall_mode;

	void update(float alpha, float V_T, Vector<3> Phi, float n_z, float omega_P, float delta_F, char autopilot_mode);

	float stabilize_roll(float phi, float P, float delta_A_cmd);
	float stabilize_pitch(float theta, float Q, float alpha, float V_T, float delta_E_cmd);
	float stabilize_speed(float delta_T_cmd);
	float stabilize_yaw(float beta, float R, float delta_R_cmd);

	float self_test_roll(float delta_A_cmd);
	float self_test_yaw(float delta_R_cmd);

	StabilityAugmentationSystem();
};


#endif /* AUTOFLIGHT_HPP_ */
