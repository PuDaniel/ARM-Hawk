/*
 * Autoflight.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: Daniel
 */

#include "Autoflight.hpp"

// SAS constant members for computing interpolated linear systems (stall protetction)
const Matrix<3, 3> StabilityAugmentationSystem::p_A[10] =
{
	Matrix<3, 3>({-1.09188e-02, 1.01357e+00, -3.30995e-04, -1.15491e-02, -2.07507e-01, -1.56805e-03, 6.70906e-02, 2.70226e+00, 9.22640e-02}),
	Matrix<3, 3>({2.67871e-04, 1.63924e+01, -2.90651e-04, -2.59557e-02, -1.74242e-01, -3.67459e-03, 5.77674e-02, -4.02373e+01, -6.44761e-01}),
	Matrix<3, 3>({2.28586e-06, 2.93235e-05, -1.48847e-08, -1.28894e-07, -2.49366e-06, -3.30025e-08, -4.88796e-06, -1.05521e-04, -7.86279e-06}),
	Matrix<3, 3>({-8.68001e-05, 9.33953e-01, 1.70135e-05, -5.01559e-03, 5.82092e-03, -5.09263e-04, 1.78385e-02, 1.97568e+00, -4.87891e-02}),
	Matrix<3, 3>({-2.41515e-04, 4.07470e-03, 1.55745e-06, 1.50427e-05, 1.07690e-03, 1.04167e-05, 4.03943e-04, 2.72878e-03, 1.06207e-03}),
	Matrix<3, 3>({4.01532e-07, 6.25728e-05, 3.21849e-07, 8.83089e-08, -4.62449e-07, 2.92852e-07, 1.66376e-05, 5.86777e-04, 4.82652e-05}),
	Matrix<3, 3>({6.48708e-02, -1.48970e+01, 3.41531e-03, 5.51522e-02, 1.45164e+00, 5.45720e-03, -6.77052e-01, -1.39745e+01, -1.35364e+00}),
	Matrix<3, 3>({-3.09873e-05, -4.93062e+00, -1.54273e-04, 1.60027e-02, 1.48757e-02, 1.65574e-03, -6.04553e-02, 1.61599e+00, 1.96935e-01}),
	Matrix<3, 3>({-2.59307e-03, -1.92930e-02, 9.57609e-06, 1.12404e-04, -7.76055e-04, -6.44436e-06, 4.32162e-03, 5.29851e-02, 2.04468e-03}),
	Matrix<3, 3>({9.34336e-02, 1.11363e+01, -4.02569e-03, -1.26476e-01, -1.10589e-01, 9.73271e-01, -9.74468e-01, -1.77389e+01, -1.98304e+00})
};

const Vector<2> StabilityAugmentationSystem::p_a[10] =
{
	Vector<2>({9.22571e-02, -3.13062e-03}),
	Vector<2>({2.62018e-01, -8.63690e-03}),
	Vector<2>({-3.53314e-20, -2.37801e-21}),
	Vector<2>({-9.23924e-03, 1.52387e-01}),
	Vector<2>({-1.02484e-17, 6.21159e-19}),
	Vector<2>({-2.99065e-20, -4.96554e-20}),
	Vector<2>({-6.72585e-01, -4.36335e-03}),
	Vector<2>({-5.85732e-02, -5.33280e-01}),
	Vector<2>({5.66972e-17, 1.24957e-18}),
	Vector<2>({1.49863e+00, 9.33028e-02})
};

const float StabilityAugmentationSystem::p_B[10] =
{
	-1.91436e-01, -1.61356e+00, -1.60514e-04, 5.81003e-02, 2.04716e-02, 1.33385e-04, -2.75457e+01, 5.91662e-02, 6.66667e-02, -6.19512e+00
};

const Vector<3> StabilityAugmentationSystem::p_dx[10] =
{
	Vector<3>({3.39153e-01, -5.55424e-02, 7.45110e-01}),
	Vector<3>({-1.91918e-01, -6.83856e-01, 2.87540e+00}),
	Vector<3>({3.98514e-05, -1.11989e-06, -3.04412e-05}),
	Vector<3>({-1.28110e-02, -9.74509e-02, 3.84956e-01}),
	Vector<3>({-5.39993e-03, 2.15474e-04, 3.53171e-04}),
	Vector<3>({-4.73109e-07, -3.06472e-06, -5.54827e-04}),
	Vector<3>({-3.56669e+00, 1.87310e-01, -1.06186e+01}),
	Vector<3>({-8.87041e+00, 3.83573e-01, -1.10663e+00}),
	Vector<3>({-1.61267e-02, 2.43762e-04, 2.11693e-02}),
	Vector<3>({4.44844e+00, -1.30645e-01, 2.58801e-01})
};

/*
 * Initialize all controller gains with minimum airspeed values
 */
StabilityAugmentationSystem::StabilityAugmentationSystem()
	: G_phi_deltaA({1.0, -2.4719, 1.9969, -5.2493e-1}, {0, -1.3146e-3, -2.6292e-3, -1.3146e-3}),
	  G_beta_deltaR({1.0, -2.5589, 2.1458, -5.8323e-1}, {0, 1.3736e-3, 1.1265e-3, -2.4707e-4}),
	  G_P_deltaA({1.0, -1.4724, 5.2515e-1}, {0, -1.3119e-1, -1.3119e-1}),
	  G_R_deltaR({1.0, -2.5589, 2.1458, -5.8323e-1}, {0, -5.4292e-2, -6.7240e-5, 5.4225e-2}),
	  H_R({1.0f, -0.9855072f}, {100.0f, -100.0f}),
	  H_beta({1.0f}, {1.105f, -0.895f}),
	  A(1), a(1), B(1), dx(0)
{
	// Enable self-test and disable throttle controller until airborn for first time
	this->ground_lock = 1;

	// Roll damper
	this->k_P = -0.34f;

	// Bank controller
	this->k_phi = -2.465f;

	this->P_max = 2.3f;

	this->bank_bypass = 0;
	this->phi_cmd = 0;

	// Yaw damper
	this->k_R = -0.314f;
	this->T_R = 1.37f;

	// Turn coordinator
	this->k_beta = 0.2335f;
	this->T_beta = 10.5f;
	this->delta_R_beta = 0;

	this->beta_max = 8.0f * PI / 180.0f;
	this->beta_cmd = 0;

	// Elevator trim
	this->elevator_trim = 0;

	// Stall protection
	this->gamma = 0;
	this->stall_mode = 0;
	this->throttle_freeze = 0;
}

/*
 * Update the controller coefficients (gain scheduling) according to airspeed;
 * Idle controllers when system in manual modes
 * !!! Must be called in every mainloop iteration !!!
 * @param V_T				Airspeed from ADC [m/s]
 * @param Phi				Attitude vector (euler angles) [rad]
 * @param autopilot_mode	Current operational mode
 */
void StabilityAugmentationSystem::update(float alpha, float V_T, Vector<3> Phi, float n_z, float omega_P, float delta_F, char autopilot_mode)
{
	// Disable self test and release throttle controller once airborne
	if(V_T > 10.0f)
		this->ground_lock = 0;

	// Reset self-tests when not needed
	if(autopilot_mode != AUTOPILOT_MODE_STABILITY)
	{
		this->G_phi_deltaA.reset();
		this->G_beta_deltaR.reset();
		this->G_P_deltaA.reset();
		this->G_R_deltaR.reset();
	}

	// Limit airspeed to avoid unreasonable gains
	if(V_T < 15.0f)
		V_T = 15.0f;
	else if(V_T > 40.0f)
		V_T = 40.0f;

	// Roll damper
	this->k_P = -0.58f + 0.016f * V_T;
	if(this->k_P > 0.0f)
		this->k_P = 0.0f;

	// Bank controller
	this->k_phi = -3.23f + 0.051f * V_T;
	if(this->k_phi > 0.0f)
		this->k_phi = 0.0f;

	this->P_max = 0.425f + 0.125f * V_T;
	if(this->P_max < 0.0f)
		this->P_max = 0.0f;

	this->bank_bypass = (autopilot_mode != AUTOPILOT_MODE_STABILITY) || (abs(Phi(1)) > (PI / 3.0f));

	if(this->bank_bypass)
		this->phi_cmd = Phi(0);

	// Yaw damper
	this->k_R =  -0.44f + 0.0084f * V_T;
	if(this->k_R > 0.0f)
		this->k_R = 0.0f;

	this->T_R = 0.56f + 0.054f * V_T;
	this->H_R.a[1] = (1.0f / this->T_R - 100.0f) / (1.0f / this->T_R + 100.0f);

	if(autopilot_mode != AUTOPILOT_MODE_STABILITY)
		this->H_R.reset();

	// Turn coordinator
	this->k_beta = 0.37f - 0.0091f * V_T;
	if(this->k_beta < 0.0f)
		this->k_beta = 0.0f;

	this->T_beta = -12.0f + 1.5f * V_T;
	this->H_beta.b[0] = this->T_beta / 100.0f + 1.0f;
	this->H_beta.b[1] = this->T_beta / 100.0f - 1.0f;

	if(autopilot_mode != AUTOPILOT_MODE_STABILITY)
	{
		this->H_beta.reset();
		this->delta_R_beta = 0;
		this->beta_cmd = 0;
	}

	// Compute static elevator trim
	this->elevator_trim = -0.01f * (SAS_ELEVATOR_ZERO_TRIM + (SAS_ELEVATOR_FLAP_TRIM - SAS_ELEVATOR_ZERO_TRIM) * delta_F);

	// Compute flight path angle
	this->gamma = asin(sin(Phi(1)) * cos(alpha) - cos(Phi(0)) * cos(Phi(1)) * sin(alpha));

	// Stall protection gains
	if(n_z < 1.0f)
		n_z = 1.0f;

	// TODO vvv This got added vvv
	if(this->gamma > (PI / 3.0f))
		this->gamma = PI / 3.0f;
	else if(this->gamma < (-PI / 3.0f))
		this->gamma = -PI / 3.0f;

	// TODO vvv This got changed vvv
	this->x_0(0) = 17.97f * sqrt(n_z);
	this->x_0(1) = SAS_ALPHA_PROT;
	this->x_0(2) = 10.03f * cos(Phi(1)) / this->x_0(0) * (n_z * cos(this->gamma) / 0.9781f - 0.9781f / (n_z * cos(this->gamma)));

	float y[9] = {n_z * n_z, Phi(1) * Phi(1), omega_P * omega_P, n_z * Phi(1), n_z * omega_P, Phi(1) * omega_P, n_z, Phi(1), omega_P};

	this->A = this->p_A[9];
	this->a = this->p_a[9];
	this->B = this->p_B[9];
	this->dx = this->p_dx[9];

	for(int i = 0; i < 9; i++)
	{
		this->A += this->p_A[i] * y[i];
		this->a += this->p_a[i] * y[i];
		this->B += this->p_B[i] * y[i];
		this->dx += this->p_dx[i] * y[i];
	}

	// Change stall protection mode
	if((autopilot_mode != AUTOPILOT_MODE_STABILITY) || (abs(Phi(1)) > (PI / 3.0f)) || this->ground_lock)
	{
		// Disengage protections for pitch outside +/-60deg
		this->stall_mode = SAS_STALL_MODE_STBY;
	}
	else if(this->stall_mode == SAS_STALL_MODE_ALPHA_HOLD)
	{
		// Try to climb with constant alpha at alpha_prot
		if(this->gamma >= SAS_STALLPROT_GAMMA_TARGET)
			this->stall_mode = SAS_STALL_MODE_GAMMA_HOLD;
	}
	else if(this->stall_mode == SAS_STALL_MODE_GAMMA_HOLD)
	{
		// Reduce elevator and wait until alpha reduces below target
		if(alpha <= SAS_STALLPROT_ALPHA_TARGET)
		{
			this->throttle_freeze = 1;
			this->stall_mode = SAS_STALL_MODE_STBY;
			print("Stall prot. disengaged at climb gradient [deg]: ", this->gamma * 180.0f / PI, MESSAGE_NOTE);
		}
	}
	else
	{
		// Protection in standby, check alpha below alpha_prot
		if(alpha >= SAS_ALPHA_PROT)
		{
			this->stall_mode = SAS_STALL_MODE_ALPHA_HOLD;
			print("Stall protection engaged at AOA [deg] = ", alpha * 180.0f / PI, MESSAGE_NOTE);
		}
	}
}

/*
 * Compute an aileron command with roll damper and bank controller;
 * Controller gains must be computed via 'update' function prior
 * @param phi				Measured bank angle [rad]
 * @param P					Measured roll rate [rad/s]
 * @param delta_A_cmd		Normalized transmitter aileron command without trim
 * @return					Closed loop aileron command to be forwarded to the servos
 */
float StabilityAugmentationSystem::stabilize_roll(float phi, float P, float delta_A_cmd)
{
	// Integrate the commanded roll rate to the commanded bank angle
	if((!this->bank_bypass) && (abs(delta_A_cmd) > 0.01))
	{
		this->phi_cmd += -delta_A_cmd * this->P_max / 50.0f;

		if(this->phi_cmd > PI)
			this->phi_cmd -= 2.0f * PI;
		else if(this->phi_cmd < -PI)
			this->phi_cmd += 2.0f * PI;
	}

	// Limit bank angle in stall protection mode
	if(this->stall_mode != SAS_STALL_MODE_STBY)
	{
		if(this->phi_cmd > SAS_STALLPROT_BANK_LIMIT)
			this->phi_cmd = SAS_STALLPROT_BANK_LIMIT;
		else if(this->phi_cmd < -SAS_STALLPROT_BANK_LIMIT)
			this->phi_cmd = -SAS_STALLPROT_BANK_LIMIT;
	}

	// Compute the bank angle difference
	float delta_phi = this->phi_cmd - phi;

	if(delta_phi > PI)
	    delta_phi -= 2.0f * PI;
	else if(delta_phi < -PI)
	    delta_phi += 2.0f * PI;

	// Compute the inner control loop aileron command with the roll damper
	float delta_A = -this->k_P * P;

	// Add the outer loop aileron command with the bank controller
	if(this->bank_bypass)
		delta_A += (1.0f - this->k_P * this->P_max) * delta_A_cmd;
	else
		delta_A += this->k_phi * delta_phi;

	// Limit aileron deflection
	if(delta_A > SAS_AILERON_TRAVEL_LIMIT)
		delta_A = SAS_AILERON_TRAVEL_LIMIT;
	else if(delta_A < -SAS_AILERON_TRAVEL_LIMIT)
		delta_A = -SAS_AILERON_TRAVEL_LIMIT;

	return delta_A;
}

/*
 * Compute an elevator command when the stall protection kicks in;
 * Controller gains must be computed via 'update' function prior
 * @param theta				Measured pitch angle [rad]
 * @param Q					Measured pitch rate [rad/s]
 * @param alpha				Measured angle of attack [rad]
 * @param V_T				Measured airspeed [m/s]
 * @param delta_E_cmd		Normalized transmitter elevator command without trim
 * @return					Closed loop elevator command to be forwarded to the servo
 */
float StabilityAugmentationSystem::stabilize_pitch(float theta, float Q, float alpha, float V_T, float delta_E_cmd)
{
	float delta_E = delta_E_cmd;

	// Perform stall protection if needed
	if(this->stall_mode == SAS_STALL_MODE_ALPHA_HOLD)
	{
		// Pertubation variables at current timestep
		float delta_VT = V_T - this->x_0(0);
		float delta_alpha = alpha - this->x_0(1);
		float delta_Q = Q - this->x_0(2);

		// Predict pertubation variables at next timestep
		float delta_theta_k = SAS_STALLPROT_DELTA_T * this->a(0) * delta_Q;
	    float delta_VT_k = SAS_STALLPROT_DELTA_T * ((1.0f / SAS_STALLPROT_DELTA_T + this->A(0, 0)) * delta_VT + \
	    					this->A(0, 1) * delta_alpha + \
							this->A(0, 2) * delta_Q + \
							this->dx(0));
	    float delta_alpha_k = SAS_STALLPROT_DELTA_T * (this->A(1, 0) * delta_VT + \
	    						(1.0f / SAS_STALLPROT_DELTA_T + this->A(1, 1)) * delta_alpha + \
								this->A(1, 2) * delta_Q + \
								this->dx(1));
		float delta_Q_k = (-this->a(1) * delta_theta_k - \
							this->A(1, 0) * delta_VT_k - \
							(1.0f / SAS_STALLPROT_DELTA_T + this->A(1, 1)) * delta_alpha_k - \
							this->dx(1)) / this->A(1, 2);

		// Compute elevator deflection for correcting pertubations at next timestep
		delta_E = (delta_Q_k / SAS_STALLPROT_DELTA_T - \
					this->A(2, 0) * delta_VT - \
					this->A(2, 1) * delta_alpha - \
					(1.0f / SAS_STALLPROT_DELTA_T + this->A(2, 2)) * delta_Q - \
					this->dx(2)) / this->B;
	}
	else if(this->stall_mode == SAS_STALL_MODE_GAMMA_HOLD)
	{
		// Zero elevator until alpha falls below 'SAS_STALLPROT_ALPHA_TARGET'
		delta_E = 0;
	}

	// Add an elevator trim in dependence of flap setting
	delta_E += this->elevator_trim;

	// Limit elevator deflection
	if(delta_E > SAS_ELEVATOR_TRAVEL_LIMIT)
		delta_E = SAS_ELEVATOR_TRAVEL_LIMIT;
	else if(delta_E < -SAS_ELEVATOR_TRAVEL_LIMIT)
		delta_E = -SAS_ELEVATOR_TRAVEL_LIMIT;

	return delta_E;
}

/*
 * Compute a throttle command when the stall protection kicks in;
 * Controller gains must be computed via 'update' function prior
 * @param delta_T_cmd		Normalized transmitter throttle command
 * @return					Closed loop throttle command to be forwarded to the ESC
 */
float StabilityAugmentationSystem::stabilize_speed(float delta_T_cmd)
{
	// Idle throttle if not yet airborn
	if(this->ground_lock)
		return -1.0f;

	float delta_T = delta_T_cmd;

	// Command some minimum throttle if stall protection active
	if((this->stall_mode != SAS_STALL_MODE_STBY) && (delta_T < SAS_STALLPROT_MIN_THROTTLE))
		delta_T = SAS_STALLPROT_MIN_THROTTLE;

	// Check if throttle was frozen by a completed stall protection
	if(this->throttle_freeze && (delta_T < SAS_STALLPROT_MIN_THROTTLE))
		delta_T = SAS_STALLPROT_MIN_THROTTLE;
	else
		this->throttle_freeze = 0;

	return delta_T;
}

/*
 * Compute a rudder command with yaw damper and turn coordinator;
 * Controller gains must be computed via 'update' function prior
 * @param beta				Measured sideslip angle [rad]
 * @param R					Measured yaw rate [rad/s]
 * @param delta_R_cmd		Normalized transmitter rudder command without trim
 * @return					Closed loop rudder command to be forwarded to the servo
 */
float StabilityAugmentationSystem::stabilize_yaw(float beta, float R, float delta_R_cmd)
{
	// Compute the inner control loop rudder command with the yaw damper
	float delta_R = -this->k_R / (1.0f / this->T_R + 100.0f) * this->H_R.update(R);

	// Add the outer loop rudder command with the turn coordinator
	this->beta_cmd = this->beta_max * delta_R_cmd;
	this->delta_R_beta += this->k_beta * this->H_beta.update(this->beta_cmd - beta);

	if(this->delta_R_beta > SAS_RUDDER_TRAVEL_LIMIT)
		this->delta_R_beta = SAS_RUDDER_TRAVEL_LIMIT;
	else if(this->delta_R_beta < -SAS_RUDDER_TRAVEL_LIMIT)
		this->delta_R_beta = -SAS_RUDDER_TRAVEL_LIMIT;

	delta_R += this->delta_R_beta;

	// Limit rudder deflection
	if(delta_R > SAS_RUDDER_TRAVEL_LIMIT)
		delta_R = SAS_RUDDER_TRAVEL_LIMIT;
	else if(delta_R < -SAS_RUDDER_TRAVEL_LIMIT)
		delta_R = -SAS_RUDDER_TRAVEL_LIMIT;

	return delta_R;
}

/*
 * Perform a self test of bank controller by applying a
 * linearized bank and roll system to the controller inputs
 * @param delta_R_cmd		Normalized transmitter aileron command without trim
 * @return					Closed loop aileron command to be forwarded to the servos
 */
float StabilityAugmentationSystem::self_test_roll(float delta_A_cmd)
{
	// Check if self test is allowed
	if(!this->ground_lock)
		print("SAS Self Test activated while not allowed", MESSAGE_WARNING);

	// Fake a controller target and apply the controller
	if(delta_A_cmd < -0.1f)
		this->phi_cmd = 1.0f;
	else if(delta_A_cmd > 0.1f)
		this->phi_cmd = -1.0f;
	else
		this->phi_cmd = 0;

	float delta_A = this->stabilize_roll(this->G_phi_deltaA.filtered, this->G_P_deltaA.filtered, 0);

	// Update the linearized system responses
	this->G_phi_deltaA.update(delta_A);
	this->G_P_deltaA.update(delta_A);

	return delta_A;
}

/*
 * Perform a self test of turn coordinator by applying a
 * linearized sideslip and yaw system to the controller inputs
 * @param delta_R_cmd		Normalized transmitter rudder command without trim
 * @return					Closed loop rudder command to be forwarded to the servos
 */
float StabilityAugmentationSystem::self_test_yaw(float delta_R_cmd)
{
	// Check if self test is allowed
	if(!this->ground_lock)
		print("SAS Self Test activated while not allowed", MESSAGE_WARNING);

	// Fake a controller target and apply the controller
	if(delta_R_cmd < -0.1f)
		delta_R_cmd = -1.0f;
	else if(delta_R_cmd > 0.1f)
		delta_R_cmd = 1.0f;
	else
		delta_R_cmd = 0;

	float delta_R = this->stabilize_yaw(this->G_beta_deltaR.filtered, this->G_R_deltaR.filtered, delta_R_cmd);

	// Update the linearized system responses
	this->G_beta_deltaR.update(delta_R);
	this->G_R_deltaR.update(delta_R);

	return delta_R;
}
