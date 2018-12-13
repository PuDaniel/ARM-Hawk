/*
 * FlightControls.hpp
 *
 *  Created on: May 24, 2018
 *      Author: Daniel
 */

#ifndef FLIGHTCONTROLS_HPP_
#define FLIGHTCONTROLS_HPP_

// Built-in modules
#include "stm32f7xx.h"
#include <cstdint>

// Own modules
#include "System.hpp"
#include "ArmHawk_math.hpp"

// S.BUS defines
#define SBUS_CH17			(1 << 0)
#define SBUS_CH18			(1 << 1)
#define SBUS_FRAME_LOST		(1 << 2)
#define SBUS_FAILSAFE		(1 << 3)

// Define all possible errors on the USART
#define USART6_ERROR_MASK	(USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)
#define USART6_ERROR_CLEAR	(USART_ICR_ORECF | USART_ICR_NCF | USART_ICR_FECF | USART_ICR_PECF)

// Define limits for allowed consecutive errors for the receiver
#define RECEIVER_MAX_USART_ERRORS	2
#define RECEIVER_MAX_SBUS_ERRORS	1
#define RECEIVER_MAX_IDLE_TIME		100

/*
 * Object wich holds the interface to position a servo with
 * some modifications like reversing, offset and travel.
 * 6 Channels available: 	CH1-4 = High precision
 * 							CH5-6 = Standard precision
 */
class Servo
{
	// Boolean to indicate if Timers has already been initialized
	static char initialized;

	// One of 6 channels, where signal shall be outputed
	char channel;

	// Lower limit point of servo travel which wraps to -1.0 input; in microseconds
	float lower_limit;

	// Offset of the servos zero position (input = 0.0); in microseconds
	float offset;

	// Upper limit point of servo travel which wraps to +1.0 input; in microseconds
	float upper_limit;

public:
	// Normalized position the servo should currently be positioned at
	float position;

	void update(float position);

	Servo(char channel, float lower_limit, float offset, float upper_limit);
	Servo(char channel) : Servo(channel, 1100, 1520, 1940) {}
};

/*
 * Reads the S.BUS signals from the RC receiver and decodes
 * all 18 channels into normalized values (-1.0 ~ 1.0)
 * Channels 17 and 18 only digital (-1.0 / +1.0)
 */
class Receiver
{
	// Center value (0%) in S.BUS serial data
	const float channel_center = 1024.0;

	// Scaling to calculate a channel value (1.0 = 100%) from S.BUS serial data
	const float channel_scaling = 1.0 / 672.0;

	// Channel limits in percent as configured in the transmitter: [lower limt, center, upper limit]
	const int channel_limits[16][3] = {	{-115,	-2,	110},
										{-89,	-1,	93},
										{-100,	0,	100},
										{-92,	-1,	115},
										{-75,	68,	211},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100},
										{-100,	0,	100} };

	// Sign of the channel direction to convert to normalized input values
	const int channel_direction[16] = {	-1, -1, +1, +1, -1, +1, +1, +1,
										+1, +1, +1, +1, +1, +1, +1, +1 };
public:
	// Receive buffer and frame position pointer for S.BUS
	static volatile uint8_t sbus_buffer[25];
	static volatile int sbus_pos;

	// Counter for amount of consecutive errors occuring at USART6
	static volatile int usart_errors;

	// Counter for amount of consecutive broken S.BUS frames
	static volatile int sbus_errors;

	// Counts the milliseconds of idle time between the frames
	static volatile int idle_time;

	// Stores a complete valid frame for decoding the channel data
	static volatile uint8_t sbus_frame[25];

	// Counter for the number of lost frames of the receiver
	int frames_lost;

	// Boolean for indicating failsafe mode
	char failsafe;

	// Normalized channel values (-1.0 ~ 1.0)
	float channel[18];

	void update();

	Receiver();
};

/*
 * Tracks the state of the electric motor
 */
class Propulsion
{
	// Electrical resistance of the propulsion system [Ohm]
	const float R = 0.128f;

	// Motor back-EMF constant [1/Vs]
	const float k_V = 22.5f;

	// Number of pole pairs of brushless motor
	const float pole_pairs = 14;

	// Maximum battery capacity [mAh]
	float Q_bat_0;

public:
	// All input captures of one mainloop cycle get added here
	static volatile int input_capture_buffer;

	// Number of input captures per mainloop cycle
	static volatile int input_capture_counter;

	// Measured motor/propeller angular velocity [rad/s] (2nd order Bessel lowpass, Fc = 1Hz)
	Filter<3, 3, float> omega_P;

	// Calculated motor current [A]
	float I;

	// Current battery capacity [mAh]
	float Q_bat;

	// Idle battery voltage [V]
	float U_bat;

	void update(float throttle);

	static float throttle_cmd(float throttle);

	Propulsion(float Q_bat_init);
};

#endif /* FLIGHTCONTROLS_HPP_ */
