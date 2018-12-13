/*
 * FlightControls.cpp
 *
 *  Created on: May 24, 2018
 *      Author: Daniel
 */

#include "FlightControls.hpp"

char Servo::initialized = 0;

/*
 * Initialize hardware including timers to output PWM signals,
 * if not already done so.
 * @param channel		One of 6 channels to output the signals
 * @param lower_limit	Travel point for -100% input
 * @param offset		Mid-point of servo travel
 * @param upper_limit	Travel point for +100% input
 */
NO_OPTIMIZATION Servo::Servo(char channel, float lower_limit, float offset, float upper_limit)
{
	this->channel = channel;

	this->lower_limit = lower_limit;
	this->offset = offset;
	this->upper_limit = upper_limit;

	this->position = 0;

	// Check if hardware is already initialized
	if(!this->initialized)
	{
		// Init GPIOA PA0, PA1, PA2 & PA3 as very high speed, alternate function 2 for Timer5
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
		GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3;
		GPIOA->AFR[0] |= (2 << 0) | (2 << 1*4) | (2 << 2*4) | (2 << 3*4);

		// Init GPIOE PE5 & PE6 as very high speed, alternate function 3 for Timer9
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
		GPIOE->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1;
		GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6;
		GPIOE->AFR[0] |= (3 << 5*4) | (3 << 6*4);

		// Init Timer5 CH1, CH2, CH3 & CH4 for high precision servo PWM output
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;			// Enable peripheral clock
		TIM5->PSC = 0;								// Prescaler = 1 => Frequency = 108MHz
		TIM5->ARR = 2160000 - 1;					// Frequency = 50Hz
		TIM5->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
						TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
		TIM5->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE |
						TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;		// Enable PWM mode 1 and buffering of CCR
		TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;	// Enable capture compare outputs
		TIM5->CCR1 = 0;
		TIM5->CCR2 = 0;
		TIM5->CCR3 = 0;
		TIM5->CCR4 = 0;								// Set all PWMs to zero (idle all servos)
		TIM5->EGR |= TIM_EGR_UG;					// Update all registers by issuing an update event
		TIM5->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;		// Buffer auto reload register, Enable Timer

		// Init Timer9 CH1 & CH2 for standard precision servo PWM output
		RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;			// Enable peripheral clock
		TIM9->PSC = 71;								// Prescaler = 72 => Frequency = 3MHz
		TIM9->ARR = 60000 - 1;						// Frequency = 50Hz
		TIM9->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
						TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;		// Enable PWM mode 1 and buffering of CCR
		TIM9->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;	// Enable capture compare outputs
		TIM9->CCR1 = 0;
		TIM9->CCR2 = 0;								// Set all PWMs to zero (idle all servos)
		TIM9->EGR |= TIM_EGR_UG;					// Update all registers by issuing an update event
		TIM9->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;		// Buffer auto reload register, Enable Timer

		this->initialized = 1;
	}

	// Set the servo to 0
	this->update(0.0f);
}

/*
 * Adjusts the PWM signal to drive the servo to a new position
 * @param position	Normalized position value ranging from -1.0 to 1.0
 */
void NO_OPTIMIZATION Servo::update(float position)
{
	// Check travel limit
	if(position < -1.0f)
	{
		print("Servo reached lower travel limit; Channel: ", this->channel, MESSAGE_WARNING);
		return;
	}
	else if(position > 1.0f)
	{
		print("Servo reached upper travel limit; Channel: ", this->channel, MESSAGE_WARNING);
		return;
	}

#ifndef FC_DISABLED
	this->position = position;

	// Convert normalized position input to general value in microseconds
	float position_us = 0;

	if(this->position < 0.0f)
		position_us = this->offset + this->position * (this->offset - this->lower_limit);
	else
		position_us = this->offset + this->position * (this->upper_limit - this->offset);

	// Convert microseconds to timer value and write right register according to channel
	if(this->channel == 1)
		TIM5->CCR1 = (int)roundf(position_us * 108.0f) - 1;
	else if(this->channel == 2)
		TIM5->CCR2 = (int)roundf(position_us * 108.0f) - 1;
	else if(this->channel == 3)
		TIM5->CCR3 = (int)roundf(position_us * 108.0f) - 1;
	else if(this->channel == 4)
		TIM5->CCR4 = (int)roundf(position_us * 108.0f) - 1;
	else if(this->channel == 5)
		TIM9->CCR1 = (int)roundf(position_us * 3.0f) - 1;
	else if(this->channel == 6)
		TIM9->CCR2 = (int)roundf(position_us * 3.0f) - 1;
#endif
}

// Initialization of static members of Receiver class
volatile uint8_t Receiver::sbus_buffer[25];
volatile int Receiver::sbus_pos = 0;
volatile int Receiver::usart_errors = 0;
volatile int Receiver::sbus_errors = 0;
volatile int Receiver::idle_time = 0;
volatile uint8_t Receiver::sbus_frame[25];

/*
 * Initialize all hardware necessary for reading the S.BUS input of the RC receiver
 */
NO_OPTIMIZATION Receiver::Receiver()
{
	this->frames_lost = 0;
	this->failsafe = 0;

	// Pause USART6 interrupt
	NVIC_DisableIRQ(USART6_IRQn);

	// Init GPIOC PC7 as very high speed, alternate function 8 for USART6
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;
	GPIOC->AFR[0] |= (8 << (7 * 4));

	// Init USART6 RX for S.BUS receiption
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;				// Enable peripheral clock
	RCC->DCKCFGR2 |= RCC_DCKCFGR2_USART6SEL_0;			// Set system clock as source
	USART6->CR1 = USART_CR1_M_0 | USART_CR1_PCE | USART_CR1_PEIE | USART_CR1_RXNEIE | USART_CR1_RE;		// Format 8E2, parity error & RX interrupt, enable RX
	USART6->CR2 = USART_CR2_RXINV | USART_CR2_STOP_1;	// Invert RX pin, 2 Stop bits
	USART6->CR3 = USART_CR3_EIE;						// Enable error interrupt
	USART6->BRR = 2160;									// Baud = 100000
	USART6->CR1 |= USART_CR1_UE;						// Enable USART6

	// Init Timer13 for S.BUS frame idle detection
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;		// Enable peripheral clock
	TIM13->PSC = 9;								// Prescaler = 10 => Frequency = 10.8MHz
	TIM13->ARR = 53999;							// Frequency = 200Hz
	TIM13->DIER = TIM_DIER_UIE;					// Enable update (overflow) interrupt
	TIM13->CR1 = TIM_CR1_CEN;					// Enable timer

#ifndef FC_DISABLED
	// Enable interrupts
	NVIC_EnableIRQ(USART6_IRQn);
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
#else
	// Set Master Caution if hardware is temporarly disabled for debugging
	print("Flight controls disabled for debugging", MESSAGE_CAUTION);
#endif
}

/*
 * Updates the channel data from a complete S.BUS frame.
 * Must be called in every mainloop cycle
 */
void NO_OPTIMIZATION Receiver::update()
{
#ifndef FC_DISABLED
	// Pause TIM13 interrupts until frame is processed to avoid write interference
	NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);

	// Decode the analog channels 1 ~ 16
	this->channel[0] = ((this->sbus_frame[2] << 8) | this->sbus_frame[1]) & 0x07FF;
	this->channel[1] = ((this->sbus_frame[3] << 5) | (this->sbus_frame[2] >> 3)) & 0x07FF;
	this->channel[2] = ((this->sbus_frame[5] << 10) | (this->sbus_frame[4] << 2) | (this->sbus_frame[3] >> 6)) & 0x07FF;
	this->channel[3] = ((this->sbus_frame[6] << 7) | (this->sbus_frame[5] >> 1)) & 0x07FF;
	this->channel[4] = ((this->sbus_frame[7] << 4) | (this->sbus_frame[6] >> 4)) & 0x07FF;
	this->channel[5] = ((this->sbus_frame[9] << 9) | (this->sbus_frame[8] << 1) | (this->sbus_frame[7] >> 7)) & 0x07FF;
	this->channel[6] = ((this->sbus_frame[10] << 6) | (this->sbus_frame[9] >> 2)) & 0x07FF;
	this->channel[7] = ((this->sbus_frame[11] << 3) | (this->sbus_frame[10] >> 5)) & 0x07FF;
	this->channel[8] = ((this->sbus_frame[13] << 8) | this->sbus_frame[12]) & 0x07FF;
	this->channel[9] = ((this->sbus_frame[14] << 5) | (this->sbus_frame[13] >> 3)) & 0x07FF;
	this->channel[10] = ((this->sbus_frame[16] << 10) | (this->sbus_frame[15] << 2) | (this->sbus_frame[14] >> 6)) & 0x07FF;
	this->channel[11] = ((this->sbus_frame[17] << 7) | (this->sbus_frame[16] >> 1)) & 0x07FF;
	this->channel[12] = ((this->sbus_frame[18] << 4) | (this->sbus_frame[17] >> 4)) & 0x07FF;
	this->channel[13] = ((this->sbus_frame[20] << 9) | (this->sbus_frame[19] << 1) | (this->sbus_frame[18] >> 7)) & 0x07FF;
	this->channel[14] = ((this->sbus_frame[21] << 6) | (this->sbus_frame[20] >> 2)) & 0x07FF;
	this->channel[15] = ((this->sbus_frame[22] << 3) | (this->sbus_frame[21] >> 5)) & 0x07FF;

	// Normalize the channel values
	for(int i = 0; i < 16; i++)
	{
		// Convert channel to percent
		this->channel[i] = (this->channel[i] - this->channel_center) * this->channel_scaling;

		// Subtract offset (center)
		this->channel[i] -= this->channel_limits[i][1] / 100.0f;

		// Scale lower/upper limit
		if(this->channel[i] > 0)
			this->channel[i] *= 100.0f / (this->channel_limits[i][2] - this->channel_limits[i][1]);
		else
			this->channel[i] *= (-100.0f) / (this->channel_limits[i][0] - this->channel_limits[i][1]);

		// Reverse channel if necessary
		this->channel[i] *= this->channel_direction[i];

		// Check channel limits
		if(this->channel[i] > 1.0f)
			this->channel[i] = 1.0f;
		else if(this->channel[i] < -1.0f)
			this->channel[i] = -1.0f;
	}

	// Get the two digital channels
	if(this->sbus_frame[23] & SBUS_CH17)
		this->channel[16] = 1.0f;
	else
		this->channel[16] = -1.0f;

	if(this->sbus_frame[23] & SBUS_CH18)
		this->channel[17] = 1.0f;
	else
		this->channel[17] = -1.0f;

	// Check the receiver state
	if(this->sbus_frame[23] & SBUS_FRAME_LOST)
		this->frames_lost++;

	if(this->sbus_frame[23] & SBUS_FAILSAFE)
		this->failsafe = 1;
	else
		this->failsafe = 0;

	// Resume interrupts
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
#endif
}

// Initialization of static members of Propeller class
volatile int Propulsion::input_capture_buffer = 0;
volatile int Propulsion::input_capture_counter = 0;

/*
 * Initialize all hardware for measuring the angular speed of the motor/propeller
 */
NO_OPTIMIZATION Propulsion::Propulsion(float Q_bat_init)
	: omega_P({1.0e+0, -1.646026632951584e+0, 6.855887866846833e-1}, {9.890538433274724e-3, 1.978107686654945e-2, 9.890538433274724e-3})
{
	this->I = 0;
	this->Q_bat_0 = Q_bat_init;
	this->Q_bat = Q_bat_init;
	this->U_bat = 42.0f;

	// Init GPIOF PF6 as very high speed, alternate function 3 for TIM10
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	GPIOF->MODER |= GPIO_MODER_MODER6_1;
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6;
	GPIOF->AFR[0] |= (3 << 24);

	// Init Timer5 CH1 for input capture
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;			// Enable peripheral clock
	TIM10->PSC = 63;								// Prescaler = 64 => Frequency = 3.375MHz
	TIM10->ARR = 0xFFFF;							// Frequency = 51.498Hz
	TIM10->CCMR1 = TIM_CCMR1_CC1S_0;				// Set CH1 as input, no prescaler, no filter
	TIM10->CCER = TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E;		// Enable capture on both edges
	TIM10->DIER = TIM_DIER_CC1IE;					// Enable capture/compare 1 interrupt
	TIM10->EGR |= TIM_EGR_UG;						// Update all registers by issuing an update event
	TIM10->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;		// Buffer auto reload register, Enable Timer

	// Enable interrupts
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/*
 * Calculates the current motor/propeller angular velocity
 * has to be called each mainloop cycle
 * @param throttle		Commanded value for the throttle channel
 */
void NO_OPTIMIZATION Propulsion::update(float throttle)
{
	// Temporarly disable input capture interrupt
	NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);

	// Calculate angular velocity from stored input capture data
	if((this->input_capture_counter > 0) && (this->input_capture_buffer > 0))
	{
		float period = ((float)this->input_capture_buffer / (float)this->input_capture_counter) / (216.0e6f / (TIM10->PSC + 1));
		this->omega_P.update((1.0f / (2.0f * period)) * 2.0f * PI / this->pole_pairs);
	}
	else
	{
		this->omega_P.update(0.0f);
	}

	// Compute the ESC throttle command (account for throttle idle deadzone)
	throttle = 0.75f / 1.7f * (throttle - 1.0f) + 1.0f;
	if(throttle < 0.25)
		throttle = 0;

	// Compute motor current with motor equations
	float I_old = this->I;
	this->I = (throttle * this->U_bat - this->omega_P.filtered / this->k_V) / this->R;
	if(this->I < 0.0f)
		this->I = 0.0f;
	else if(this->I > 90.0f)
		this->I = 90.0f;

	// Update the battery electric charge and the idle voltage
	this->Q_bat -= (I_old + this->I) * 0.01f / 3.6f;
	if(this->Q_bat > 0.0f)
		this->U_bat = 42.0f - (1.0f - this->Q_bat / this->Q_bat_0) * 6.0f;
	else
		this->U_bat = 36.0f;

	// Reset all buffers for next frame
	this->input_capture_buffer = 0;
	this->input_capture_counter = 0;

	// Reenable input capture interrupt
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/**
 * Calculate a throttle value to be sent to the ESC with
 * a normalized throttle input (0~1)
 * @param throttle			Normalized (between 0 and 1) throttle input
 * @return					ESC throttle command (accounted for throttle idle deadzone)
 */
float Propulsion::throttle_cmd(float throttle)
{
	if(throttle < 0.25)
			return -1.0f;
	else
		return 1.7f / 0.75f * (throttle - 1.0f) + 1.0f;
}

/*
 * Interrupt handler for USART6
 * Receive one byte from RC receiver via S.BUS and decode if frame is complete
 */
extern "C" void NO_OPTIMIZATION USART6_IRQHandler(void)
{
	// Check for any bus errors
	if(USART6->ISR & USART6_ERROR_MASK)
	{
		Receiver::usart_errors++;

		// Clear errors and flush receive register
		USART6->ICR |= USART6_ERROR_CLEAR;
		USART6->RQR |= USART_RQR_RXFRQ;

		// Check if error limit reached
		if(Receiver::usart_errors > RECEIVER_MAX_USART_ERRORS)
			print("S.BUS (USART6) repeated error with ISR = ", USART6->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);

		return;
	}
	else
	{
		// Everything OK => reset error counter
		Receiver::usart_errors = 0;
	}

	// Check if frame overrun occured
	if(Receiver::sbus_pos > 24)
	{
		print("S.BUS (USART6) frame overflow", MESSAGE_WARNING);
		return;
	}

	// Add the new byte to the correct position in the frame
	Receiver::sbus_buffer[Receiver::sbus_pos] = USART6->RDR;
	Receiver::sbus_pos++;

	// Ongoing transaction => reset Timer13
	TIM13->CNT = 0;

	// Clear the interrupt flag and wait until it is definitely cleared
	NVIC_ClearPendingIRQ(USART6_IRQn);
	__DSB();
	__ISB();
}

/*
 * Interrupt handler for Timer13
 * USART6 has been idle for at least 1ms => reset the SBUS frame
 */
extern "C" void NO_OPTIMIZATION TIM8_UP_TIM13_IRQHandler(void)
{
	// Check the received frame for validity
	if(Receiver::sbus_pos == 0)
	{
		// No new data since last idle interrupt
		Receiver::idle_time += 5;

		// Check if error limit reached
		if(Receiver::idle_time > RECEIVER_MAX_IDLE_TIME)
			print("RC receiver disconnected", MESSAGE_WARNING);
	}
	else if((Receiver::sbus_pos != 25) || (Receiver::sbus_buffer[0] != 0x0F))
	{
		// Received frame is not valid
		Receiver::sbus_errors++;

		// Check if error limit reached
		if(Receiver::sbus_errors > RECEIVER_MAX_SBUS_ERRORS)
			print("S.BUS repeated frame error", MESSAGE_WARNING);
	}
	else
	{
		// Everything OK => Reset error counter/idle time and copy frame
		Receiver::sbus_errors = 0;
		Receiver::idle_time = 0;

		for(int i = 0; i < 25; i++)
			Receiver::sbus_frame[i] = Receiver::sbus_buffer[i];
	}

	// Reset the input buffer
	Receiver::sbus_pos = 0;

	// Clear all Timer13 status flags
	TIM13->SR = 0;

	// Clear the interrupt flag and wait until it is definitely cleared
	NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
	__DSB();
	__ISB();
}

/*
 * Interrupt handler for Timer10
 * Read in and store the input capture channel for calculation of omega_P
 */
extern "C" void NO_OPTIMIZATION TIM1_UP_TIM10_IRQHandler(void)
{
	// Store the measured half-phase length
	if(!(TIM10->SR & TIM_SR_UIF))
	{
		Propulsion::input_capture_buffer += TIM10->CCR1;
		Propulsion::input_capture_counter++;

		TIM10->CNT = 0;
	}

	// Clear all Timer10 status flags
	TIM10->SR = 0;

	// Clear the interrupt flag and wait until it is definitely cleared
	NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
	__DSB();
	__ISB();
}
