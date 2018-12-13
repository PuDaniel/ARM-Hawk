/*
 * System.hpp
 *
 *  Created on: 06. Aug. 2016
 *      Author: Daniel
 */

#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

// !!! ONLY INCLUDE BUILT IN MODULES !!!
#include "stm32f7xx.h"
#include <cstring>
#include <cstdint>
#include <cmath>

// Temporarly disable some hardware if needed
//#define ADC_DISABLED
//#define GPS_DISABLED
//#define IMU_DISABLED
//#define FC_DISABLED

// Put this in front of functions that access MCU registers in a time critical manner
#ifndef DEBUG
#define NO_OPTIMIZATION __attribute__((optimize("O0")))
#else
#define NO_OPTIMIZATION
#endif

// Other defines
#define MESSAGE_DEBUG		(char) -1
#define MESSAGE_NOTE		(char) 0
#define MESSAGE_CAUTION		(char) 1
#define MESSAGE_WARNING		(char) 2

#define MESSAGE_OPTION_HEX	(char) 1

#define MAINLOOP_FREQUENCY	50.0f			// Targeted frequency of mainloop in Hz

#define AUTOPILOT_MODE_STANDBY		(char) 0
#define AUTOPILOT_MODE_DIRECT		(char) 1
#define AUTOPILOT_MODE_STABILITY	(char) 2

// Class definitions
/**
 * Creates a message object with some text, priority and time stamp
 */
class Message
{
public:
	// Buffer for holding the message text
	volatile char text[65];

	// Defines how important the message is (Note, Caution, Warning)
	volatile char priority;

	// Set the system tick the message raised for further analysis afterwards
	volatile int tick;

	void set(const char* msgText, char msgPriority, int msgTick) volatile;
	void get(char* destination) volatile;

	Message();
	Message(const char* msgText, char msgPriority, int msgTick);
};

/**
 * Creates a buffer for holding different messages complete with status handling
 */
class MessageBuffer
{
public:
	// Buffer for holding up to 32 messages at once
	volatile Message message[32];

	// Bitfield to indicate the status of the messages (1 == new message; has to be sent)
	volatile uint32_t status;

	void add(const char* msgText, char msgPriority, int msgTick) volatile;
	void receive(char* destination) volatile;

	int count() volatile;

	MessageBuffer();
};

/**
 * System class handles all general purpose tasks and functions.
 * Content is accessible by all subsystems.
 */
class System
{
	// Counter for profiling system speed
	uint16_t profiling_time;

	// Saves the last button state for the 'button clicked' poll
	char button_state;

	void setSystemClock_216();
	void initSystemPeripherals();

public:
	// State for fail-safe: Malfunction occurred but operation may continue safely
	volatile char masterCaution;

	// State for fail-safe: Severe malfunction occurred and safe operation cannot be continued
	volatile char masterWarning;

	// Autopilot state (Standby, Direct, Stability Augmentation)
	volatile char autopilot_mode;

	// Track the current time (in seconds) since startup finished (before startup = 0)
	float time;

	// Track the current mainloop ticks since startup finished (before startup = 0)
	int tick;

	// Functions for green debug LED control
	void LedGreen_set() { GPIOB->BSRR = GPIO_BSRR_BS_0; }
	void LedGreen_clear() { GPIOB->BSRR = GPIO_BSRR_BR_0; }
	void LedGreen_toggle() { GPIOB->ODR ^= GPIO_ODR_ODR_0; }

	// Functions for blue debug LED control
	void LedBlue_set() { GPIOB->BSRR = GPIO_BSRR_BS_7; }
	void LedBlue_clear() { GPIOB->BSRR = GPIO_BSRR_BR_7; }
	void LedBlue_toggle() { GPIOB->ODR ^= GPIO_ODR_ODR_7; }

	// Functions for red debug LED control
	void LedRed_set() { GPIOB->BSRR = GPIO_BSRR_BS_14; }
	void LedRed_clear() { GPIOB->BSRR = GPIO_BSRR_BR_14; }
	void LedRed_toggle() { GPIOB->ODR ^= GPIO_ODR_ODR_14; }

	// Function for polling the User Button
	char userButton_pressed() { return (GPIOC->IDR & GPIO_IDR_IDR_13) != 0; }
	char userButton_clicked();

	// Functions for syncing the system
	void finish_initialization();
	void finish_mainloop();

	// Functions for profiling
	void profiling_start();
	void profiling_end();

	System();
};

// Object and variable definitions
extern System sys;

extern volatile MessageBuffer debug_messageBuffer;
extern volatile MessageBuffer datalog_messageBuffer;

extern volatile char debugSerialBuffer[85];
extern volatile int debugSerialBuffer_position;

// General functions
void waitMs(int ms);

void print(const char* message, char priority);
void print(const char* message, int number, char priority);
void print(const char* message, double number, char priority);
void print(const char* message, uint32_t number, char option, char priority);

// String conversion functions
int toString(int number, char* ascii);
int toString(int number, int digitCount, char* ascii);
int toString(double number, int decimalCount, char* ascii);
int toStringHex(uint32_t number, char* ascii);
int toStringScientific(float number, int decimalCount, char* ascii);

#endif /* SYSTEM_HPP_ */
