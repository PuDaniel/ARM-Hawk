/*
 * System.cpp
 *
 *  Created on: 06. Aug. 2016
 *      Author: Daniel
 */

#include "System.hpp"

/**
 * Constructor for the Message object
 */
Message::Message()
{
	this->text[0] = '\0';
	this->priority = MESSAGE_NOTE;
	this->tick = 0;
}

/**
 * Constructor for the Message object
 * @param msgText 		The text (string) for the message object
 * @param msgPriority 	The priority of the Message object (Warning, Caution, Note, Debug)
 * @param msgTime 		Timestamp of the current system tick
 */
Message::Message(const char* msgText, char msgPriority, int msgTick)
{
	// Initialize the message first with standard data
	this->text[0] = '\0';
	this->priority = MESSAGE_NOTE;
	this->tick = 0;

	// Now initialize the message with the given data
	this->set(msgText, msgPriority, msgTick);
}

/**
 * Set all variables of the message at once
 * @param msgText 		The text (string) for the message object
 * @param msgPriority 	The priority of the Message object (Warning, Caution, Note, Debug)
 * @param msgTime 		Timestamp of the current system tick
 */
void Message::set(const char* msgText, char msgPriority, int msgTick) volatile
{
	// Check if string is too long
	if(strlen(msgText) > 64)
	{
		memcpy(const_cast<char*>(this->text), msgText, 64);
		this->text[64] = '\0';
	}
	else
	{
		strcpy(const_cast<char*>(this->text), msgText);
	}

	this->priority = msgPriority;
	this->tick = msgTick;
}

/**
 * Get a full message string for debug printout
 * @param destination 	A pointer to the buffer the string should be written to (at least 85 bytes long)
 */
void Message::get(char* destination) volatile
{
	// Begin a new line
	destination[0] = '\n';

	// Write message header
	if(this->priority == MESSAGE_DEBUG)
		strcpy(destination + 1, "[DEBUG   at ");
	else if(this->priority == MESSAGE_NOTE)
		strcpy(destination + 1, "[NOTE    at ");
	else if(this->priority == MESSAGE_CAUTION)
		strcpy(destination + 1, "[CAUTION at ");
	else if(this->priority == MESSAGE_WARNING)
		strcpy(destination + 1, "[WARNING at ");

	// Write timestamp (wrapped around at 999999)
	toString(this->tick % 999999, 6, destination + 13);

	// Close header
	destination[19] = ']';
	destination[20] = ' ';

	// Copy message body
	strcpy(destination + 21, const_cast<char*>(this->text));
}

/**
 * Constructor for the MessageBuffer object
 */
MessageBuffer::MessageBuffer()
{
	this->status = 0;
}

/**
 * Add a message to the buffer and mark as new (status)
 * @param msgText 		The text (string) for the message object
 * @param msgPriority 	The priority of the Message object (Warning, Caution, Note, Debug)
 * @param msgTime 		16-bit timestamp of the current system tick after log beginn
 */
void MessageBuffer::add(const char* msgText, char msgPriority, int msgTick) volatile
{
	// Search for an empty place to put the new message in (Slot 0 is reserved for overflow error)
	for(int i = 1; i < 32; i++)
	{
		if(!(this->status & (1 << i)))
		{
			// Empty slot found, add data
			this->message[i].set(msgText, msgPriority, msgTick);

			this->status |= (1 << i);
			return;
		}
	}

	// No empty slot found => overflow error
	this->message[0].set("Message Buffer Overflow", MESSAGE_CAUTION, msgTick);
	this->status |= (1 << 0);
	sys.masterCaution = 1;
}

/**
 * Get the next new message from the buffer if there is any
 * @param destination 	A pointer to the buffer the string should be written to (at least 85 bytes big)
 */
void MessageBuffer::receive(char* destination) volatile
{
	for(int i = 0; i < 32; i++)
	{
		if(this->status & (1 << i))
		{
			// New message found, generate it and set to read
			this->message[i].get(destination);
			this->status &= ~(1 << i);
			return;
		}
	}
}

/**
 * Get the amount of new messages stored in the buffer
 * @return	Amount of messages stored
 */
int MessageBuffer::count() volatile
{
	int count = 0;

	for(int i = 0; i < 32; i++)
		if(this->status & (1 << i))
			count++;

	return count;
}

/**
 * Constructor of System class.
 * Always first to be called at program start and manages a clean initialization.
 * Sets clockspeed and inits all generall system peripherals.
 */
NO_OPTIMIZATION System::System()
{
	// Initialise all Systems, temporary stop all interrupts
	__disable_irq();
	__DSB();
	__ISB();

	// Init variables
	this->autopilot_mode = AUTOPILOT_MODE_STANDBY;
	this->tick = 0;
	this->time = 0;
	this->profiling_time = 0;

	this->masterCaution = 0;
	this->masterWarning = 0;

	// Init clock at 216MHz
	setSystemClock_216();

	// Init system peripherals
	initSystemPeripherals();

	// Enable LEDs to indicate startup
	sys.LedGreen_set();
	sys.LedRed_set();
	sys.LedBlue_set();
}

/**
 * Configure the PLL to run the system clock at 216MHz
 */
NO_OPTIMIZATION void System::setSystemClock_216()
{
	// Change Flash latency in accordance with Sysclock
	do
		FLASH->ACR = FLASH_ACR_LATENCY_7WS;
	while(!(FLASH->ACR & FLASH_ACR_LATENCY_7WS));		// Check if latency change was successfull

	// Initialize System Clock to 216MHz (from 8MHz external clock source)
	do
	{
		// Enable external oszillator and bypass
		RCC->CR |= RCC_CR_HSEBYP;
		RCC->CR |= RCC_CR_HSEON;
		while(!(RCC->CR & RCC_CR_HSERDY));			// Wait till HSE locked (ready)

		// Configure PLL (M=4; N=216; P=2; Q=9; Source=HSE)
		RCC->PLLCFGR = (4 << 0) | (216 << 6) | (0 << 16) | (9 << 24) | RCC_PLLCFGR_PLLSRC_HSE;
		RCC->CR |= RCC_CR_PLLON;					// Enable PLL
		while(!(RCC->CR & RCC_CR_PLLRDY));			// Wait till PLL locked (ready)

		RCC->CFGR = RCC_CFGR_SW_PLL | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;		// Set PLL as Sysclock and set APB prescaler
	}
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));		// Check if change was successfull

	SystemCoreClockUpdate();
}

/**
 * Initialize all system relevant peripherals (Debug LEDs, SysTick)
 */
NO_OPTIMIZATION void System::initSystemPeripherals()
{
	// Initialize Debug LEDs (PB0 & PB7 & PB14 as output)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER14_0;

	// Initialize User Button (PC13)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// Set Timer14 for 50Hz main loop
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->PSC = 99;				// 1.08MHz clock after prescaler
	TIM14->ARR = 21599;				// 50Hz counter frequency
	TIM14->CR1 = TIM_CR1_CEN;		// Enable timer

	// Set System-Tick to 1ms elapse time (source = processor clock)
	SysTick->LOAD = 215999;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	// Configure USART3 for sending debug information
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= GPIO_MODER_MODER8_1;			// Set PD8 to AF mode
	GPIOD->AFR[1] |= (7 << 0);						// Link PD8 to AF7

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->DCKCFGR2 |= RCC_DCKCFGR2_USART3SEL_0;		// System clock as source
	USART3->BRR = 0x03A9;							// Configure Baud Rate 230400
	USART3->CR1 = USART_CR1_TXEIE | USART_CR1_TE;	// Enable Transmitter and Transmit Register Empty Interrupt
	USART3->CR1 |= USART_CR1_UE;					// Enable USART3

	NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * Ends the initialization of the program to continue with the main execution.
 * Always call at last before entering the mainloop.
 */
NO_OPTIMIZATION void System::finish_initialization()
{
	// Disable LEDs to indicate startup complete
	sys.LedGreen_clear();
	sys.LedRed_clear();
	sys.LedBlue_clear();

	// Set sytsem tick to 1
	this->tick = 1;
	this->time = 1.0 / MAINLOOP_FREQUENCY;

	// Re-Initialize Timer14
	TIM14->CNT = 0;					// Set to zero
	TIM14->SR &= ~TIM_SR_UIF;		// Clear flag

	// Re-Enable all interrupts
	__enable_irq();
	__DSB();
	__ISB();
}

/**
 * Wait until the mainloop has reached 20ms execution time for steady 50Hz operation
 */
void NO_OPTIMIZATION System::finish_mainloop()
{
	// Check if main loop took more than 20ms time
	if(TIM14->SR & TIM_SR_UIF)
		print("Mainloop time overrun", MESSAGE_WARNING);

	// Wait until 20ms elapsed
	while(!(TIM14->SR & TIM_SR_UIF));

	TIM14->SR &= ~TIM_SR_UIF;			// Clear flag

	// Increase system tick & time
	this->tick++;
	this->time += 1.0f / MAINLOOP_FREQUENCY;
}

/**
 * Start of the profiling sequence.
 * Measures the approximate execution time of all code between 'profiling_start()'
 * and 'profiling_end()' and prints the result via a debug message.
 * To be valid, t has to be less than 1 mainloop cycle.
 */
void NO_OPTIMIZATION System::profiling_start()
{
	this->profiling_time = TIM14->CNT;
}

/**
 * End of the profiling sequence.
 * Measures the approximate execution time aof all code between 'profiling_start()'
 * and 'profiling_end()' and prints the result via a debug message.
 * To be valid, t has to be less than 1 mainloop cycle.
 */
void NO_OPTIMIZATION System::profiling_end()
{
	this->profiling_time = (TIM14->CNT - this->profiling_time);
	float elapsed_time = this->profiling_time / 1080.0f;

	print("Profiling result: t [ms]= ", elapsed_time, MESSAGE_DEBUG);
}

/**
 * Check if the user button has been clicked (pressed and released)
 * Has to be called regularly for correct results
 */
char System::userButton_clicked()
{
	char clicked = 0;

	if(this->userButton_pressed())
	{
		this->button_state = 1;
	}
	else
	{
		if(this->button_state)
			clicked = 1;

		this->button_state = 0;
	}

	return clicked;
}

/**
 * Wait the specified amount of time in milliseconds
 * @param ms 	Amount of time to be waited before function return in milliseconds
 */
void NO_OPTIMIZATION waitMs(int ms)
{
	SysTick->VAL = 0;		// Restart SysTick from zero

	// Count the number of elapsed SysTick events
	for(int i=0; i<ms; i++)
		while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}

/**
 * Put the given message into the message queue for further handling
 * @param message 	The message string (max 64 characters)
 * @param priority	The priority of the Message object (Warning, Caution, Note, Debug)
 */
void NO_OPTIMIZATION print(const char* message, char priority)
{
	// Add the message to all buffers as required
	debug_messageBuffer.add(message, priority, sys.tick);

	if(priority != MESSAGE_DEBUG)
		datalog_messageBuffer.add(message, priority, sys.tick);

	// Set Master Caution/Warning if needed
	if(priority == MESSAGE_CAUTION)
		sys.masterCaution = 1;
	else if(priority == MESSAGE_WARNING)
		sys.masterWarning = 1;

	// Continue sending over serial debug
	NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * Put the given message into the message queue for further handling with an additional floating point number
 * @param message 	The message string (max 64 characters minus the characters in number)
 * @param number	A signed floating point (double) number which gets added after the message string
 * @param priority	The priority of the Message object (Warning, Caution, Note, Debug)
 */
void print(const char* message, double number, char priority)
{
	char mBuffer[100];				// Buffer slightly larger than needed to avoid overflow
	strcpy(mBuffer, message);
	toString(number, 5, mBuffer + strlen(message));

	print(mBuffer, priority);		// Maximum length of 64 characters will be checked here
}

/**
 * Put the given message into the message queue for further handling with an additional integer
 * @param message 	The message string (max 64 characters minus the characters in number)
 * @param number	A signed fixed point (int) number which gets added after the message string
 * @param priority	The priority of the Message object (Warning, Caution, Note, Debug)
 */
void print(const char* message, int number, char priority)
{
	char mBuffer[100];				// Buffer slightly larger than needed to avoid overflow
	strcpy(mBuffer, message);
	toString(number, mBuffer + strlen(message));

	print(mBuffer, priority);		// Maximum length of 64 characters will be checked here
}

/**
 * Put the given message into the message queue for further handling with an additional hexadecimal integer
 * Implements further options for number formatting
 * @param message 	The message string (max 64 characters minus the characters in number)
 * @param number	An unsigned fixed point (int) number which gets added after the message string
 * @param option	Specifies how the integer shoud be formatted
 * @param priority	The priority of the Message object (Warning, Caution, Note, Debug)
 */
void print(const char* message, uint32_t number, char option, char priority)
{
	char mBuffer[100];				// Buffer slightly larger than needed to avoid overflow
	strcpy(mBuffer, message);
	toStringHex(number, mBuffer + strlen(message));

	print(mBuffer, priority);		// Maximum length of 64 characters will be checked here
}

/**
 * Convert an integer to an ASCII number (text)
 * @param number	Integer number to be converted
 * @param ascii		Pointer to the buffer the converted string should be written to
 * @return			Length of converted string in characters
 */
int toString(int number, char* ascii)
{
	// Check if we can skip some work
	if(number == 0)
	{
		ascii[0] = '0';
 		ascii[1] = '\0';
 		return (1);
 	}

	// Fix sign
	int sign = 1;

 	if(number < 0)
 	{
 		ascii[0] = '-';
 		ascii++;
 		sign = -1;
 		number = std::abs(number);
 	}

 	// Check how long the number is
 	int length = 1;
 	int divisor = 10;
 	while(number / divisor != 0)
 	{
 		length++;
 		divisor *= 10;
 	}

	// Convert the number
	for(int i=length-1; i>=0; i--)
 	{
		ascii[i] = (number % 10) + '0';
		number /= 10;
	}

	// Finish string
	ascii[length]='\0';

	if(sign == 1)
		return length;
	else
		return length + 1;
}

/**
 * Convert an integer to an ASCII number (text) with leading zeros
 * @param number		Integer number to be converted
 * @param digitCount	Number of characters the converted string should be long (minimum)
 * @param ascii			Pointer to the buffer the converted string should be written to
 * @return				Length of converted string in characters
 */
int toString(int number, int digitCount, char* ascii)
{
	int length = toString(number, ascii);

	if(length < digitCount)
	{
		// Move the whole string backwards
		for(int i=0; i<length; i++)
			ascii[digitCount-1-i] = ascii[length-1-i];

		// Fill the first digits with leading zeros
		for(int i=0; i<(digitCount-length); i++)
			ascii[i] = '0';

		// Close string
		ascii[digitCount] = '\0';

		return (digitCount);
	}
	else
	{
		return length;
	}
}

/**
 * Convert a double to an ASCII number (text)
 * @param number		Double number to be converted
 * @param commaCount	Amount of positions after the comma the converted string should have
 * @param ascii			Pointer to the buffer the converted string should be written to
 * @return				Length of converted string in characters
 */
int toString(double number, int decimalCount, char* ascii)
{
	// Split number into two parts
	int pre_decimal = (int)number;
	int post_decimal = (int)std::abs((number - pre_decimal) * pow(10, decimalCount));

	// Convert everything before the decimal
	int length = 0;

	if(number < 0)
	{
		ascii[0] = '-';
		length++;
	}

 	length += toString(std::abs(pre_decimal), ascii + length);

 	// Add the comma
 	ascii[length] = '.';
 	length++;

 	// Convert everything after the decimal
 	length += toString(post_decimal, decimalCount, ascii + length);

 	return length;
}

/**
 * Convert an integer to an ASCII number (text) in HEX format
 * @param number	Unsigned 32 bit integer to be converted
 * @param ascii		Pointer to the buffer the converted string should be written to
 * @return			Length of converted string in characters
 */
int toStringHex(uint32_t number, char* ascii)
{
	// Write the HEX identifier
	ascii[0] = '0';
	ascii[1] = 'x';

	// Convert the number
	for(int i=7; i>=0; i--)
	{
		ascii[i+2] = (number % 16) + '0';

		if(ascii[i+2] > '9')
			ascii[i+2] += 7;

		number = (number >> 4);
	}

	// Finish string
	ascii[10]='\0';

	return 10;
}

/**
 * Convert a float to an ASCII number (text) utilizing an exponential notation
 * @param number		Float number to be converted
 * @param commaCount	Amount of positions after the comma the converted string should have
 * @param ascii			Pointer to the buffer the converted string should be written to
 * @return				Length of converted string in characters
 */
int toStringScientific(float number, int decimalCount, char* ascii)
{
	// Get the exponent and shift the number
	int exponent = 0;

	// Compare if the number is within some working range
	if((fabs(number) > 1e-20) && (fabs(number) < 1e20))
	{
		while(fabs(number) >= 10.0)
		{
			// Number bigger than 1 decimal digit => increase exponent
			number /= 10.0;
			exponent += 1;
		}

		while(fabs(number) < 1.0)
		{
			// Number smaller than 1 decimal digit => decrease exponent
			number *= 10.0;
			exponent -= 1;
		}
	}
	else
	{
		number = 0;
	}

	// Split number into two parts
	int pre_decimal = (int)number;
	int post_decimal = (int)std::abs((number - pre_decimal) * pow(10, decimalCount));

	// Convert everything before the decimal
	int length = 0;

	if(number < 0)
	{
		ascii[0] = '-';
		length++;
	}

 	length += toString(std::abs(pre_decimal), ascii + length);

 	// Add the comma
 	ascii[length] = '.';
 	length++;

 	// Convert everything after the decimal
 	length += toString(post_decimal, decimalCount, ascii + length);

 	// Add the exponent
 	ascii[length] = 'e';
 	length++;

 	length += toString(exponent, ascii + length);

 	return length;
}

// Buffer for shifting out a message over the serial port
volatile char debugSerialBuffer[85] = {'\0'};
volatile int debugSerialBuffer_position = 0;

/**
 * Push the characters stored in debug_messageBuffer out over USART3
 */
extern "C" void NO_OPTIMIZATION USART3_IRQHandler(void)
{
	// Check if sentence transmission complete ('\0' character)
	if(debugSerialBuffer[debugSerialBuffer_position] == '\0')
	{
		if(debug_messageBuffer.status == 0)
		{
			// Nothing to do, stop all interrupts
			NVIC_DisableIRQ(USART3_IRQn);
		}
		else
		{
			// Send the next message with status = 1 (new message)
			debug_messageBuffer.receive(const_cast<char*>(debugSerialBuffer));
			debugSerialBuffer_position = 0;
		}
	}
	else
	{
		// Send next character
		USART3->TDR = debugSerialBuffer[debugSerialBuffer_position];
		debugSerialBuffer_position++;
	}

	// Clear the interrupt flag and wait until it is definitely cleared
	NVIC_ClearPendingIRQ(USART3_IRQn);
	__DSB();
	__ISB();
}
