/*
 * InertialNavigationSystem.cpp
 *
 *  Created on: Apr 13, 2018
 *      Author: Daniel
 */

#include "InertialNavigationSystem.hpp"

// Initialize IMU_bus class
char IMU_bus::initialized = 0;

/**
 * Initialize the I2C bus for communication
 */
NO_OPTIMIZATION IMU_bus::IMU_bus()
{
	// Check if Hardware is already initialized
	if(this->initialized)
		return;

	// Initialize PF0 & PF1 as alternate function 4 (I2C2) with open drain outputs
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	GPIOF->OTYPER |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1;
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOF->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOF->AFR[0] |= (4 << 0) | (4 << 4);

	// Initialize I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->DCKCFGR2 |= RCC_DCKCFGR2_I2C2SEL_1;	// Select HSI clock as source (16MHz)
	I2C2->CR1 &= ~I2C_CR1_PE;					// Disable I2C2
	I2C2->TIMINGR |= (0x00 << 28) | (0x02 << 20) | (0x00 << 16) | (0x02 << 8) | (0x04 << 0);	// 1000kHz clock
	I2C2->TIMEOUTR |= I2C_TIMEOUTR_TIMOUTEN | (0 << 0);		// Enable timeout; timeout = 62.5ns*2048*1 = 128us
	I2C2->CR1 |= I2C_CR1_PE;					// Enable I2C2

	this->initialized = 1;
}

/**
 * Read one byte from the specified device after providing the register.
 * First sends a write command with 1 byte containing the register address,
 * then sends a repeated start condition with a following read command for 1 byte.
 * @param address		Address of slave device
 * @param reg			Register address to read from
 * @return				The received register value
 */
uint8_t NO_OPTIMIZATION IMU_bus::readRegister(uint8_t address, uint8_t reg)
{
	uint8_t data = 0;

	// Clear all raised errors
	I2C2->ICR |= I2C2_ERROR_CLEAR;

	// Transfer length = 1, write, slave address, start communication
	I2C2->CR2 = (1 << 16) | ((address & 0x7F) << 1) | I2C_CR2_START;

	// Wait until ready to transmit or error occured
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C2_ERROR_MASK)));

	if(I2C2->ISR & I2C_ISR_TXIS)
	{
		// Transfer length = 1, read, slave address, repeated start
		I2C2->CR2 = 0;
		I2C2->CR2 |= (1 << 16) | I2C_CR2_RD_WRN | ((address & 0x7F) << 1) | I2C_CR2_START;

		// Write one byte (register address)
		I2C2->TXDR = reg;
	}
	else
	{
		// Error occured => Terminate
		print("I2C2 readRegister start error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return 0;
	}

	// Wait until transfer complete
	while(!(I2C2->ISR & (I2C_ISR_TXE | I2C2_ERROR_MASK)));

	if(!(I2C2->ISR & I2C_ISR_TXE))
	{
		// Error occured => Terminate
		print("I2C2 readRegister TX error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return 0;
	}

	// Wait until one byte received or error occured
	while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C2_ERROR_MASK)));

	if(I2C2->ISR & I2C_ISR_RXNE)
	{
		// Send stop condition
		I2C2->CR2 |= I2C_CR2_STOP;

		// Read one byte
		data = I2C2->RXDR;
	}
	else
	{
		// Error occured => Terminate
		print("I2C2 readRegister RX error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return 0;
	}

	// Wait until transfer complete
	while(I2C2->ISR & I2C_ISR_BUSY);

	return data;
}

/**
 * Read the specified number of bytes from the specified device after providing the register.
 * First sends a write command with 1 byte containing the register address,
 * then sends a repeated start condition with a following read command for n bytes.
 * @param address		Address of slave device
 * @param reg			Register address to read from
 * @param buffer		Pointer to byte buffer the data should be read into
 * @param nBytes		Number of bytes to read from the slave (max. 255)
 */
void NO_OPTIMIZATION IMU_bus::readRegister_multiple(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes)
{
	// Clear all raised errors
	I2C2->ICR |= I2C2_ERROR_CLEAR;

	// Transfer length = 1, write, slave address, start communication
	I2C2->CR2 = (1 << 16) | ((address & 0x7F) << 1) | I2C_CR2_START;

	// Wait until ready to transmit or error occured
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C2_ERROR_MASK)));

	if(I2C2->ISR & I2C_ISR_TXIS)
	{
		// Transfer length = nBytes, read, slave address, repeated start
		I2C2->CR2 = 0;
		I2C2->CR2 |= (nBytes << 16) | I2C_CR2_RD_WRN | ((address & 0x7F) << 1) | I2C_CR2_START;

		// Write one byte (register address)
		I2C2->TXDR = reg;
	}
	else
	{
		// Error occured => Terminate
		print("I2C2 readRegister_multiple start error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Wait until transfer complete
	while(!(I2C2->ISR & (I2C_ISR_TXE | I2C2_ERROR_MASK)));

	if(!(I2C2->ISR & I2C_ISR_TXE))
	{
		// Error occured => Terminate
		print("I2C2 readRegister_multiple TX error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Receive all bytes
	for(int i = 0; i < nBytes; i++)
	{
		// Wait until one byte received or error occured
		while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C2_ERROR_MASK)));

		if(I2C2->ISR & I2C_ISR_RXNE)
		{
			// Send stop condition if this is the last byte
			if(i == (nBytes - 1))
				I2C2->CR2 |= I2C_CR2_STOP;

			// Read one byte
			buffer[i] = I2C2->RXDR;
		}
		else
		{
			// Error occured => Terminate
			print("I2C2 readRegister_multiple RX error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			return;
		}
	}

	// Wait until transfer complete
	while(I2C2->ISR & I2C_ISR_BUSY);
}

/**
 * Write one byte to the specified device after providing the register.
 * Sends a write command with 2 bytes containing the register address and given data.
 * @param address		Address of slave device
 * @param reg			Register address to write to
 * @param data			Data byte which should be written
 */
void NO_OPTIMIZATION IMU_bus::writeRegister(uint8_t address, uint8_t reg, uint8_t data)
{
	// Clear all raised errors
	I2C2->ICR |= I2C2_ERROR_CLEAR;

	// Autoend mode, transfer length = 2, slave address, write, start communication
	I2C2->CR2 = I2C_CR2_AUTOEND | (2 << 16) | ((address & 0x7F) << 1) | I2C_CR2_START;

	// Wait until ready to transmit or error occured
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C2_ERROR_MASK)));

	if(I2C2->ISR & I2C_ISR_TXIS)
	{
		// Write one byte (register address)
		I2C2->TXDR = reg;
	}
	else
	{
		// Error occured => Terminate
		print("I2C2 writeRegister start error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Wait until ready to transmit or error occured
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C2_ERROR_MASK)));

	if(I2C2->ISR & I2C_ISR_TXIS)
	{
		// Transmit data byte
		I2C2->TXDR = data;
	}
	else
	{
		// Error occured => Terminate
		print("I2C2 writeRegister TX error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Wait until transfer complete
	while(!(I2C2->ISR & (I2C_ISR_TXE | I2C2_ERROR_MASK)));

	if(!(I2C2->ISR & I2C_ISR_TXE))
	{
		// Error occured => Terminate
		print("I2C2 writeRegister finish error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Wait until transfer complete
	while(I2C2->ISR & I2C_ISR_BUSY);
}

/**
 * Write the specified number of bytes to the specified device after providing the register.
 * Sends a write command with n+1 bytes containing the register address and given data.
 * @param address		Address of slave device
 * @param reg			Register address to write to
 * @param buffer		Pointer to byte buffer the data should be written from
 * @param nBytes		Number of bytes to write to the slave (max. 255)
 */
void NO_OPTIMIZATION IMU_bus::writeRegister_multiple(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes)
{
	// Clear all raised errors
	I2C2->ICR |= I2C2_ERROR_CLEAR;

	// Autoend mode, transfer length = nBytes + 1, slave address, write, start communication
	I2C2->CR2 = I2C_CR2_AUTOEND | ((nBytes + 1) << 16) | ((address & 0x7F) << 1) | I2C_CR2_START;

	// Wait until ready to transmit or error occured
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C2_ERROR_MASK)));

	if(I2C2->ISR & I2C_ISR_TXIS)
	{
		// Write one byte (register address)
		I2C2->TXDR = reg;
	}
	else
	{
		// Error occured => Terminate
		print("I2C2 writeRegister_multiple start error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Transmit all data bytes
	for(int i = 0; i < nBytes; i++)
	{
		// Wait until ready to transmit or error occured
		while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C2_ERROR_MASK)));

		if(I2C2->ISR & I2C_ISR_TXIS)
		{
			// Write one byte
			I2C2->TXDR = buffer[i];
		}
		else
		{
			// Error occured => Terminate
			print("I2C2 writeRegister_multiple TX error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			return;
		}
	}

	// Wait until transfer complete
	while(!(I2C2->ISR & (I2C_ISR_TXE | I2C2_ERROR_MASK)));

	if(!(I2C2->ISR & I2C_ISR_TXE))
	{
		// Error occured => Terminate
		print("I2C2 writeRegister_multiple finish error with ISR= ", I2C2->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Wait until transfer complete
	while(I2C2->ISR & I2C_ISR_BUSY);
}

/**
 * Initialize the FXAS21002 gyroscope
 */
FXAS21002::FXAS21002()
{
	// Indicate initialization phase
	this->running = 0;

#ifndef IMU_DISABLED
	// Initialize PG5 (INT1) & PG4 (INT2) as input
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;

	// Switch to standby mode (required for initializing registers)
	this->bus.writeRegister(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::CTRL_REG1, 0);

	// Set FIFO to circular mode, watermark at 16 samples
	this->bus.writeRegister(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::F_SETUP, (0b01 << 6) | 16);

	// LPF cutoff = 256Hz (ODR = 800Hz), no HPF, range = 500dps
	this->bus.writeRegister(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::CTRL_REG0, (0b10 << 0));

	// FIFO interrupt enabled on INT1, INT pins polarity: active high, push-pull
	this->bus.writeRegister(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::CTRL_REG2, (1 << 7) | (1 << 6) | (1 << 1));

	// Auto increment wrap-over to address 0x01
	this->bus.writeRegister(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::CTRL_REG3, (1 << 3));

	// ODR = 800Hz, select active mode
	this->bus.writeRegister(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::CTRL_REG1, (1 << 1));
#else
	// Set Master Caution if hardware is temporarly disabled for debugging
	print("IMU (FXAS21002) disabled for debugging", MESSAGE_CAUTION);
#endif
}

/**
 * Read the sensor's FIFO and calculate the new filtered angular rates.
 * Has to be called periodically in the application's main loop
 */
void FXAS21002::update(const Vector<3>& f)
{
#ifndef IMU_DISABLED
	// Retrieve the amount of data samples in the FIFO
	uint8_t fifo_status = this->bus.readRegister(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::F_STATUS) & 0x3F;
	int fifo_count = fifo_status & 0x3F;

	// Check if the FIFO is empty
	if((fifo_count == 0)  && this->running)
		print("FXAS21002 FIFO empty error", MESSAGE_WARNING);

	// Check for FIFO overflow
	if((fifo_status & (1 << 7)) && this->running)
		print("FXAS21002 FIFO overflow error", MESSAGE_WARNING);

	// Get all data from the FIFO
	uint8_t buffer[192];

	this->bus.readRegister_multiple(FXAS21002_CMD::ADDRESS, FXAS21002_CMD::OUT_X_MSB, buffer, 6 * fifo_count);

	// Combine and scale the data to rad/s
	for(int i = 0; i < fifo_count; i++)
	{
		// Angular rate in IC axes
		float x = (int16_t)((buffer[i*6] << 8) | buffer[i*6+1]) * this->sensitivity_gyro;
		float y = (int16_t)((buffer[i*6+2] << 8) | buffer[i*6+3]) * this->sensitivity_gyro;
		float z = (int16_t)((buffer[i*6+4] << 8) | buffer[i*6+5]) * this->sensitivity_gyro;

		// Angular rate in IMU axes
		Vector<3> omega_raw({y, x, -z});

		// Apply calibration
		this->omega.push(this->C_gyro * (omega_raw - this->omega_0) - this->G * f);
	}
#else
	for(int i = 0; i < 16; i++)
	{
		this->omega.push(Vector<3, float>(0));
	}
#endif

	this->running = 1;
}

/**
 * Initialize the FXOS8700 accelerometer and magnetometer.
 */
FXOS8700::FXOS8700()
{
	// Indicate initialization phase
	this->running = 0;

#ifndef IMU_DISABLED
	// Initialize PG8 (INT1) & PG14 (INT2) as input
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;

	// Switch to standby mode (required for initializing registers)
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::CTRL_REG1, 0);

	// Set FIFO to circular mode, watermark at 8 samples
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::F_SETUP, (0b01 << 6) | 8);

	// Range = 8g, no HPF
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::XYZ_DATA_CFG, (0b10 << 0));

	// OSR = 2 (high resolution, ODR = 400Hz, hybrid)
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::CTRL_REG2, (0b10 << 0));

	// INT pins polarity: active high, push-pull
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::CTRL_REG3, (1 << 1));

	// FIFO interrupt enable
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::CTRL_REG4, (1 << 6));

	// FIFO interrupt on INT1
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::CTRL_REG5, (1 << 6));

	// Initialize all magnetometer registers before switching to active mode

	// OSR = 2 (ODR = 400Hz, hybrid), select hybrid mode
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::M_CTRL_REG1, (7 << 2) | (0b11 << 0));

	// Disable hybrid auto increment mode, disable max/min measurement
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::M_CTRL_REG2, (1 << 4));

	// Disable automatic offset compensation
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::M_CTRL_REG3, (1 << 7));

	// ODR = 400Hz (hybrid mode), select active mode
	this->bus.writeRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::CTRL_REG1, (1 << 0));
#else
	// Set Master Caution if hardware is temporarly disabled for debugging
	print("IMU (FXOS8700) disabled for debugging", MESSAGE_CAUTION);
#endif
}

/**
 * Read the sensor's FIFO and calculate the new filtered acceleration and magnetic flux.
 * Has to be called periodically in the application's main loop
 * @param motor_current 	Propulsion motor current draw for magnetometer compensation [A]
 */
void FXOS8700::update(float motor_current)
{
#ifndef IMU_DISABLED
	// Retrieve the amount of data samples in the FIFO
	uint8_t fifo_status = this->bus.readRegister(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::STATUS) & 0x3F;
	int fifo_count = fifo_status & 0x3F;

	// Check if the FIFO is empty
	if((fifo_count == 0)  && this->running)
		print("FXOS8700 FIFO empty error", MESSAGE_WARNING);

	// Check for FIFO overflow
	if((fifo_status & (1 << 7)) && this->running)
		print("FXOS8700 FIFO overflow error", MESSAGE_WARNING);

	// Get all acceleration data from the FIFO
	uint8_t buffer[192];

	this->bus.readRegister_multiple(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::OUT_X_MSB, buffer, 6 * fifo_count);

	// Only store the last 40ms of data (discard the rest)
	if(fifo_count > 16)
		fifo_count = 16;

	// Combine and scale the acceleration data to m/s^2
	for(int i = 0; i < fifo_count; i++)
	{
		// Acceleration in IC axes
		float x = ((int16_t)((buffer[i*6] << 8) | buffer[i*6+1]) >> 2) * this->sensitivity_acc;
		float y = ((int16_t)((buffer[i*6+2] << 8) | buffer[i*6+3]) >> 2) * this->sensitivity_acc;
		float z = ((int16_t)((buffer[i*6+4] << 8) | buffer[i*6+5]) >> 2) * this->sensitivity_acc;

		// Specific force (= acceleration - gravity) in IMU axes
		Vector<3> f_raw({y, x, -z});

		// Apply accelerometer calibration
		this->f.push(this->C_acc * (f_raw - this->f_0));
	}

	// Get one sample of magnetic flux density
	this->bus.readRegister_multiple(FXOS8700_CMD::ADDRESS, FXOS8700_CMD::M_OUT_X_MSB, buffer, 6);

	// Combine and scale the magnetic flux density data to T (in IC axes)
	float x = (int16_t)((buffer[0] << 8) | buffer[1]) * this->sensitivity_mag;
	float y = (int16_t)((buffer[2] << 8) | buffer[3]) * this->sensitivity_mag;
	float z = (int16_t)((buffer[4] << 8) | buffer[5]) * this->sensitivity_mag;

	// Magnetic flux density in IMU axes
	Vector<3> B_raw({y, x, -z});

	// Apply magnetometer calibration
	this->B = this->C_mag * (B_raw - this->B_0 - motor_current * this->B_I);
#else
	for(int i = 0; i < 8; i++)
	{
		this->f.push(Vector<3, float>(0));
	}

	this->B = Vector<3, float>(0);
#endif

	this->running = 1;
}

/**
 * Initialize the Coordinate object
 */
Coordinate::Coordinate()
{
	this->degrees = 0.0;
	this->minutes = 0.0;
}

/**
 * Constructor for the Coordinate object
 * @param deg	Degree value of the coordinate without decimals
 * @param min	Minute value of coordinate with seconds as decimals
 */
Coordinate::Coordinate(float deg, float min)
{
	this->degrees = deg;
	this->minutes = min;
}

/**
 * Constructor for the Coordinate object
 * @param deg	Degree value of the coordinate with minutes and seconds as decimals
 */
Coordinate::Coordinate(double deg)
{
	this->degrees = (int)deg;
	this->minutes = (deg - this->degrees) * 60.0;
}

/**
 * Convert the coordinate to a fractional value of radians
 * @return	Coordinate value in radians with decimals
 */
double Coordinate::get_radians()
{
	return ((double)(this->degrees) + (double)(this->minutes) / 60.0) * PI / 180.0;
}

/**
 * Convert the coordinate to a fractional value of degrees
 * @return	Coordinate value in degrees with decimals
 */
double Coordinate::get_degrees()
{
	return (double)(this->degrees) + (double)(this->minutes) / 60.0;
}

/**
 * Convert the coordinate to a fractional values of minutes
 * @return	Coordinate value in minutes with decimals
 */
double Coordinate::get_minutes()
{
	return (double)(this->degrees) * 60.0 + (double)(this->minutes);
}

/**
 * Convert the coordinate to a fractional values of seconds
 * @return	Coordinate value in seconds with decimals
 */
double Coordinate::get_seconds()
{
	return (double)(this->degrees) * 360.0 + (double)(this->minutes) * 60.0;
}

// Initialization of static members of Venus638 class
volatile char Venus638::NMEA_sentence[7][80];
volatile char Venus638::NMEA_ready[7] = {0};
volatile uint8_t Venus638::rx_buffer = 0;
volatile uint8_t Venus638::rx_position = 0;
volatile int Venus638::receiveErrors = 0;

/**
 * Constructor for Venus638 class; Init all peripherals (UART)
 */
NO_OPTIMIZATION Venus638::Venus638()
{
	// Init variables
	this->locked = 0;
	this->update_time = 0.0;
	this->checksumErrors = 0;
	this->altitude = 0;
	this->speed = 0;
	this->course = 0;
	this->mode = VENUS638_CMD::MODE::NOFIX;
	this->satellites = 0;
	this->hdop = 0;

	// Be sure that interrupt is deactivated
	NVIC_DisableIRQ(UART4_IRQn);

	// Init GPIOD for UART4
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;								// Enable peripheral clocks
	GPIOD->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;			// Set PD0 (RX) & PD1 (TX) as AF Mode
	GPIOD->AFR[0] |= (8 << 0) | (8 << 4);								// PD0 & PD1 to alternate function 7
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;	// Enable high speed on PD0 & PD1

	// Init UART4
	RCC->APB1ENR |= RCC_APB1ENR_UART4EN;								// Enable peripheral clocks
	RCC->DCKCFGR2 |= RCC_DCKCFGR2_UART4SEL_0;							// Set system clock as source
	UART4->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;		// Enable TX and RX; Enable RX Interrupt
	UART4->CR2 = ('$' << 24) | USART_CR2_ABRMODE_0 | USART_CR2_ABREN;	// Character match for '$'; Baud rate detection sequence '10xxxxxx'
	UART4->CR3 = USART_CR3_EIE | USART_CR3_OVRDIS;						// Enable interrupt on error; Disable overrun handling
	UART4->CR1 |= USART_CR1_UE;											// Enable UART4


#ifndef GPS_DISABLED
	// Change settings on the Venus638 and tune in the right baud rate
	this->setup();

	// Reenable interrupt
	NVIC_EnableIRQ(UART4_IRQn);

#else
	// Set Master Caution if hardware is temporarly disabled for debugging
	print("GPS disabled for debugging", MESSAGE_CAUTION);
#endif
}

/**
 * Automatically set the baud rate corresponding to the GPS receiver;
 * Disable UART4-NVIC before calling
 * @return 	The measured and tuned baud rate
 */
int NO_OPTIMIZATION Venus638::autoBaud()
{
	// Iterate until the auto baud detection was successfull (max. within 3 NMEA frames)
	for(int i = 0; i < 240; i++)
	{
		// Init baud rate; reset all Flags, flush receive buffer and request new auto baud detection
		UART4->BRR = 16;
		UART4->ICR |= USART_ICR_NCF | USART_ICR_FECF;
		UART4->RQR |= USART_RQR_ABRRQ | USART_RQR_RXFRQ;

		// Wait until complete or error
		while(!(UART4->ISR & (USART_ISR_ABRF | UART4_ERROR_MASK)));

		// Check if a trusted character (frame = '10xxxxxx') was received with no errors
		if(UART4->ISR & USART_ISR_ABRF)
			if(UART4->RDR == 0x0D)
				return SystemCoreClock / UART4->BRR;		// Everything fine => Return detected baud
	}

	// Couldn't detect a valid baud rate within 3 frames => Raise error
	print("UART4 auto baud detection failed with ISR= ", UART4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
	return 0;
}

/**
 * Send a command with the given payload to the GPS receiver and wait for a response
 * Disable UART-NVIC before calling
 * @param payload			Pointer to the buffer which holds the payload-bytes of the command
 * @param payload_length	Number of bytes in payload (max 65535 bytes)
 * @return 					Command response (ACK, NACK, TIMEOUT)
 */
int NO_OPTIMIZATION Venus638::sendCommand(char* payload, uint16_t payload_length)
{
	// Create command header
	char command[payload_length + 7] = {0};

	command[0] = 0xA0;
	command[1] = 0xA1;
	command[2] = (payload_length >> 8) & 0xFF;
	command[3] = payload_length & 0xFF;

	// Copy payload to command array and calculate checksum
	char checksum = 0;
	for(int i = 0; i < payload_length; i++)
	{
		command[i + 4] = payload[i];
		checksum ^= payload[i];
	}

	// Create command stop
	command[payload_length + 4] = checksum;
	command[payload_length + 5] = 0x0D;
	command[payload_length + 6] = 0x0A;

	// Send command array via UART4
	UART4->ICR |= USART_ICR_NCF | USART_ICR_FECF;		// Clear all error flags

	for(int i = 0; i < (payload_length + 7); i++)
	{
		// Wait until last byte copied to shift register or error
		while(!(UART4->ISR & (USART_ISR_TXE | UART4_ERROR_MASK)));

		if(UART4->ISR & USART_ISR_TXE)
		{
			// Everything fine => Continue normally
			UART4->TDR = command[i];
		}
		else
		{
			// Error occured => Terminate
			print("UART4 transmit error with ISR= ", UART4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			return VENUS638_CMD::BUS_ERROR;
		}
	}

	// Wait until last byte sent or error
	while(!(UART4->ISR & (USART_ISR_TC | UART4_ERROR_MASK)));

	if(!(UART4->ISR & USART_ISR_TC))
	{
		// Error occured => Terminate
		print("UART4 transmit finish error with ISR= ", UART4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return VENUS638_CMD::BUS_ERROR;
	}

	// Receive ACK or NACK
	char ACK[9] = {0xA0, 0xA1, 0x00, 0x02, 0x83, payload[0], (char)(0x83 ^ payload[0]), 0x0D, 0x0A};
	char NACK[9] = {0xA0, 0xA1, 0x00, 0x02, 0x84, payload[0], (char)(0x84 ^ payload[0]), 0x0D, 0x0A};
	char response[9] = {0};
	int pos = 0;

	for(int i = 0; i < 360; i++)
	{
		// Wait until byte received or error
		while(!(UART4->ISR & (USART_ISR_RXNE | UART4_ERROR_MASK)));

		if(UART4->ISR & USART_ISR_RXNE)
		{
			// Everything fine => Continue normally
			response[pos] = UART4->RDR;

			if(pos == 0)
			{
				// Check if start byte received
				if(response[pos] == 0xA0)
					pos = 1;					// Start byte received, set frame to start
			}
			else if(pos == 8)
			{
				// End of frame, check for ACK or NACK
				if(memcmp(response, ACK, 9) == 0)			// Compare for ACK
					return VENUS638_CMD::ACK;
				else if(memcmp(response, NACK, 9) == 0)		// Compare for NACK
					return VENUS638_CMD::NACK;
				else										// Wrong frame otherwise
					pos = 0;
			}
			else
			{
				// Proceed in frame
				pos++;
			}
		}
		else
		{
			// Error occured => Terminate
			print("UART4 receive error with ISR= ", UART4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			return VENUS638_CMD::BUS_ERROR;
		}
	}

	// Response did not occur within a reasonable timeframe -> return with error
	return VENUS638_CMD::TIMEOUT;
}

/**
 * Configure the gps receiver to run at the required settings (Baud, UDR, NMEA messages).
 * Disable UART4-NVIC before calling
 */
void Venus638::setup(void)
{
	// Be sure that the correct baud rate is tuned in
	this->autoBaud();

	// Configure serial port (Baud 115200)
	char cmd1[4] = {0x05, 0, VENUS638_CMD::BAUD::B115200, 0};
	int result = sendCommand(cmd1, 4);

	if(result != VENUS638_CMD::ACK)
	{
		// Did not revieve an ACK => Terminate
		print("GPS init failed at baud config with error code ", result, MESSAGE_WARNING);
		return;
	}

	// Change the baud rate to the new setting
	this->autoBaud();

	// Configure NMEA messages (only GPGGA and GPVTG)
	char cmd2[9] = {0x08, 1, 0, 0, 0, 1, 0, 0, 0};
	result = sendCommand(cmd2, 9);

	if(result != VENUS638_CMD::ACK)
	{
		// Did not revieve an ACK => Terminate
		print("GPS init failed at NMEA config with error code ", result, MESSAGE_WARNING);
		return;
	}

	// Configure update rate (10Hz)
	char cmd3[3] = {0x0E, 10, 0};
	result = sendCommand(cmd3, 3);

	if(result != VENUS638_CMD::ACK)
	{
		// Did not revieve an ACK => Terminate
		print("GPS init failed at UDR config with error code ", result, MESSAGE_WARNING);
		return;
	}
}

/**
 * Get the first character position in sentence of the given field
 * @param sentence	Pointer to the buffer which holds the NMEA message
 * @param field		Field number from the start (field == enclosed between ',')
 * @return			Position of field start in bytes of sentence
 */
int Venus638::getFieldStart(char* sentence, int field)
{
	int fieldCounter = 0;

	for(int i = 0; i < 79; i++)
	{
		if(sentence[i] == ',')
		{
			fieldCounter++;
			if(fieldCounter == field)
				return (i + 1);
		}
	}

	return 0;
}

/**
 * Get the last character position in NMEA_sentence of the given field
 * @param sentence	Pointer to the buffer which holds the NMEA message
 * @param field		Field number from the start (field == enclosed between ',')
 * @return			Position of field end in bytes of sentence
 */
int Venus638::getFieldEnd(char* sentence, int field)
{
	int fieldCounter = 0;

	for(int i = 0; i < 80; i++)
	{
		if((sentence[i] == ',') || (sentence[i] == '*'))
		{
			fieldCounter++;
			if(fieldCounter == field+1)
				return (i - 1);
		}
	}

	return 0;
}

/**
 * Decode the NMEA 0183 sentences received in the NMEA_sentence buffer.
 * Has to be called periodically in the application's main loop
 */
void Venus638::update(void)
{
#ifndef GPS_DISABLED
	for(int i = 0; i < 7; i++)
	{
		// Check if new data availabe
		if(this->NMEA_ready[i] == 0)
			continue;

		// Get the sentence that has to be processed
		char* sentence = const_cast<char*>(this->NMEA_sentence[i]);

		// Calculate checksum
		int NMEA_checksum_start = 0;
		char checksum = 0;

		for(int j = 1; j < 80; j++)
		{
			if(sentence[j] == '*')
			{
				NMEA_checksum_start = j + 1;
				break;
			}

			checksum ^= sentence[j];
		}

		// Convert the hexadecimal NMEA checksum to a decimal value
		char NMEA_checksum = ((sentence[NMEA_checksum_start] - '0') << 4);

		if(sentence[NMEA_checksum_start + 1] <= '9')
			NMEA_checksum |= sentence[NMEA_checksum_start + 1] - '0';
		else
			NMEA_checksum |= sentence[NMEA_checksum_start + 1] - 'A' + 10;

		// Compare the calculated with the transmitted checksum
		if(checksum != NMEA_checksum)
		{
			this->checksumErrors++;

			if(this->checksumErrors > GPS_MAX_CHECKSUM_ERRORS)
				print("GPS repeated checksum error. Errors: ", this->checksumErrors, MESSAGE_WARNING);

			this->NMEA_ready[i] = 0;

			continue;
		}
		else
		{
			this->checksumErrors = 0;
		}

		int fStart, fEnd;		// Holds field start and end values

		// Decode the NMEA sentence
		if(memcmp("GPGGA", sentence + 1, 5) == 0)
		{
			// Latitude [deg, min]
			fStart = getFieldStart(sentence, 2);

			this->latitude.degrees = stringToDouble(sentence + fStart, 2);
			this->latitude.minutes = stringToDouble(sentence + fStart + 2, 7);

			fStart = getFieldStart(sentence, 3);		// Fix sign
			if(sentence[fStart] == 'S')
				this->latitude.degrees *= -1;

			// Longitude [deg, min]
			fStart = getFieldStart(sentence, 4);

			this->longitude.degrees = stringToDouble(sentence + fStart, 3);
			this->longitude.minutes = stringToDouble(sentence + fStart + 3, 7);

			fStart = getFieldStart(sentence, 5);		// Fix sign
			if(sentence[fStart] == 'W')
				this->longitude.degrees *= -1;

			// Operating Mode
			fStart = getFieldStart(sentence, 6);
			this->mode = (int)(sentence[fStart] - '0');

			// Satellite count
			fStart = getFieldStart(sentence, 7);
			fEnd = getFieldEnd(sentence, 7);
			this->satellites = (int)stringToDouble(sentence + fStart, fEnd - fStart + 1);

			// Horizontal dilution of precision [m^2]
			fStart = getFieldStart(sentence, 8);
			fEnd = getFieldEnd(sentence, 8);
			this->hdop = stringToDouble(sentence + fStart, fEnd - fStart + 1);

			// Altitude [m]
			fStart = getFieldStart(sentence, 9);
			fEnd = getFieldEnd(sentence, 9);
			this->altitude = stringToDouble(sentence + fStart, fEnd - fStart + 1);
		}
		else if(memcmp("GPRMC", sentence + 1, 5) == 0)
		{
			// Speed [m/s]
			fStart = getFieldStart(sentence, 7);
			fEnd = getFieldEnd(sentence, 7);
			this->speed = stringToDouble(sentence + fStart, fEnd - fStart + 1) * 0.514445f;

			// Course [rad]
			fStart = getFieldStart(sentence, 8);
			fEnd = getFieldEnd(sentence, 8);
			this->course = stringToDouble(sentence + fStart, fEnd - fStart + 1) * PI / 180.0f;
			if(this->course > PI)
				this->course -= 2.0f * PI;
		}
		else if(memcmp("GPVTG", sentence + 1, 5) == 0)
		{
			// Course [rad]
			fStart = getFieldStart(sentence, 1);
			fEnd = getFieldEnd(sentence, 1);
			this->course = stringToDouble(sentence + fStart, fEnd - fStart + 1) * PI / 180.0f;
			if(this->course > PI)
				this->course -= 2.0f * PI;

			// Speed [m/s]
			fStart = getFieldStart(sentence, 2);
			fEnd = getFieldEnd(sentence, 2);
			this->speed = stringToDouble(sentence + fStart, fEnd - fStart + 1);
		}

		this->NMEA_ready[i] = 0;

		this->update_time = sys.time;
	}

	// Mark gps as ready, as soon as an accurate 3D position fix is established
	if((!this->locked) && (this->mode == VENUS638_CMD::MODE::FIX3D) && (this->hdop < GPS_HDOP_LIMIT))
	{
		this->locked = 1;
		print("GPS 3D position lock acquired", MESSAGE_NOTE);
	}

	// Check if a previous 3D position fix has been lost
	if(this->locked && (this->mode != VENUS638_CMD::MODE::FIX3D))
		print("GPS 3D position fix lost with mode = ", this->mode, MESSAGE_WARNING);

	// Check if hdop is in an reasonable range for accurate navigation
	if(this->locked && (this->hdop > GPS_HDOP_LIMIT))
		print("GPS accuracy error with HDOP = ", this->hdop, MESSAGE_CAUTION);

	// Check if a position update has recently been received
	if(sys.time > (this->update_time + 0.2))
		print("GPS update timeout", MESSAGE_WARNING);
#else
	this->latitude.degrees = 0;
	this->latitude.minutes = 0;
	this->longitude.degrees = 0;
	this->longitude.minutes = 0;
	this->mode = VENUS638_CMD::MODE::MANUAL;
	this->satellites = 0;
	this->hdop = GPS_HDOP_LIMIT;
	this->altitude = 0;
	this->speed = 0;
	this->course = 0;
	this->locked = 1;
#endif
}

/*
 * Initialize all components of the inertial navigation system
 */
InertialNavigationSystem::InertialNavigationSystem(Vector<3> r_CG_REF)
	: earth_RN(earth_a), earth_RM(earth_a),
	  q({1.0, 0, 0, 0}), P_rot(0), r_V_E_measured(0), x_trans(0), P_trans(0),
	  B_E_V({1.0,-1.192901206361467,0.387061157429015}, {0.048539987766887,0.097079975533774,0.048539987766887}, this->B_E_E),
	  omega_V_V_bias(0),
	  omega_V_V({1.0,-2.686157396548144,2.419655110966473,-0.730165345305723}, {4.165461390757476e-4,0.001249638417227,0.001249638417227,4.165461390757476e-4}, Vector<3>(0)),
	  a_g({1.0,-0.975177876180649}, {0.012411061909675,0.012411061909675}, 1.0), Phi_measured(0), Phi(0),
	  f_V({1.0,-2.135141627568843,1.562431414465148,-0.390088163176912}, {0.004650202964924,0.013950608894772,0.013950608894772,0.004650202964924}, -this->g_E),
	  n({1.0, -1.1929, 3.8706e-1}, {4.8540e-2, 9.7080e-2, 4.8540e-2}, Vector<3>({0, 0, 1})),
	  v_V_V(0),
	  r_V_E(0),
	  v_W_E({1.0,-0.998744151845968}, {6.279240770159843e-4,6.279240770159843e-4}, Vector<3>(0)),
	  T_VE({1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0})
{
	// Compute the IMU position vector (vehicle coordinates)
	this->r_CG_IMU = r_CG_REF - this->r_IMU_REF;

	// Precompute the earth reference basis for vehicle attitude estimation
	Vector<3> e1 = this->g_E / norm(this->g_E) + this->B_E_E / norm(this->B_E_E);
	e1 /= norm(e1);

	Vector<3> e2 = this->g_E.cross(this->B_E_E);
	e2 /= norm(e2);

	Vector<3> e3 = e1.cross(e2);

	this->E = Matrix<3, 3>({e1(0), e2(0), e3(0),
							e1(1), e2(1), e3(1),
							e1(2), e2(2), e3(2)});

	// Start initialization phase
	this->state = INS_CALIBRATING;
	this->init_step = -799;
}

/*
 * Perform the initial setup process for the Inertial Navigation System:
 * 	- Measure startup gyro bias (should be small)
 * 	- Get the reference latitude and longitude for the flat-earth equations
 * 	- Compute an initial guess for the rotational state vector
 * 	- Set the initial translational state vector
 * 	- Initialize the kalman filter covariance matrices
 */
void InertialNavigationSystem::initialize(float altitude_baro)
{
	// Compute gyro startup bias 1 second after system power up
	this->state = INS_CALIBRATING;
	if(sys.time < 1.0f)
		return;

	if(this->init_step <= 0)
	{
		// Collect 800 samples
		while(!this->gyro.omega.empty())
		{
			this->omega_V_V_bias += this->gyro.omega.pop() / 800.0f;
			this->init_step++;
		}

		return;
	}

	// Wait until an accurate GPS 3D position fix is available
	this->state = INS_WAIT_FOR_GPS;
	if(!this->gps.locked)
		return;

	// Perform setup for 1 second after GPS lock
	this->state = INS_INITIALIZING;
	this->init_step++;

	// Get reference location
	this->latitude_ref = Coordinate(47.8346);
	this->longitude_ref = Coordinate(16.5529);

	double c = 1.0 - (2.0 * this->earth_f - pow(this->earth_f, 2.0)) * pow(sin(this->latitude_ref.get_radians()), 2.0);
	this->earth_RN = this->earth_a / sqrt(c);
	this->earth_RM = this->earth_RN * (1.0 - (2.0 * this->earth_f - pow(this->earth_f, 2.0))) / c;

	// Filter acceleration
	while(!this->acc_mag.f.empty())
		this->f_V.update(this->acc_mag.f.pop());

	// Initialize the state vectors
	this->rotation_measure();

	float c_phi = cos(this->Phi_measured(0) / 2.0f);
	float s_phi = sin(this->Phi_measured(0) / 2.0f);
	float c_theta = cos(this->Phi_measured(1) / 2.0f);
	float s_theta = sin(this->Phi_measured(1) / 2.0f);
	float c_psi = cos(this->Phi_measured(2) / 2.0f);
	float s_psi = sin(this->Phi_measured(2) / 2.0f);

	this->q = Vector<4>({c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
						 s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
						 c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
						 c_phi * c_theta * s_psi - s_phi * s_theta * c_psi});

	this->rotation_output();

	this->translation_measure(altitude_baro);
	this->x_trans = Vector<6>({0, 0, 0, this->r_V_E_measured(0), this->r_V_E_measured(1), this->r_V_E_measured(2)});
	this->translation_output();

	// Initialize rotational and translational covariance matrices
	this->P_rot = Matrix<3, 3>({1e-2, 0,    0,
								0,    1e-2, 0,
								0,    0,    1e-2});

	this->P_trans = Matrix<6, 6>({1e-1, 0,    0,    0,    0,    0,
								  0,    1e-1, 0,    0,    0,    0,
								  0,    0,    1e-1, 0,    0,    0,
								  0,    0,    0,    1e-1, 0,    0,
								  0,    0,    0,    0,    1e-1, 0,
								  0,    0,    0,    0,    0,    1e-1});
}

/*
 * Measure the vehicle attitude from a computed gravity vector in vehicle coordinates
 * and the magnetic flux density vector in vehicle coordinates by comparing an earth base
 * with a vehicle base
 */
void InertialNavigationSystem::rotation_measure()
{
	// Compute the gravity vector in vehicle axes [m/s^2]
	Vector<3> g_V = this->omega_V_V.filtered.cross(this->v_V_V) - this->f_V.filtered;

	// Compute the vehicle reference basis for attitude estimation
	Vector<3> v1 = g_V / norm(g_V) + this->B_E_V.filtered / norm(this->B_E_V.filtered);
	v1 /= norm(v1);

	Vector<3> v2 = g_V.cross(this->B_E_V.filtered);
	v2 /= norm(v2);

	Vector<3> v3 = v1.cross(v2);

	Matrix<3, 3> V({v1(0), v2(0), v3(0),
					v1(1), v2(1), v3(1),
					v1(2), v2(2), v3(2)});

	// Compute the transformation from earth basis to vehicle basis
	Matrix<3, 3> T_VE_estimated = V * transpose(this->E);

	if(!T_VE_estimated.finite())
	{
		print("Estimated transformation matrix T_VE is not finite", MESSAGE_WARNING);
		return;
	}

	float phi = atan2(T_VE_estimated(1, 2), T_VE_estimated(2, 2));
	float theta = -asin(T_VE_estimated(0, 2));

	// Compute yaw from magnetic field only
	float B_x = cos(phi) * this->B_E_V.filtered(0) + sin(phi) * sin(theta) * this->B_E_V.filtered(1) + cos(phi) * sin(theta) * this->B_E_V.filtered(2);
	float B_y = cos(phi) * this->B_E_V.filtered(1) - sin(phi) * this->B_E_V.filtered(2);
	float psi = atan2(this->B_E_E(1), this->B_E_E(0)) - atan2(B_y, B_x);
	if(psi > PI)
		psi -= 2.0f * PI;

	// Compute acceleration gain (for complemantary filtering)
	float a_g_new = 1 - 10.0f * abs(norm(g_V) / this->g_E(2) - 1.0f);

	if(a_g_new < 0.0f)
		a_g_new = 0.0f;
	else if(a_g_new > 1.0f)
		a_g_new = 1.0f;

	this->a_g.update(a_g_new);

	// Blend estimated and measured yaw together, via acceleration gain
	float delta_psi = psi - this->Phi(2);

	if(delta_psi > PI)
	    delta_psi -= 2.0f * PI;
	else if(delta_psi < -PI)
	    delta_psi += 2.0f * PI;

	psi = this->Phi(2) + this->a_g.filtered * delta_psi;

	if(psi > PI)
	    psi -= 2.0f * PI;
	else if(psi < -PI)
	    psi += 2.0f * PI;

	this->Phi_measured = Vector<3>({phi, theta, psi});
}

/*
 * Perform the prediction step of the kalman fliter
 * for vehicle attitude and heading (rotation quaternion) estimation;
 * Runs at 800Hz
 */
void InertialNavigationSystem::rotation_kalman_predict()
{
	// Get the inputs
	float P = this->omega_V_V.raw(0);
	float Q = this->omega_V_V.raw(1);
	float R = this->omega_V_V.raw(2);

	// Get jacobian of state function
	Matrix<4, 4> F = Matrix<4, 4>({1,           -P / 1600.0f, -Q / 1600.0f, -R / 1600.0f,
	                               P / 1600.0f,  1,            R / 1600.0f, -Q / 1600.0f,
	                               Q / 1600.0f, -R / 1600.0f,  1,            P / 1600.0f,
	                               R / 1600.0f,  Q / 1600.0f, -P / 1600.0f,  1});

	// Get jacobian of reduced state function
	Matrix<3, 3> F_red = Matrix<3, 3>({1,           R / 800.0f, -Q / 800.0f,
	                                  -R / 800.0f,  1,           P / 800.0f,
                                       Q / 800.0f, -P / 800.0f,  1});

	// Perform prediction step and normalize the quaternion
	this->q = F * this->q;
	this->q /= norm(this->q);
	this->P_rot = F_red * this->P_rot * transpose(F_red) + 1.0f / 256.0e4f * this->Q_rot;

	// Refresh the outputs
	this->rotation_output();
}

/*
 * Update the rotational kalman fliter with computed attitude data
 * from measured gravity vector and magnetic flux density vector;
 * Runs at 50Hz
 */
void InertialNavigationSystem::rotation_kalman_update()
{
	// Convert measured euler angles to a rotation quaternion with matched sign to estimated quaternion
	float c_phi = cos(this->Phi_measured(0) / 2.0f);
	float s_phi = sin(this->Phi_measured(0) / 2.0f);
	float c_theta = cos(this->Phi_measured(1) / 2.0f);
	float s_theta = sin(this->Phi_measured(1) / 2.0f);
	float c_psi = cos(this->Phi_measured(2) / 2.0f);
	float s_psi = sin(this->Phi_measured(2) / 2.0f);


	Vector<4> q_measured = Vector<4>({c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
									  s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
									  c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
									  c_phi * c_theta * s_psi - s_phi * s_theta * c_psi});

	if(sgn(this->q(0)) != sgn(q_measured(0)))
		q_measured = -q_measured;

	// Compute the reduced state vector (error quaternion)
	Matrix<4, 3> J = Matrix<4, 3>({-this->q(1), -this->q(2), -this->q(3),
									this->q(0), -this->q(3),  this->q(2),
									this->q(3),  this->q(0), -this->q(1),
								   -this->q(2),  this->q(1),  this->q(0)});

	Vector<3> delta_q = transpose(J) * q_measured;

    // Compute the Kalman gain
	c_phi = cos(this->Phi(0) / 2.0f);
	s_phi = sin(this->Phi(0) / 2.0f);
	c_theta = cos(this->Phi(1) / 2.0f);
	s_theta = sin(this->Phi(1) / 2.0f);
	c_psi = cos(this->Phi(2) / 2.0f);
	s_psi = sin(this->Phi(2) / 2.0f);

	Vector<4> q_ = Vector<4>({c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
						      s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
						      c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
						      c_phi * c_theta * s_psi - s_phi * s_theta * c_psi});

	Vector<4> r_ = Vector<4>({c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
						      s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
						      c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
						      c_phi * c_theta * s_psi + s_phi * s_theta * c_psi});

    Matrix<4, 3> L = 0.5f * Matrix<4, 3>({-q_(1), -r_(2), -q_(3),
                                           q_(0), -r_(3), -q_(2),
                                           q_(3),  r_(0),  q_(1),
                                          -q_(2), -r_(1),  q_(0)});

    Matrix<3, 3> M = transpose(J) * L;

    Matrix<3, 3> K = this->P_rot * inverse(this->P_rot + M * this->R_rot * transpose(M));

	// Perform update step to the reduced state vector
	delta_q = K * delta_q;
	this->P_rot -= K * this->P_rot;

	// Update the full state vector and normalize it
	this->q += J * delta_q;
	this->q /= norm(this->q);

	// Refresh the outputs
	this->rotation_output();
}

/*
 * Update the outputs of the rotational kalman filter from the estimated rotation quaternion:
 * 	- Transformation matrix from earth to vehicle coordinates
 * 	- Vehicle attitude vector Phi [rad]
 */
void InertialNavigationSystem::rotation_output()
{
	this->T_VE(0, 0) = this->q(0) * this->q(0) + this->q(1) * this->q(1) - this->q(2) * this->q(2) - this->q(3) * this->q(3);
	this->T_VE(0, 1) = 2.0f * (this->q(1) * this->q(2) + this->q(0) * this->q(3));
	this->T_VE(0, 2) = 2.0f * (this->q(1) * this->q(3) - this->q(0) * this->q(2));
	this->T_VE(1, 0) = 2.0f * (this->q(1) * this->q(2) - this->q(0) * this->q(3));
	this->T_VE(1, 1) = this->q(0) * this->q(0) - this->q(1) * this->q(1) + this->q(2) * this->q(2) - this->q(3) * this->q(3);
	this->T_VE(1, 2) = 2.0f * (this->q(2) * this->q(3) + this->q(0) * this->q(1));
	this->T_VE(2, 0) = 2.0f * (this->q(1) * this->q(3) + this->q(0) * this->q(2));
	this->T_VE(2, 1) = 2.0f * (this->q(2) * this->q(3) - this->q(0) * this->q(1));
	this->T_VE(2, 2) = this->q(0) * this->q(0) - this->q(1) * this->q(1) - this->q(2) * this->q(2) + this->q(3) * this->q(3);

	this->Phi(0) = atan2(this->T_VE(1, 2), this->T_VE(2, 2));
	this->Phi(1) = -asin(this->T_VE(0, 2));
	this->Phi(2) = atan2(this->T_VE(0, 1), this->T_VE(0, 0));
}

/*
 * Compute the vehicle flat-earth position from the GPS measurement and
 * the altitude from the barometer measurement
 * @param altitude_baro			Barometric altitude measurement from ADC
 */
void InertialNavigationSystem::translation_measure(float altitude_baro)
{
	double delta_lat = this->gps.latitude.get_radians() - this->latitude_ref.get_radians();
	double delta_lon = this->gps.longitude.get_radians() - this->longitude_ref.get_radians();

	this->r_V_E_measured(0) = delta_lat / atan2(1, this->earth_RM);
	this->r_V_E_measured(1) = delta_lon / atan2(1, this->earth_RN * cos(this->latitude_ref.get_radians()));
	this->r_V_E_measured(2) = -altitude_baro;
}

/*
 * Perform the prediction step of the kalman fliter
 * for vehicle groundspeed, vertical speed, position and altitude estimation;
 * Runs at 400Hz
 */
void InertialNavigationSystem::translation_kalman_predict()
{
	// Get the inputs
	float P = this->omega_V_V.filtered(0);
	float Q = this->omega_V_V.filtered(1);
	float R = this->omega_V_V.filtered(2);

	// Get state jacobian of state function
	Matrix<6, 6> F = Matrix<6, 6>({1,		             R / 400.0f, 		  -Q / 400.0f, 	        0, 0, 0,
							      -R / 400.0f,	         1, 	               P / 400.0f, 	        0, 0, 0,
								   Q / 400.0f, 		    -P / 400.0f, 	       1, 	                0, 0, 0,
								   T_VE(0, 0) / 400.0f,  T_VE(1, 0) / 400.0f,  T_VE(2, 0) / 400.0f, 1, 0, 0,
								   T_VE(0, 1) / 400.0f,  T_VE(1, 1) / 400.0f,  T_VE(2, 1) / 400.0f, 0, 1, 0,
								   T_VE(0, 2) / 400.0f,  T_VE(1, 2) / 400.0f,  T_VE(2, 2) / 400.0f, 0, 0, 1});

	// Get noise jacobian of state function
	Matrix<6, 3> L = Matrix<6, 3>({1.0f / 400.0f, 0,             0,
								   0,             1.0f / 400.0f, 0,
								   0,             0,             1.0f / 400.0f,
								   0,             0,             0,
								   0,             0,             0,
								   0,             0,             0});

	// Get state equation input vector
	Vector<6> u = Vector<6>({(this->f_V.raw(0) + this->g_E(2) * this->T_VE(0, 2)) / 400.0f,
						     (this->f_V.raw(1) + this->g_E(2) * this->T_VE(1, 2)) / 400.0f,
						     (this->f_V.raw(2) + this->g_E(2) * this->T_VE(2, 2)) / 400.0f,
							 0,
							 0,
							 0});

	// Perform prediction step
	this->x_trans = F * this->x_trans + u;
	this->P_trans = F * this->P_trans * transpose(F) + L * this->Q_trans * transpose(L);

	// Refresh the outputs
	this->translation_output();
}

/*
 * Update the translational kalman fliter with computed position data
 * from gps and barometer measurements;
 * Runs at 50Hz
 */
void InertialNavigationSystem::translation_kalman_update()
{
	// Output jacobian of the output function of the translational kalman filter
	Matrix<3, 6> H = Matrix<3, 6>({0, 0, 0, 1, 0, 0,
								   0, 0, 0, 0, 1, 0,
								   0, 0, 0, 0, 0, 1});

	// Compute the kalman gain
	Matrix<6, 3> K = this->P_trans * transpose(H) * inverse(H * this->P_trans * transpose(H) + this->R_trans);

	// Perform the update step
	this->x_trans += K * (this->r_V_E_measured - this->r_V_E);
	this->P_trans -= K * H * this->P_trans;

	// Refresh the outputs
	this->translation_output();
}

/*
 * Update the outputs of the translational kalman filter:
 * 	- Vehicle ground speed and vertical speed in vehicle coordinates [m/s]
 * 	- Vehicle position and altitude in flat-earth coordinates [m]
 */
void InertialNavigationSystem::translation_output()
{
	this->v_V_V(0) = this->x_trans(0);
	this->v_V_V(1) = this->x_trans(1);
	this->v_V_V(2) = this->x_trans(2);

	this->r_V_E(0) = this->x_trans(3);
	this->r_V_E(1) = this->x_trans(4);
	this->r_V_E(2) = this->x_trans(5);
}

/*
 * Update the vehicle state:
 * Shift the measurements to vehicle cg;
 * Perform an attitude and heading state estimation;
 * Kalman-filter the rotational vehicle states (rotation quaternion);
 * Kalman-filter the translational vehicle states (groundspeed, vertical speed, flat-earth position, altitude);
 * Estimate the wind speed
 * @param altitude			Barometric MSL altitude measurement (QNH) [m]
 * @param motor_current 	Propulsion motor current draw for magnetometer compensation [A]
 * @param v_rel				Vehicle velocity realtive to air (from ADC)
 */
void InertialNavigationSystem::update(float motor_current, float altitude_baro, Vector<3> v_rel)
{
	// Update gps position
	this->gps.update();

	// Update accelerometer and magnetometer with estimated motor current
	this->acc_mag.update(motor_current);

	// Take the mean acceleration and update gyro with it
	Vector<3> f_gyro(0);

	if(!this->acc_mag.f.empty())
	{
		for(int i = 0; i < this->acc_mag.f.size(); i++)
			f_gyro += this->acc_mag.f[i];

		f_gyro /= this->acc_mag.f.size();
	}

	this->gyro.update(f_gyro);

	// Update magnetic flux density vector (in vehicle coordinates)
	this->B_E_V.update(this->acc_mag.B);

	// If the setup process is not yet complete, skip state estimation
	if(this->init_step <= 50)
	{
		this->initialize(altitude_baro);
		return;
	}

	// Perform 800Hz and 400Hz state predictions interleaved
	for(int i = 0; i < 32; i++)
	{
		if(!this->gyro.omega.empty())
		{
			// Update vehicle angular rate vector (in vehicle coordinates)
			this->omega_V_V.update(this->gyro.omega.pop() - this->omega_V_V_bias);

			// Perform prediction step of extended kalman filter for vehicle attitude and heading
			this->rotation_kalman_predict();
		}

		if((!this->acc_mag.f.empty()) && (i % 2))
		{
			// Update the specific force vector (in vehicle coordinates, at CG)
			this->f_V.update(this->acc_mag.f.pop() + this->omega_V_V.filtered.cross(this->omega_V_V.filtered.cross(this->r_CG_IMU)));

			// Perform prediction step of extended kalman filter for vehicle speed and position
			this->translation_kalman_predict();
		}
	}

	// Update attitude and heading with gravity and magnetic field information
	this->rotation_measure();
	this->rotation_kalman_update();

	// Update position and altitude with GPS and barometer measurements at 50Hz
	this->translation_measure(altitude_baro);
	this->translation_kalman_update();

	// Estimate wind components in earth fixed axes
	this->v_W_E.update(transpose(this->T_VE) * (this->v_V_V - v_rel));

	// Update the load factor
	this->n.update(-this->f_V.filtered / 9.81f);

	this->state = INS_RUNNING;
}

/**
 * Convert an ASCII string with fixed length (no '\0' delimiter) to a double number
 * @param ascii		Pointer to the string that should be converted
 * @param length	Length of the number in characters which has to be converted
 * @return			Converted value as double
 */
double stringToDouble(const char* ascii, int length)
{
	double number = 0;
	int decimal = length;		// Position of decimal point

	// Search for the decimal point
	for(int i = 0; i < length; i++)
	{
		if(ascii[i] == '.')
		{
			decimal = i;
			break;
		}
	}

	// Convert chars before the decimal point
	for(int i = 0; i < decimal; i++)
	{
		if((ascii[i] >= '0') && (ascii[i] <= '9'))
			number += (double)(ascii[i] - '0') * pow(10.0, decimal - i - 1);
		else
			return 0;
	}

	// Convert chars after the decimal point
	for(int i = decimal + 1; i < length; i++)
	{
		if((ascii[i] >= '0') && (ascii[i] <= '9'))
			number += (double)(ascii[i] - '0') * pow(10.0, decimal-i);
		else
			return 0;
	}

	return number;
}

/**
 * Convert an ASCII string with unknown length to a double number
 * @param ascii		Pointer to the string that should be converted
 * @return			Converted value as double
 */
double stringToDouble(const char* ascii)
{
	int length = strlen(ascii);

	return stringToDouble(ascii, length);
}

/**
 * Interrupt handler for incoming NMEA GPS sentences
 */
extern "C" void NO_OPTIMIZATION UART4_IRQHandler(void)
{
	// Check for Frame and Noise Error, discard data if error
	if(UART4->ISR & UART4_ERROR_MASK)
	{
		Venus638::receiveErrors++;

		// Raise a warning, if a repeated error occured
		if(Venus638::receiveErrors > GPS_MAX_RECEIVE_ERRORS)
			print("GPS repeated receive error. UART4->ISR= ", UART4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);

		// Clear flags and flush receive register
		UART4->ICR |= USART_ICR_FECF | USART_ICR_NCF;
		UART4->RQR |= USART_RQR_RXFRQ;
		return;
	}
	else
	{
		Venus638::receiveErrors = 0;
	}

	// Check if $ character received
	if(UART4->ISR & USART_ISR_CMF)
	{
		UART4->ICR |= USART_ICR_CMCF;

		// Set old buffer as ready for processing
		Venus638::NMEA_ready[Venus638::rx_buffer] = 1;

		// Change rx buffer
		if(Venus638::rx_buffer < 6)
			Venus638::rx_buffer++;
		else
			Venus638::rx_buffer = 0;

		Venus638::rx_position = 0;
	}

	// Get the received Character
	Venus638::NMEA_sentence[Venus638::rx_buffer][Venus638::rx_position] = (char)UART4->RDR;

	if(Venus638::rx_position < 79)
		Venus638::rx_position++;

	// Clear the interrupt flag and wait until it is definitely cleared
	NVIC_ClearPendingIRQ(UART4_IRQn);
	__DSB();
	__ISB();
}
