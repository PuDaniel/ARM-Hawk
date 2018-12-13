/*
 * AirDataComputer.cpp
 *
 *  Created on: Jan 22, 2018
 *      Author: Daniel
 */

#include "AirDataComputer.hpp"

// Initialize SensorUnitBus class
char SensorUnitBus::initialized = 0;

/**
 * Initialize the I2C bus for communication
 */
NO_OPTIMIZATION SensorUnitBus::SensorUnitBus()
{
	// Check if Hardware is already initialized
	if(this->initialized)
		return;

	// Initialize PB8 & PB9 as alternate function 1 (I2C4) with open drain outputs
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
	GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOB->AFR[1] |= (1 << 0) | (1 << 4);

	// Initialize PB4, PB5 & PB6 as output (Enable PINs for SDA line on SM9541)
	GPIOB->MODER &= ~GPIO_MODER_MODER4;
	GPIOB->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4;

	// Initialize PD2 (EOC) as input
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	// Initialize I2C4
	RCC->APB1ENR |= RCC_APB1ENR_I2C4EN;
	RCC->DCKCFGR2 |= RCC_DCKCFGR2_I2C4SEL_1;	// Select HSI clock as source (16MHz)
	I2C4->CR1 &= ~I2C_CR1_PE;					// Disable I2C4
	I2C4->TIMINGR |= (0x03 << 28) | (0x04 << 20) | (0x02 << 16) | (0x0F << 8) | (0x13 << 0);	// 100kHz clock
	I2C4->TIMEOUTR |= I2C_TIMEOUTR_TIMOUTEN | (7 << 0);		// Enable timeout; timeout = 62.5ns*2048*8 = 1.024ms
	I2C4->CR1 |= I2C_CR1_PE;					// Enable I2C4

	this->initialized = 1;
}

/**
 * Read the specified number of bytes from the specified device.
 * @param address		Address of slave device to read from
 * @param buffer		Pointer to byte buffer the data should be read into
 * @param nBytes		Number of bytes to read from the slave
 */
void NO_OPTIMIZATION SensorUnitBus::read(uint8_t address, uint8_t* buffer, uint8_t nBytes)
{
	// Reset config register 2; set slave address, read, transfer length, autoend mode; start communication
	I2C4->CR2 = 0;
	I2C4->CR2 |= I2C_CR2_AUTOEND | (nBytes << 16) | I2C_CR2_RD_WRN | ((address & 0x7F) << 1);
	I2C4->CR2 |= I2C_CR2_START;

	// Receive all bytes
	for(int i = 0; i < nBytes; i++)
	{
		// Wait until one byte received or error occured
		while(!(I2C4->ISR & (I2C_ISR_RXNE | I2C4_ERROR_MASK)));

		if(I2C4->ISR & I2C_ISR_RXNE)
		{
			// Read one byte
			buffer[i] = I2C4->RXDR;
		}
		else
		{
			// Error occured => Terminate
			print("I2C4 read error with ISR= ", I2C4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			return;
		}
	}

	// Clear stop flag and all raised errors
	I2C4->ICR |= I2C_ICR_STOPCF | I2C4_ERROR_CLEAR;
}

/**
 * Write the specified number of bytes to the specified device.
 * @param address		Address of slave device to write to
 * @param buffer		Pointer to byte buffer the data should be written from
 * @param nBytes		Number of bytes to write to the slave
 */
void NO_OPTIMIZATION SensorUnitBus::write(uint8_t address, uint8_t* buffer, uint8_t nBytes)
{
	// Reset config. register 2; set slave address, write, transfer length, autoend mode; start communication
	I2C4->CR2 = 0;
	I2C4->CR2 |= I2C_CR2_AUTOEND | (nBytes << 16) | ((address & 0x7F) << 1);
	I2C4->CR2 |= I2C_CR2_START;

	// Transmit all bytes
	for(int i = 0; i < nBytes; i++)
	{
		// Wait until ready to transmit or error occured
		while(!(I2C4->ISR & (I2C_ISR_TXIS | I2C4_ERROR_MASK)));

		if(I2C4->ISR & I2C_ISR_TXIS)
		{
			// Write one byte
			I2C4->TXDR = buffer[i];
		}
		else
		{
			// Error occured => Terminate
			print("I2C4 write error with ISR= ", I2C4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			return;
		}
	}

	// Wait until transfer complete
	while(!(I2C4->ISR & (I2C_ISR_TXE | I2C4_ERROR_MASK)));

	if(!(I2C4->ISR & I2C_ISR_TXE))
	{
		// Error occured => Terminate
		print("I2C4 write finish error with ISR= ", I2C4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Clear stop flag and all raised errors
	I2C4->ICR |= I2C_ICR_STOPCF | I2C4_ERROR_CLEAR;
}

/**
 * Read the specified number of bytes from the specified device after providing the register.
 * First sends a write command with 1byte containing the register address,
 * then sends a repeated start condition with a following read command for n bytes.
 * @param address		Address of slave device
 * @param reg			Register address to read from
 * @param buffer		Pointer to byte buffer the data should be read into
 * @param nBytes		Number of bytes to read from the slave
 */
void NO_OPTIMIZATION SensorUnitBus::readRegister(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes)
{
	// Reset config. register 2; set slave address, write, transfer length = 1; start communication
	I2C4->CR2 = 0;
	I2C4->CR2 |= (1 << 16) | ((address & 0x7F) << 1);
	I2C4->CR2 |= I2C_CR2_START;

	// Wait until ready to transmit or error occured
	while(!(I2C4->ISR & (I2C_ISR_TXIS | I2C4_ERROR_MASK)));

	if(I2C4->ISR & I2C_ISR_TXIS)
	{
		// Write one byte
		I2C4->TXDR = reg;
	}
	else
	{
		// Error occured => Terminate
		print("I2C4 readRegister TX error with ISR= ", I2C4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Wait until transfer complete
	while(!(I2C4->ISR & (I2C_ISR_TXE | I2C4_ERROR_MASK)));

	if(!(I2C4->ISR & I2C_ISR_TXE))
	{
		// Error occured => Terminate
		print("I2C4 readRegister TX finish error with ISR= ", I2C4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
		return;
	}

	// Clear stop flag and all raised errors
	I2C4->ICR |= I2C_ICR_STOPCF | I2C4_ERROR_CLEAR;

	// Reset config. register 2; set slave address, read, transfer length, autoend mode; restart communication
	I2C4->CR2 = 0;
	I2C4->CR2 |= I2C_CR2_AUTOEND | (nBytes << 16) | I2C_CR2_RD_WRN | ((address & 0x7F) << 1);
	I2C4->CR2 |= I2C_CR2_START;

	// Receive all bytes
	for(int i = 0; i < nBytes; i++)
	{
		// Wait until one byte received or error occured
		while(!(I2C4->ISR & (I2C_ISR_RXNE | I2C4_ERROR_MASK)));

		if(I2C4->ISR & I2C_ISR_RXNE)
		{
			// Read one byte
			buffer[i] = I2C4->RXDR;
		}
		else
		{
			// Error occured => Terminate
			print("I2C4 readRegister RX error with ISR= ", I2C4->ISR, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			print("^^^^ tried to interface address ", address, MESSAGE_OPTION_HEX, MESSAGE_WARNING);
			return;
		}
	}

	// Clear stop flag and all raised errors
	I2C4->ICR |= I2C_ICR_STOPCF | I2C4_ERROR_CLEAR;
}

/**
 * Manages the SDA lines to the three SM9541 sensors with the same I2C address
 * @param sensorNumber		Index of the sensor to be enabled (0 = disable all)
 */
void NO_OPTIMIZATION SensorUnitBus::enableSensor(char sensorNumber)
{
	if(sensorNumber == DISABLE_ALL)
		GPIOB->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_6;
	else if(sensorNumber == SENSOR_1)
		GPIOB->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_6;
	else if(sensorNumber == SENSOR_2)
		GPIOB->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BR_6;
	else if(sensorNumber == SENSOR_3)
		GPIOB->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BS_6;
}

/**
 * Poll the EOC pin of the barometer.
 * @return			1 for conversion ended; 0 for conversion in progress
 */
char NO_OPTIMIZATION SensorUnitBus::endOfConversion()
{
	if(GPIOD->IDR & GPIO_IDR_IDR_2)
		return 1;
	else
		return 0;
}

/**
 * Initialize one SM9541 differential pressure sensor
 * @param sensorNumber		Index of sensor on the SDA line
 */
SM9541::SM9541(char sensorNumber, float scale)
{
	this->scale = scale;

	this->pressure = 0;
	this->temperature = 0;
	this->status = 0;

	this->sensorNumber = sensorNumber;
}

/**
 * Update pressure and temperature of the sensor
 */
void SM9541::update()
{
// Disable update method if hardware is temporarly disabled for debugging
#ifndef ADC_DISABLED
	// Read SMI9541
	this->bus.enableSensor(this->sensorNumber);

	uint8_t buffer[4];
	this->bus.read(0x28, buffer, 4);

	// Extract status bits
	this->status = (buffer[0] >> 6) & 0x03;

	if(this->status == 0b00)
	{
		// Data OK
		// Extract pressure and convert to Pa
		this->pressure = ((buffer[0] & 0x3F) << 8) | buffer[1];
		this->pressure = (this->pressure - 1638.0f) * (2.0f * this->scale) / 13107.0f - this->scale;

		// Extract temperature and convert to Celsius
		this->temperature = (buffer[2] << 3) | ((buffer[3] >> 5) & 0x07);
		this->temperature = this->temperature * 200.0f / 2048.0f - 50.0f;
	}
	else if(this->status == 0b10)
	{
		// Data already read => do nothing
		return;
	}
	else
	{
		// Pressure sensor inop => Terminate
		print("SM9541 pressure sensor failed! Sensor #: ", this->sensorNumber, MESSAGE_WARNING);
		return;
	}
#else
	this->pressure = 0;
	this->temperature = 15;
#endif
}

/**
 * Initialize the BMP085 static pressure sensor
 */
BMP085::BMP085()
{
	this->pressure = 0;
	this->temperature = 0;
	this->cycle = 0;

// Disable initialization if hardware is temporarly disabled for debugging
#ifndef ADC_DISABLED
	// Read calibration data from device EEPROM
	uint8_t buffer[22];
	this->bus.readRegister(0x77, 0xAA, buffer, 22);

	this->AC1 = ((int16_t)buffer[0] << 8) | (int16_t)buffer[1];
	this->AC2 = ((int16_t)buffer[2] << 8) | (int16_t)buffer[3];
	this->AC3 = ((int16_t)buffer[4] << 8) | (int16_t)buffer[5];
	this->AC4 = ((uint16_t)buffer[6] << 8) | (uint16_t)buffer[7];
	this->AC5 = ((uint16_t)buffer[8] << 8) | (uint16_t)buffer[9];
	this->AC6 = ((uint16_t)buffer[10] << 8) | (uint16_t)buffer[11];
	this->B1 = ((int16_t)buffer[12] << 8) | (int16_t)buffer[13];
	this->B2 = ((int16_t)buffer[14] << 8) | (int16_t)buffer[15];
	this->MC = ((int16_t)buffer[18] << 8) | (int16_t)buffer[19];
	this->MD = ((int16_t)buffer[20] << 8) | (int16_t)buffer[21];

	this->B5 = 0;
#else
	this->AC1 = 0;
	this->AC2 = 0;
	this->AC3 = 0;
	this->AC4 = 0;
	this->AC5 = 0;
	this->AC6 = 0;
	this->B1 = 0;
	this->B2 = 0;
	this->MC = 0;
	this->MD = 0;
	this->B5 = 0;
#endif
}

/**
 * Update pressure and temperature if new values are available.
 * Start a new update cycle if not.
 */
void BMP085::update()
{
// Disable update method if hardware is temporarly disabled for debugging
#ifndef ADC_DISABLED
	if(this->cycle < 10)
	{
		// Update pressure data
		if(this->bus.endOfConversion())
		{
			// Read pressure data
			uint8_t buffer[3];
			this->bus.readRegister(0x77, 0xF6, buffer, 3);

			int32_t UP = ((((int32_t)buffer[0] << 16) | ((int32_t)buffer[1] << 8) | (int32_t)buffer[2]) >> 5);

			// Calculate compensated pressure value
			int32_t B6 = this->B5 - 4000;
			int32_t X1 = (this->B2 * ((B6 * B6) / 4096)) / 2048;
			int32_t X2 = (this->AC2 * B6) / 2048;
			int32_t X3 = X1 + X2;
			int32_t B3 = (((this->AC1 * 4 + X3) << 3) + 2) / 4;
			X1 = (this->AC3 * B6) / 8192;
			X2 = (this->B1 * ((B6 * B6) / 4096)) / 65536;
			X3 = ((X1 + X2) + 2) / 4;
			uint32_t B4 = (this->AC4 * (X3 + 32768)) / 32768;
			uint32_t B7 = (UP - B3) * (50000 >> 3);
			int32_t pressure = 0;
			if(B7 < 0x80000000)
				pressure = (B7 * 2) / B4;
			else
				pressure = (B7 / B4) * 2;
			X1 = (pressure / 256) * (pressure / 256);
			X1 = (X1 * 3038) / 65536;
			X2 = (-7357 * pressure) / 65536;
			pressure += (X1 + X2 + 3791) / 16;

			// Convert pressure data to float and store
			this->pressure = pressure;

			this->cycle ++;
		}
		else
		{
			// Start a new pressure conversion (ultra high resolution mode)
			uint8_t buffer[2] = {0xF4, 0xF4};
			this->bus.write(0x77, buffer, 2);
		}
	}
	else
	{
		// Update temperature data
		if(this->bus.endOfConversion())
		{
			// Read temperature data
			uint8_t buffer[2];
			this->bus.readRegister(0x77, 0xF6, buffer, 2);

			int32_t UT = ((int32_t)buffer[0] << 8) | (int32_t)buffer[1];

			// Calculate compensated temperature value
			int32_t X1 = ((UT - (int32_t)this->AC6) * (int32_t)this->AC5) / 32768;
			int32_t X2 = ((int32_t)this->MC * 2048) / (X1 + this->MD);
			this->B5 = X1 + X2;

			int32_t temperature = (this->B5 + 8) / 16;

			// Convert temperature data to float and store
			this->temperature = temperature / 10.0;

			this->cycle = 0;
		}
		else
		{
			// Start a new temperature conversion
			uint8_t buffer[2] = {0xF4, 0x2E};
			this->bus.write(0x77, buffer, 2);
		}
	}
#else
	this->pressure = 101325;
	this->temperature = 15;
#endif
}

/**
 * Constructor for the AirDataComputer object
 * @ param r_probe		Vector pointing from cg to tip of probe location
 */
AirDataComputer::AirDataComputer(float QNH, Vector<3> r_CG_REF)
	: diff_pressure_12(3, 980.638), diff_pressure_34(2, 980.638), diff_pressure_0S(1, 1961.276),
	  T_stat({1.0, -9.974898788670976e-1}, {1.255060566451240e-3, 1.255060566451240e-3}, 20.0),
	  p_stat({1.0, -1.889019457901979, 8.930517880857102e-1}, {1.008082545932828e-3, 2.016165091865655e-3, 1.008082545932828e-3}, 101325.0),
	  alpha({1.0, -1.276848411297126, 4.342575131441521e-1}, {3.935227546175648e-2, 7.870455092351296e-2, 3.935227546175648e-2}),
	  beta({1.0, -1.276848411297126, 4.342575131441521e-1}, {3.935227546175648e-2, 7.870455092351296e-2, 3.935227546175648e-2}),
	  V_T({1.0, -9.042035937390208e-1}, {4.789820313048960e-2, 4.789820313048960e-2})
{
	this->QNH = QNH;
	this->rho = 0;
	this->altitude = 0;

	// Compute the ADC (probe tip location) position vector (vehicle coordinates)
	this->r_CG_ADC = r_CG_REF - this->r_ADC_REF;

// Set Master Caution if hardware is temporarly disabled for debugging
#ifdef ADC_DISABLED
	print("ADC disabled for debugging", MESSAGE_CAUTION);
#endif
}

/**
 * Update the air data by reading all sensors and doing all necessary computations
 * @param omega_v		Vehicle angular rate given in vehicle coordinates
 */
void AirDataComputer::update(const Vector<3>& omega_v)
{
	// Update all sensors
	this->diff_pressure_12.update();
	this->diff_pressure_34.update();
	this->diff_pressure_0S.update();
	this->barometer.update();

	// Compute static temperature by taking the average of all 4 sensors
	this->T_stat.update((this->diff_pressure_12.temperature + this->diff_pressure_34.temperature +
			this->diff_pressure_0S.temperature + this->barometer.temperature) / 4.0f * this->C_temp);

	// Compute static pressure (calibrated with current airspeed; delta_p = 0.579 * U^2 - 0.819 * U)
	this->p_stat.update(this->barometer.pressure + (0.579f * this->v_rel(0) - 0.819f) * this->v_rel(0));

	// Compute air density with air gas constant R = 287J/(kg*K)
	this->rho = this->p_stat.filtered / (287.0f * (this->T_stat.filtered + 273.15f));

	// Compute airplane altitude over MSL (current QNH required)
	this->altitude = 29588.8f * (1 - pow(this->p_stat.filtered / this->QNH, 0.285));

	// Get the three differential pressure readings from probe tip
	float p_12 = this->diff_pressure_12.pressure;
	float p_34 = this->diff_pressure_34.pressure;
	float p_0S = this->diff_pressure_0S.pressure;

	// Compute alpha, beta and q_T via interpolation in a lookup table
	float alpha_new = 0;
	float beta_new = 0;
	float V_T_new = 0;

	if(p_0S > 0.0f)
	{
		// Rescale pressure values for array indexing
		float p_12_rel = abs(p_12 / ADC_data::pressure_step);
		float p_34_rel = abs(p_34 / ADC_data::pressure_step);
		float p_0S_rel = p_0S / ADC_data::pressure_step;

		// Compute alpha
		arm_bilinear_interp_instance_f32 alpha_data_low;
		alpha_data_low.numCols = ADC_data::p_34_length;
		alpha_data_low.numRows = ADC_data::p_12_length;
		alpha_data_low.pData = const_cast<float*>(ADC_data::alpha[(int)floor(p_0S_rel)]);
		float alpha_low = arm_bilinear_interp_f32(&alpha_data_low, p_34_rel, p_12_rel);

		arm_bilinear_interp_instance_f32 alpha_data_high;
		alpha_data_high.numCols = ADC_data::p_34_length;
		alpha_data_high.numRows = ADC_data::p_12_length;
		alpha_data_high.pData = const_cast<float*>(ADC_data::alpha[(int)ceil(p_0S_rel)]);
		float alpha_high = arm_bilinear_interp_f32(&alpha_data_high, p_34_rel, p_12_rel);

		alpha_new = alpha_low + (p_0S_rel - floor(p_0S_rel)) * (alpha_high - alpha_low);
		alpha_new = copysign(alpha_new, p_12) * PI / 180.0f;

		// Compute beta
		arm_bilinear_interp_instance_f32 beta_data_low;
		beta_data_low.numCols = ADC_data::p_34_length;
		beta_data_low.numRows = ADC_data::p_12_length;
		beta_data_low.pData = const_cast<float*>(ADC_data::beta[(int)floor(p_0S_rel)]);
		float beta_low = arm_bilinear_interp_f32(&beta_data_low, p_34_rel, p_12_rel);

		arm_bilinear_interp_instance_f32 beta_data_high;
		beta_data_high.numCols = ADC_data::p_34_length;
		beta_data_high.numRows = ADC_data::p_12_length;
		beta_data_high.pData = const_cast<float*>(ADC_data::beta[(int)ceil(p_0S_rel)]);
		float beta_high = arm_bilinear_interp_f32(&beta_data_high, p_34_rel, p_12_rel);

		beta_new = beta_low + (p_0S_rel - floor(p_0S_rel)) * (beta_high - beta_low);
		beta_new = copysign(beta_new, p_34) * PI / 180.0f;

		// Compute q_T
		arm_bilinear_interp_instance_f32 q_T_data_low;
		q_T_data_low.numCols = ADC_data::p_34_length;
		q_T_data_low.numRows = ADC_data::p_12_length;
		q_T_data_low.pData = const_cast<float*>(ADC_data::q_T[(int)floor(p_0S_rel)]);
		float q_T_low = arm_bilinear_interp_f32(&q_T_data_low, p_34_rel, p_12_rel);

		arm_bilinear_interp_instance_f32 q_T_data_high;
		q_T_data_high.numCols = ADC_data::p_34_length;
		q_T_data_high.numRows = ADC_data::p_12_length;
		q_T_data_high.pData = const_cast<float*>(ADC_data::q_T[(int)ceil(p_0S_rel)]);
		float q_T_high = arm_bilinear_interp_f32(&q_T_data_high, p_34_rel, p_12_rel);

		float q_T = q_T_low + (p_0S_rel - floor(p_0S_rel)) * (q_T_high - q_T_low);
		V_T_new = sqrt(q_T * 2.0f / this->rho);
	}

	// Compensate vehicle angular rates if the precision is big enough
	if(V_T_new > 10.0f)
	{
		Vector<3> v_rel_ADC({V_T_new * cos(alpha_new) * cos(beta_new),
							 V_T_new * sin(beta_new),
							 V_T_new * sin(alpha_new) * cos(beta_new)});

		Vector<3> v_rel_CG = v_rel_ADC + omega_v.cross(this->r_CG_ADC);

		V_T_new = norm(v_rel_CG);
		alpha_new = atan2(v_rel_CG(2), v_rel_CG(0));
		beta_new = asin(v_rel_CG(1) / V_T_new);
	}

	// Update and lowpass-filter the relative air vector properties
	this->alpha.update(alpha_new);
	this->beta.update(beta_new);
	this->V_T.update(V_T_new);

	// Compute the relative air velocity vector
	this->v_rel(0) = this->V_T.filtered * cos(this->alpha.filtered) * cos(this->beta.filtered);
	this->v_rel(1) = this->V_T.filtered * sin(this->beta.filtered);
	this->v_rel(2) = this->V_T.filtered * sin(this->alpha.filtered) * cos(this->beta.filtered);
}
