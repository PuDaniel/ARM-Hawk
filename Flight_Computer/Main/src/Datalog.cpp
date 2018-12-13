/*
 * Datalog.cpp
 *
 *  Created on: 14. Apr. 2017
 *      Author: Daniel
 */

#include "Datalog.hpp"

/**
 * Constructor of S25FL512 (external Flash) class. Init all peripherals for Quad-SPI.
 */
NO_OPTIMIZATION S25FL512::S25FL512()
{
	this->state = S25FL512_WRITING;

	// Init GPIOB/D/E/F for Quad-SPI (PB10 = NCS; PD11 = IO0; PD12 = IO1; PD13 = IO3; PE2 = IO2; PF10 = CLK)

	// Enable peripheral clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN;

	// Enable alternate function modes
	GPIOB->MODER |= GPIO_MODER_MODER10_1;
	GPIOD->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1;
	GPIOE->MODER |= GPIO_MODER_MODER2_1;
	GPIOF->MODER |= GPIO_MODER_MODER10_1;

	// Set all pins to AF9
	GPIOB->AFR[1] |= (9 << (10-8)*4);
	GPIOD->AFR[1] |= (9 << (11-8)*4) | (9 << (12-8)*4) | (9 << (13-8)*4);
	GPIOE->AFR[0] |= (9 << 2*4);
	GPIOF->AFR[1] |= (9 << (10-8)*4);

	// Enable very high speed on all pins
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13;
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;

	// Init Quad-SPI
	RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;					// Enable peripheral clock
	QUADSPI->CR |= (3 << 8) | QUADSPI_CR_EN;			// Set FIFO threshold to 4, Enable Quad-SPI
	QUADSPI->DCR |= (this->fsize << 16);				// Set flash size (size = 2 ^ (fsize + 1))

	// Enable QUAD mode in Configuration Register 1
	this->writeEnable();
	this->setClockspeed(REGISTER_ACCESS_CLOCKSPEED);

	// Configure to send 2 bytes: Indirect write; Data on 1 line; Instruction on 1 line; WRR command
	QUADSPI->FCR |= QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;		// Clear all flags
	QUADSPI->DLR = 1;
	QUADSPI->CCR = QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | S25FL512_CMD::WRR;
	QUADSPI->DR = ((S25FL512_CMD::CR1_LC1 | S25FL512_CMD::CR1_QUAD) << 8) | 0;		// SR1: 0; CR1: Latency code 10 & Enable Quad mode

	// Finish transfer and catch errors
	if(!this->finishTransfer())
	{
		// Error occured => Terminate
		print("QUADSPI error initiating CR1 of ext. flash", MESSAGE_CAUTION);
		return;
	}

	// Wait while Write in Progress is set
	int timeout = 0;

	while(1)
	{
		// Get content of status register
		uint8_t status = this->readStatusRegister();

		if(!(status & S25FL512_CMD::SR1_WIP))
		{
			// Write in Progress cleared => Finish without error
			break;
		}

		timeout++;

		if(timeout >= REGISTER_ACCESS_CLOCKSPEED * 125)
		{
			// Timeout occured => Raise error
			print("QUADSPI timeout initiating CR1 of ext. flash", MESSAGE_CAUTION);
			return;
		}
	}

	// Confirm ext. flash working by reading Identification Register
	this->setClockspeed(REGISTER_ACCESS_CLOCKSPEED);

	// Configure to receive 2 bytes: Indirect read; Data on 1 line; 24bit Address on 1 line; Instruction on 1 line; REMS command
	QUADSPI->FCR |= QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;		// Clear all flags
	QUADSPI->DLR = 1;
	QUADSPI->CCR = QUADSPI_CCR_FMODE_0 | QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADSIZE_1 | QUADSPI_CCR_IMODE_0 | S25FL512_CMD::REMS;
	QUADSPI->AR = 0;

	// Finish transfer and catch errors
	if(!this->finishTransfer())
	{
		// Error occured => Terminate
		print("QUADSPI error accessing REMS register of ext. flash", MESSAGE_CAUTION);
		return;
	}

	// Check if the ID is correct
	uint16_t ID = QUADSPI->DR;

	if(ID != 0x1901)
	{
		// ID mismatch => Raise error
		print("QUADSPI Device ID error of ext. flash", MESSAGE_CAUTION);
		return;
	}

	this->state = S25FL512_IDLE;
}

/**
 * Set the clockspeed of the Quad-SPI.
 * @param clockspeed	Desired speed of the Quad-SPI clock in MHz
 * @return 				Actual clockspeed of the Quad-SPI
 */
float NO_OPTIMIZATION S25FL512::setClockspeed(uint32_t clockspeed)
{
	// Check if requested clockspeed is too high
	if(clockspeed > DATALOG_MAX_CLOCKSPEED)
		clockspeed = DATALOG_MAX_CLOCKSPEED;

	// Compute prescaler and round up to ensure requested clockspeed is not exceeded
	uint8_t prescaler = 216 / clockspeed;
	if(clockspeed * prescaler < 216)
		prescaler++;

	QUADSPI->CR &= ~QUADSPI_CR_PRESCALER;
	QUADSPI->CR |= ((prescaler - 1) << 24);

	return (216.0 / prescaler);
}

/**
 * Finish an ongoing transfer on the QUADSPI bus and return the outcome.
 * @return	Successfull (1) or failed (0) transaction.
 */
char NO_OPTIMIZATION S25FL512::finishTransfer()
{
	// Wait until transfer complete or error
	while(!(QUADSPI->SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF)));

	if(QUADSPI->SR & QUADSPI_SR_TEF)
	{
		// Error occured => return abnormally
		return 0;
	}

	// Everything fine => return normally
	return 1;
}

/**
 * Read the Status Register 1 (SR1) and return it's content.
 * @return	Content of the Status Register 1 (SR1)
 */
uint8_t NO_OPTIMIZATION S25FL512::readStatusRegister()
{
	this->setClockspeed(REGISTER_ACCESS_CLOCKSPEED);

	// Configure for reading 1 byte: Indirect read; Data on 1 line; Instruction on 1 line; RDSR1 command
	QUADSPI->FCR |= QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;		// Clear all flags
	QUADSPI->DLR = 0;
	QUADSPI->CCR = QUADSPI_CCR_FMODE_0 | QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | S25FL512_CMD::RDSR1;

	// Finish transfer and catch errors
	if(!this->finishTransfer())
	{
		// Error occured => Terminate
		print("QUADSPI error reading SR1 from ext. flash", MESSAGE_CAUTION);
		return 0;
	}

	// Read and return the received byte
	uint8_t response = QUADSPI->DR;

	return response;
}

/**
 * Set the Write Enable Latch to allow register write, page program and erease commands.
 */
void NO_OPTIMIZATION S25FL512::writeEnable()
{
	this->setClockspeed(REGISTER_ACCESS_CLOCKSPEED);

	// Indirect write; Instruction on 1 line; WREN command
	QUADSPI->FCR |= QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;		// Clear all flags
	QUADSPI->CCR = QUADSPI_CCR_IMODE_0 | S25FL512_CMD::WREN;

	// Finish transfer and catch errors
	if(!this->finishTransfer())
	{
		// Error occured => Terminate
		print("QUADSPI error sending WREN command to ext. flash", MESSAGE_CAUTION);
		return;
	}

	// Wait until Write Enable Latch set
	int timeout = 0;

	while(1)
	{
		// Get content of status register
		uint8_t status = this->readStatusRegister();

		if(status & S25FL512_CMD::SR1_WEL)
		{
			// Write Enable Latch set => Finish without error
			break;
		}

		timeout++;

		if(timeout >= REGISTER_ACCESS_CLOCKSPEED * 125)
		{
			// Timeout occured => Raise error
			print("QUADSPI timeout setting WEL in SR1 of ext. flash", MESSAGE_CAUTION);
			return;
		}
	}
}

/**
 * Poll the ext. flash Status Register and extract relevant information
 * @return	Status of the ext. flash
 */
char NO_OPTIMIZATION S25FL512::get_state()
{
	// Get current status from Status Register 1 of ext. flash
	uint8_t status = this->readStatusRegister();

	// Check for errors
	if(status & S25FL512_CMD::SR1_E_ERR)
	{
		// An erase error occurred
		this->state = S25FL512_ERROR;
		print("Ext. flash erase error occurred", MESSAGE_CAUTION);
	}
	else if(status & S25FL512_CMD::SR1_P_ERR)
	{
		// A write error occurred
		this->state =  S25FL512_ERROR;
		print("Ext. flash write error occurred", MESSAGE_CAUTION);
	}
	else if(status & S25FL512_CMD::SR1_WIP)
	{
		// Ext. flash is busy ...
		if(this->state == S25FL512_ERASING)
		{
			// ... erasing all
			this->state = S25FL512_ERASING;
		}
		else
		{
			// ... writing something
			this->state = S25FL512_WRITING;
		}
	}
	else
	{
		// Ext. flash is in an idle condition
		this->state = S25FL512_IDLE;
	}

	return this->state;
}

/**
 * Erase the whole flash (Bulk erase) => all logs lost.
 * External flash has to be in an idle state. Check with 'getState()'.
 */
void NO_OPTIMIZATION S25FL512::erase()
{
	// Check if all previous events are finished
	if(this->get_state() != S25FL512_IDLE)
	{
		print("Access collision erasing ext. flash occurred", MESSAGE_CAUTION);
		return;
	}

	// Configure for erasing
	this->writeEnable();
	this->setClockspeed(REGISTER_ACCESS_CLOCKSPEED);

	// Indirect write; Instruction on 1 line; BE command
	QUADSPI->CCR = QUADSPI_CCR_IMODE_0 | S25FL512_CMD::BE;

	// Finish transfer and catch errors
	if(!this->finishTransfer())
	{
		// Error occured => Terminate
		print("QUADSPI error sending BE command to ext. flash", MESSAGE_CAUTION);
		return;
	}

	this->state = S25FL512_ERASING;
}

/**
 * Write a whole page of the external flash.
 * External flash has to be in an idle state. Check with 'getState()'.
 * @param pageAddress	The page to be written to. 0 to flashsize/512.
 * @param writeBuffer	Buffer (32bit) provided with data for writing the flash memory.
 */
void NO_OPTIMIZATION S25FL512::writePage(uint32_t pageAddress, uint32_t* writeBuffer)
{
	if(pageAddress >= this->pages)
	{
		// Memory overflow => Terminate
		print("External flash overflow: tried to write page ", (int)pageAddress, MESSAGE_CAUTION);
		return;
	}

	// Check if all previous events are finished
	if(this->get_state() != S25FL512_IDLE)
	{
		print("Access collision erasing ext. flash occurred", MESSAGE_CAUTION);
		return;
	}

	// Configure for writing
	this->writeEnable();
	this->setClockspeed(80);

	// Configure for sending 512 bytes: Indirect write; Data on 4 lines; 32-bit address on 1 line; Instruction on 1 line; QPP4 command
	QUADSPI->DLR = 511;
	QUADSPI->CCR = QUADSPI_CCR_DMODE | QUADSPI_CCR_ADSIZE | QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_IMODE_0 | S25FL512_CMD::QPP4;
	QUADSPI->AR = pageAddress * 512;

	// Shift all bytes into the FIFO
	for(int i=0; i<128; i++)
	{
		// Wait until FIFO has at least 4 free bytes or some error occured
		while(!(QUADSPI->SR & (QUADSPI_SR_FTF | QUADSPI_SR_TEF)));

		if(QUADSPI->SR & QUADSPI_SR_TEF)
		{
			// Error occured => Terminate
			print("QUADSPI error writing ext. flash at int #: ",i , MESSAGE_CAUTION);
			return;
		}

		// Write the next 4 bytes
		QUADSPI->DR = writeBuffer[i];
	}

	// Finish transfer and catch errors
	if(!this->finishTransfer())
	{
		// Error occured => Terminate
		print("QUADSPI error sending QPP4 command to ext. flash", MESSAGE_CAUTION);
		return;
	}

	this->state = S25FL512_WRITING;
}

/**
 * Read a whole page from the external flash.
 * External flash has to be in an idle state. Check with 'getState()'.
 * @param pageAddress	The page to be read from. 0 to flashsize/512.
 * @param readBuffer	Buffer (32bit) provided to store the received data.
 */
void NO_OPTIMIZATION S25FL512::readPage(uint32_t pageAddress, uint32_t* readBuffer)
{
	if(pageAddress >= this->pages)
	{
		// Memory overflow => Terminate
		print("External flash overflow: tried to read page ", (int)pageAddress, MESSAGE_CAUTION);
		return;
	}

	// Check if all previous events are finished
	if(this->get_state() != S25FL512_IDLE)
	{
		print("Access collision erasing ext. flash occurred", MESSAGE_CAUTION);
		return;
	}

	// Configure for reading
	this->setClockspeed(104);

	// Configure for receiving 512 bytes: Indirect read; Data on 4 lines; 8 Dummy Bits; 32-bit address on 1 line; Instruction on 1 line; QOR4 command
	QUADSPI->DLR = 511;
	QUADSPI->CCR = QUADSPI_CCR_FMODE_0 | QUADSPI_CCR_DMODE | (8 << 18) | QUADSPI_CCR_ADSIZE | QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_IMODE_0 | S25FL512_CMD::QOR4;
	QUADSPI->AR = pageAddress * 512;

	// Shift all bytes out of the FIFO
	for(int i = 0; i < 128; i++)
	{
		// Wait until FIFO holds at least 4 received bytes or some error occured
		while(!(QUADSPI->SR & (QUADSPI_SR_FTF | QUADSPI_SR_TEF)));

		if(QUADSPI->SR & QUADSPI_SR_TEF)
		{
			// Error occured => Terminate
			print("QUADSPI error writing ext. flash at int #: ",i , MESSAGE_CAUTION);
			return;
		}

		// Read the next 4 bytes
		readBuffer[i] = QUADSPI->DR;
	}

	// Finish transfer and catch errors
	if(!this->finishTransfer())
	{
		// Error occured => Terminate
		print("QUADSPI error sending QOR4 command to ext. flash", MESSAGE_CAUTION);
		return;
	}
}

/**
 * Constructor of the LogVaribale object.
 * Initialize an empty log entry.
 */
LogVariable::LogVariable()
{
	// Initialize all varaibles
	strcpy(this->name, "          EMPTY");
	this->name[15] = ',';
	this->type = LOG_TYPE_NONE;
	this->variable = nullptr;
}

/**
 * Set the name of the logged variable. (Gets printed out in first line of log)
 * @param name		Name of the logged variable (max. 15 characters)
 */
void LogVariable::set_name(const char* name)
{
	// Clear the name buffer
	strcpy(this->name, "               ");		// <- 15 whitespaces

	// Copy the name
	if(strlen(name) > 15)
	{
		// Name is too long
		strcpy(this->name, "  NAME_TOO_LONG");
	}
	else
	{
		// Name OK => Copy name with the right offset (right aligned)
		strcpy(this->name + 15 - strlen(name), name);
	}

	// Add the comma seperator instead of '\0'
	this->name[15] = ',';
}

/**
 * Retrieve the name of the variable
 * @return	Char pointer to the name buffer of the object
 */
char* LogVariable::get_name()
{
	return this->name;
}

/**
 * Store a pointer to the given variable for logging purposes
 * @param variable		Char variable to be logged
 */
void LogVariable::set_variable(volatile char* variable)
{
	this->variable = reinterpret_cast<volatile uint32_t*>(variable);

	this->type = LOG_TYPE_CHAR;
}

/**
 * Store a pointer to the given variable for logging purposes
 * @param variable		Int variable to be logged
 */
void LogVariable::set_variable(volatile int* variable)
{
	this->variable = reinterpret_cast<volatile uint32_t*>(variable);

	this->type = LOG_TYPE_INT;
}

/**
 * Store a pointer to the given variable for logging purposes
 * @param variable		Float variable to be logged
 */
void LogVariable::set_variable(volatile float* variable)
{
	this->variable = reinterpret_cast<volatile uint32_t*>(variable);

	this->type = LOG_TYPE_FLOAT;
}

/**
 * Retrieve the logged variable as an unsigned integer
 * @return	Unsigned integer variable (bit representation of stored datatype)
 */
uint32_t LogVariable::get_value()
{
	if(this->type == LOG_TYPE_CHAR)
		return *reinterpret_cast<volatile char*>(this->variable);
	else if(this->type == LOG_TYPE_INT)
		return *this->variable;
	else if(this->type == LOG_TYPE_FLOAT)
		return *this->variable;
	else
		return 0;
}

/**
 * Constructor of Datalog class.
 */
Datalog::Datalog()
{
	// System variables
	this->variable[0].set_name("log_tick");
	this->variable[0].set_variable(&this->tick);

	this->variable[1].set_name("system_time");
	this->variable[1].set_variable(&sys.time);

	this->variable[2].set_name("master_caution");
	this->variable[2].set_variable(&sys.masterCaution);

	this->variable[3].set_name("master_warning");
	this->variable[3].set_variable(&sys.masterWarning);

	this->variable[3].set_name("autopilot_mode");
	this->variable[3].set_variable(&sys.autopilot_mode);

	// ADC variables
	this->variable[10].set_name("V_T");
	this->variable[10].set_variable(&adc.V_T.filtered);

	this->variable[11].set_name("alpha");
	this->variable[11].set_variable(&adc.alpha.filtered);

	this->variable[12].set_name("beta");
	this->variable[12].set_variable(&adc.beta.filtered);

	this->variable[13].set_name("p_stat");
	this->variable[13].set_variable(&adc.p_stat.filtered);

	this->variable[14].set_name("T_stat");
	this->variable[14].set_variable(&adc.T_stat.filtered);

	this->variable[15].set_name("altitude");
	this->variable[15].set_variable(&adc.altitude);

	// GPS variables
	this->variable[16].set_name("lat_deg");
	this->variable[16].set_variable(&ins.gps.latitude.degrees);

	this->variable[17].set_name("lat_min");
	this->variable[17].set_variable(&ins.gps.latitude.minutes);

	this->variable[18].set_name("lon_deg");
	this->variable[18].set_variable(&ins.gps.longitude.degrees);

	this->variable[19].set_name("lon_min");
	this->variable[19].set_variable(&ins.gps.longitude.minutes);

	this->variable[20].set_name("gps_altitude");
	this->variable[20].set_variable(&ins.gps.altitude);

	this->variable[21].set_name("gps_course");
	this->variable[21].set_variable(&ins.gps.course);

	this->variable[22].set_name("gps_speed");
	this->variable[22].set_variable(&ins.gps.speed);

	this->variable[23].set_name("gps_mode");
	this->variable[23].set_variable(&ins.gps.mode);

	this->variable[24].set_name("gps_satellites");
	this->variable[24].set_variable(&ins.gps.satellites);

	this->variable[25].set_name("gps_hdop");
	this->variable[25].set_variable(&ins.gps.hdop);

	// Propulsion variables
	this->variable[26].set_name("omega_P");
	this->variable[26].set_variable(&propulsion.omega_P.filtered);

	this->variable[27].set_name("I_mot");
	this->variable[27].set_variable(&propulsion.I);

	this->variable[28].set_name("U_bat");
	this->variable[28].set_variable(&propulsion.U_bat);

	this->variable[29].set_name("Q_bat");
	this->variable[29].set_variable(&propulsion.Q_bat);

	// INS variables
	this->variable[30].set_name("omega_V_V_x");
	this->variable[30].set_variable(&ins.omega_V_V.filtered(0));

	this->variable[31].set_name("omega_V_V_y");
	this->variable[31].set_variable(&ins.omega_V_V.filtered(1));

	this->variable[32].set_name("omega_V_V_z");
	this->variable[32].set_variable(&ins.omega_V_V.filtered(2));

	this->variable[33].set_name("f_V_x");
	this->variable[33].set_variable(&ins.f_V.filtered(0));

	this->variable[34].set_name("f_V_y");
	this->variable[34].set_variable(&ins.f_V.filtered(1));

	this->variable[35].set_name("f_V_z");
	this->variable[35].set_variable(&ins.f_V.filtered(2));

	this->variable[36].set_name("B_E_V_x");
	this->variable[36].set_variable(&ins.B_E_V.filtered(0));

	this->variable[37].set_name("B_E_V_y");
	this->variable[37].set_variable(&ins.B_E_V.filtered(1));

	this->variable[38].set_name("B_E_V_z");
	this->variable[38].set_variable(&ins.B_E_V.filtered(2));

	this->variable[39].set_name("phi");
	this->variable[39].set_variable(&ins.Phi(0));

	this->variable[40].set_name("theta");
	this->variable[40].set_variable(&ins.Phi(1));

	this->variable[41].set_name("psi");
	this->variable[41].set_variable(&ins.Phi(2));

	this->variable[42].set_name("r_V_E_x");
	this->variable[42].set_variable(&ins.r_V_E(0));

	this->variable[43].set_name("r_V_E_y");
	this->variable[43].set_variable(&ins.r_V_E(1));

	this->variable[44].set_name("r_V_E_z");
	this->variable[44].set_variable(&ins.r_V_E(2));

	this->variable[45].set_name("v_V_V_x");
	this->variable[45].set_variable(&ins.v_V_V(0));

	this->variable[46].set_name("v_V_V_y");
	this->variable[46].set_variable(&ins.v_V_V(1));

	this->variable[47].set_name("v_V_V_z");
	this->variable[47].set_variable(&ins.v_V_V(2));

	this->variable[48].set_name("v_W_E_x");
	this->variable[48].set_variable(&ins.v_W_E.filtered(0));

	this->variable[49].set_name("v_W_E_y");
	this->variable[49].set_variable(&ins.v_W_E.filtered(1));

	this->variable[50].set_name("v_W_E_z");
	this->variable[50].set_variable(&ins.v_W_E.filtered(2));

	this->variable[51].set_name("lat_deg_ref");
	this->variable[51].set_variable(&ins.latitude_ref.degrees);

	this->variable[52].set_name("lat_min_ref");
	this->variable[52].set_variable(&ins.latitude_ref.minutes);

	this->variable[53].set_name("lon_deg_ref");
	this->variable[53].set_variable(&ins.longitude_ref.degrees);

	this->variable[54].set_name("lon_min_ref");
	this->variable[54].set_variable(&ins.longitude_ref.minutes);

	this->variable[55].set_name("phi_est");
	this->variable[55].set_variable(&ins.Phi_measured(0));

	this->variable[56].set_name("theta_est");
	this->variable[56].set_variable(&ins.Phi_measured(1));

	this->variable[57].set_name("psi_est");
	this->variable[57].set_variable(&ins.Phi_measured(2));

	this->variable[58].set_name("omega_bias_x");
	this->variable[58].set_variable(&ins.omega_V_V_bias(0));

	this->variable[59].set_name("omega_bias_y");
	this->variable[59].set_variable(&ins.omega_V_V_bias(1));

	this->variable[60].set_name("omega_bias_z");
	this->variable[60].set_variable(&ins.omega_V_V_bias(2));

	this->variable[61].set_name("a_g");
	this->variable[61].set_variable(&ins.a_g.filtered);

	this->variable[62].set_name("INS_state");
	this->variable[62].set_variable(&ins.state);

	// Flight Controls variables
	this->variable[70].set_name("CH1");
	this->variable[70].set_variable(&receiver.channel[0]);

	this->variable[71].set_name("CH2");
	this->variable[71].set_variable(&receiver.channel[1]);

	this->variable[72].set_name("CH3");
	this->variable[72].set_variable(&receiver.channel[2]);

	this->variable[73].set_name("CH4");
	this->variable[73].set_variable(&receiver.channel[3]);

	this->variable[74].set_name("CH5");
	this->variable[74].set_variable(&receiver.channel[4]);

	this->variable[75].set_name("CH6");
	this->variable[75].set_variable(&receiver.channel[5]);

	this->variable[76].set_name("CH7");
	this->variable[76].set_variable(&receiver.channel[6]);

	this->variable[77].set_name("CH8");
	this->variable[77].set_variable(&receiver.channel[7]);

	this->variable[78].set_name("CH9");
	this->variable[78].set_variable(&receiver.channel[8]);

	this->variable[79].set_name("CH10");
	this->variable[79].set_variable(&receiver.channel[9]);

	this->variable[80].set_name("CH11");
	this->variable[80].set_variable(&receiver.channel[10]);

	this->variable[81].set_name("CH12");
	this->variable[81].set_variable(&receiver.channel[11]);

	this->variable[82].set_name("CH13");
	this->variable[82].set_variable(&receiver.channel[12]);

	this->variable[83].set_name("CH14");
	this->variable[83].set_variable(&receiver.channel[13]);

	this->variable[84].set_name("CH15");
	this->variable[84].set_variable(&receiver.channel[14]);

	this->variable[85].set_name("CH16");
	this->variable[85].set_variable(&receiver.channel[15]);

	this->variable[86].set_name("CH17");
	this->variable[86].set_variable(&receiver.channel[16]);

	this->variable[87].set_name("CH18");
	this->variable[87].set_variable(&receiver.channel[17]);

	this->variable[88].set_name("delta_A");
	this->variable[88].set_variable(&aileron.position);

	this->variable[89].set_name("delta_E");
	this->variable[89].set_variable(&elevator.position);

	this->variable[90].set_name("delta_T");
	this->variable[90].set_variable(&throttle.position);

	this->variable[91].set_name("delta_R");
	this->variable[91].set_variable(&rudder.position);

	// SAS variables
	this->variable[100].set_name("bank_bypass");
	this->variable[100].set_variable(&sas.bank_bypass);

	this->variable[101].set_name("phi_cmd");
	this->variable[101].set_variable(&sas.phi_cmd);

	this->variable[102].set_name("beta_cmd");
	this->variable[102].set_variable(&sas.beta_cmd);

	this->variable[103].set_name("ground_lock");
	this->variable[103].set_variable(&sas.ground_lock);

	this->variable[104].set_name("gamma");
	this->variable[104].set_variable(&sas.gamma);

	this->variable[105].set_name("stall_mode");
	this->variable[105].set_variable(&sas.stall_mode);

	// Search for the first empty page for variable logging
	uint32_t buffer[128];

	for(this->tick = 0; this->tick < (int)this->message_log_start; this->tick++)
	{
		this->ext_flash.readPage(this->tick, buffer);

		if(buffer[0] == 0xFFFFFFFF)
			break;
	}

	// Search for the first empty page for message logging
	for(this->message_log_pos = this->message_log_start;
			this->message_log_pos < (int)this->ext_flash.pages; this->message_log_pos++)
	{
		this->ext_flash.readPage(this->message_log_pos, buffer);

		if(buffer[0] == 0xFFFFFFFF)
			break;
	}

	// Pause the datalog
	this->running = 0;
}

/**
 * Blink some LEDs while waiting for a press of the User Button
 */
void Datalog::serial_sync()
{
	// Wait until button is released
	sys.LedRed_clear();
	sys.LedBlue_clear();
	sys.LedGreen_clear();

	while(sys.userButton_pressed());

	// Do a nice running light while waiting for new button press
	char state = 1;

	while(!sys.userButton_pressed())
	{
		switch(state)
		{
		case 1:	sys.LedRed_set();
				sys.LedBlue_clear();
				sys.LedGreen_clear();
				break;
		case 2:	sys.LedRed_clear();
				sys.LedBlue_set();
				sys.LedGreen_clear();
				break;
		case 3:	sys.LedRed_clear();
				sys.LedBlue_clear();
				sys.LedGreen_set();
				break;
		case 4:	sys.LedRed_clear();
				sys.LedBlue_set();
				sys.LedGreen_clear();
				break;
		}

		state++;

		if(state > 4)
			state = 1;

		waitMs(150);
	}

	// Wait until button is released again
	while(sys.userButton_pressed());

	// Complete => Clear all LEDs
	sys.LedRed_clear();
	sys.LedBlue_clear();
	sys.LedGreen_clear();
}

/**
 * Send a character over serial COM port (USART3)
 * @param character		One byte to be sent over USART3
 */
void Datalog::send_char_COM(char character)
{
	// Wait until Transmit register is empty
	while(!(USART3->ISR & USART_ISR_TXE));

	// Send next character
	USART3->TDR = character;
}

/**
 * Resume the datalog
 */
void Datalog::start()
{
	if(!this->running)
	{
		this->running = 1;

		// Tell where the log is picked up
		print("Datalog resumed at log-tick ", this->tick, MESSAGE_NOTE);
	}
}

/**
 * Pause the datalog
 */
void Datalog::stop()
{
	if(this->running)
	{
		this->running = 0;

		// Tell where the log is paused
		print("Datalog paused at log-tick ", this->tick, MESSAGE_NOTE);
	}

	// Clear the green LED
	sys.LedGreen_clear();
}

/**
 * Update datalog by writing all current variables and messages to ext. flash.
 */
void Datalog::update()
{
	// Only log data if the log is suppused to be running
	if(this->running)
	{
		// Log all variables on ext. flash if it has space left
		if(this->tick < (int)(this->message_log_start - 1))
		{
			// Store all current variables as unsigned integers in a data buffer
			uint32_t data[128];

			for(int i = 0; i < 128; i++)
				data[i] = this->variable[i].get_value();

			// Write all variables to ext. flash
			this->ext_flash.writePage(this->tick, data);

			// Increase variable logging tick
			this->tick++;
		}
		else
		{
			// Ext. flash has no space for variable logging left
			print("External flash data log overflow", MESSAGE_CAUTION);
		}

		// Flash the green LED
		if(!(sys.tick % 2))
			sys.LedGreen_toggle();
	}

	// Log all new messages on ext. flash if it has space left
	if(this->message_log_pos < (int)(this->ext_flash.pages - 1))
	{
		// Check if there are new messages available for logging
		if(datalog_messageBuffer.status)
		{
			char buffer[512] = {'\0'};

			for(int i = 0; i < 6; i++)
			{
				// Check if there are new messages left
				if(!datalog_messageBuffer.status)
					break;

				// Get the message and add a new line at the end
				datalog_messageBuffer.receive(buffer + i * 85);
				buffer[i * 85 + 84] = '\n';
			}

			// Wait until last access is complete, then go ahead
			while(this->ext_flash.get_state() == S25FL512_WRITING);
			this->ext_flash.writePage(this->message_log_pos, reinterpret_cast<uint32_t*>(buffer));

			// Increase message logging tick
			this->message_log_pos++;
		}
	}
	else
	{
		// Ext. flash has no space for message logging left
		print("External flash message log overflow", MESSAGE_CAUTION);
	}
}

/**
 * Read out the whole datalog and send it as formatted text over serial.
 * Meant to be called for readout to a PC.
 */
void Datalog::read()
{
	// Wait until button pressed, then start sending variable log via UART
	this->serial_sync();

	// Send the header first (variable names)
	for(int i = 0; i < 128; i++)
		for(int j = 0; j < 16; j++)
			if((i != 127) || (j != 15))
				this->send_char_COM(this->variable[i].get_name()[j]);

	// Add a new line at the end
	this->send_char_COM('\n');

	// Continue sending the data
	uint32_t buffer[128];
	char strBuffer[32];

	for(uint32_t index = 0; index < this->message_log_start; index++)
	{
		// Skip data-log readout when user button is pressed
		if(sys.userButton_pressed())
			break;

		// Read the next page
		this->ext_flash.readPage(index, buffer);

		if(buffer[0] == 0xFFFFFFFF)
		{
			// Reached an unwritten page => End of log; Stop reading
			break;
		}

		// Generate a formatted string for each value and send it
		for(int i = 0; i < 128; i++)
		{
			// Format it
			int length = 0;

			if((this->variable[i].type == LOG_TYPE_CHAR) || (this->variable[i].type == LOG_TYPE_INT))
				length = toString(*reinterpret_cast<int*>(&buffer[i]), strBuffer);
			else if(this->variable[i].type == LOG_TYPE_FLOAT)
				length = toStringScientific(*reinterpret_cast<float*>(&buffer[i]), 7, strBuffer);

			if(i != 127)
			{
				strBuffer[length] = ',';
				length++;
			}

			// Send it
			for(int j = 0; j < length; j++)
				this->send_char_COM(strBuffer[j]);
		}

		// End the line
		this->send_char_COM('\n');

		// Flash status LEDs
		if(!(index % 5))
		{
			sys.LedRed_toggle();
			sys.LedBlue_toggle();
			sys.LedGreen_toggle();
		}
	}

	// Wait until button pressed again, then start sending message log via UART
	this->serial_sync();

	// Send the messages
	for(uint32_t index = this->message_log_start; index < this->ext_flash.pages; index++)
	{
		// Skip message-log readout when user button is pressed
		if(sys.userButton_pressed())
			break;

		// Read the next page
		this->ext_flash.readPage(index, buffer);

		if(buffer[0] == 0xFFFFFFFF)
		{
			// Reached an unwritten page => End of log; Stop reading
			break;
		}

		// Send the message string
		for(int i = 0; i < 512; i++)
			this->send_char_COM(reinterpret_cast<char*>(buffer)[i]);

		// Flash status LEDs
		if(!(index % 5))
		{
			sys.LedRed_toggle();
			sys.LedBlue_toggle();
			sys.LedGreen_toggle();
		}
	}
}

/**
 * Erases the whole datalog and resets everything to an initial state.
 * Meant to be called after reading the log and sending to a PC.
 */
void Datalog::erase()
{
	// Wait until button pressed, then start erasing log
	this->serial_sync();

	// Send erase command
	this->ext_flash.erase();

	// Blink some LEDs while waiting to be complete
	while(this->ext_flash.get_state() == S25FL512_ERASING)
	{
		sys.LedRed_toggle();
		sys.LedBlue_toggle();
		sys.LedGreen_toggle();

		waitMs(100);
	}

	// Complete => Clear all LEDs
	sys.LedRed_clear();
	sys.LedBlue_clear();
	sys.LedGreen_clear();
}
