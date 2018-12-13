/*
 * Datalog.hpp
 *
 *  Created on: 14. Apr. 2017
 *      Author: Daniel
 */

#ifndef DATALOG_HPP_
#define DATALOG_HPP_

// Built in modules
#include "stm32f7xx.h"
#include <cstdint>
#include <cstring>

// !!! ONLY INCLUDE 'System.hpp' AND 'ArmHawk.hpp' !!!
#include "System.hpp"
#include "ArmHawk.hpp"

// Maximum clockspeed limit due to PCB restrictions
#define DATALOG_MAX_CLOCKSPEED	104

// Reduced clockspeed for securely accessing external flash registers
#define REGISTER_ACCESS_CLOCKSPEED	20

// Define some possible states of the external flash memory
#define S25FL512_ERROR		-1
#define	S25FL512_IDLE		0
#define	S25FL512_ERASING	1
#define	S25FL512_WRITING	2

// Define the loggable types
#define LOG_TYPE_NONE		0
#define LOG_TYPE_CHAR		1
#define LOG_TYPE_INT		2
#define LOG_TYPE_FLOAT		3

/**
 * Creates an object to access the Spansion S25FL512 external flash memory via QUADSPI
 */
class S25FL512
{
	// Size of flash memory according to formula size = 2 ^ (fsize + 1)
	const uint8_t fsize = 25;		// 64MB

	// Save the last known state for future comparisons
	char state;

	float setClockspeed(uint32_t clockspeed);
	char finishTransfer();
	uint8_t readStatusRegister();
	void writeEnable();

public:
	// Size of flash memory in bytes
	const uint32_t size =  (1 << (fsize + 1));

	// Number of flash memory pages (each page = 512 bytes)
	const uint32_t pages = (1 << (fsize - 8));

	char get_state();
	void erase();
	void writePage(uint32_t pageAddress, uint32_t* writeBuffer);
	void readPage(uint32_t pageAddress, uint32_t* readBuffer);

	S25FL512();
};

/**
 * Creates an log object, which holds a pointer to the logged variable
 * and a name of the variable.
 */
class LogVariable
{
	// Short name of variable; Gets printed out in first line in log; 15 characters + '\0'
	char name[16];

	// Unsigned integer pointer to the 4-byte log variable
	volatile uint32_t* variable;

public:
	// Type identifier of the logged variable
	char type;

	void set_name(const char* name);
	char* get_name();

	void set_variable(volatile char* variable);
	void set_variable(volatile int* variable);
	void set_variable(volatile float* variable);

	uint32_t get_value();

	LogVariable();
};

/**
 * Provides all tools for datalogging.
 * Divides the external flash into two parts:
 * 	-First part is used for logging variables (128 per page; 1 page per tick).
 * 	-Second part is used for logging messages (up to 8 per page; 1 page per tick).
 */
class Datalog
{
	// External flash object
	S25FL512 ext_flash;

	// Objects for the logged variables (containing a name and a pointer)
	LogVariable variable[128];

	// End of variable log and start of message log in ext. flash pages
	const uint32_t message_log_start = ext_flash.pages  * 3 / 4;

	// Track the current position of the message-log from start of ext. flash
	int message_log_pos;

	void serial_sync();
	void send_char_COM(char character);

public:
	// Track current log tick (mainloop ticks); Gets resumed after a reset
	int tick;

	// Condition if the data log is currently active or not
	char running;

	void start();
	void stop();

	void update();
	void read();
	void erase();

	Datalog();
};

namespace S25FL512_CMD
{
	// Register definitions
	const uint8_t WRR			= 0x01;
	const uint8_t PP 			= 0x02;
	const uint8_t READ			= 0x03;
	const uint8_t WRDI			= 0x04;
	const uint8_t RDSR1			= 0x05;
	const uint8_t WREN			= 0x06;
	const uint8_t RDSR2			= 0x07;
	const uint8_t FAST_READ		= 0x0B;
	const uint8_t FAST_READ4	= 0x0C;
	const uint8_t DDRFR			= 0x0D;
	const uint8_t DDRFR4		= 0x0E;
	const uint8_t PP4			= 0x12;
	const uint8_t READ4			= 0x13;
	const uint8_t ABRD			= 0x14;
	const uint8_t ABWR			= 0x15;
	const uint8_t BRRD			= 0x16;
	const uint8_t BRWR			= 0x17;
	const uint8_t ASPRD			= 0x2B;
	const uint8_t ASPP			= 0x2F;
	const uint8_t CLSR			= 0x30;
	const uint8_t QPP			= 0x32;
	const uint8_t QPP4			= 0x34;
	const uint8_t RDCR			= 0x35;
	const uint8_t DOR			= 0x3B;
	const uint8_t DOR4			= 0x3C;
	const uint8_t DLPRD			= 0x41;
	const uint8_t OTPP			= 0x42;
	const uint8_t PNVDLR		= 0x43;
	const uint8_t WVDLR			= 0x4A;
	const uint8_t OTPR			= 0x4B;
	const uint8_t RSFDP			= 0x5A;
	const uint8_t BE			= 0x60;
	const uint8_t QOR			= 0x6B;
	const uint8_t QOR4			= 0x6C;
	const uint8_t ERSP			= 0x75;
	const uint8_t ERRS			= 0x7A;
	const uint8_t PGSP			= 0x85;
	const uint8_t PGRS			= 0x8A;
	const uint8_t REMS			= 0x90;
	const uint8_t RDID			= 0x9F;
	const uint8_t MPM			= 0xA3;
	const uint8_t PLBWR			= 0xA6;
	const uint8_t PLBRD			= 0xA7;
	const uint8_t RES			= 0xAB;
	const uint8_t BRAC			= 0xB9;
	const uint8_t DIOR			= 0xBB;
	const uint8_t DIOR4			= 0xBC;
	const uint8_t DDRDIOR		= 0xBD;
	const uint8_t DDRDIOR4		= 0xBE;
	const uint8_t SE			= 0xD8;
	const uint8_t SE4			= 0xDC;
	const uint8_t DYBRD			= 0xE0;
	const uint8_t DYBWR			= 0xE1;
	const uint8_t PPBRD			= 0xE2;
	const uint8_t PPBP			= 0xE3;
	const uint8_t PPBE			= 0xE4;
	const uint8_t PASSRD		= 0xE7;
	const uint8_t PASSP			= 0xE8;
	const uint8_t PASSU			= 0xE9;
	const uint8_t QIOR			= 0xEB;
	const uint8_t QIOR4			= 0xEC;
	const uint8_t DDRQIOR		= 0xED;
	const uint8_t DDRQIOR4		= 0xEE;
	const uint8_t RESET			= 0xF0;
	const uint8_t MBR			= 0xFF;

	// Status Register 1 Bits
	const uint8_t SR1_SRWD		= (1 << 7);
	const uint8_t SR1_P_ERR		= (1 << 6);
	const uint8_t SR1_E_ERR		= (1 << 5);
	const uint8_t SR1_BP2		= (1 << 4);
	const uint8_t SR1_BP1		= (1 << 3);
	const uint8_t SR1_BP0		= (1 << 2);
	const uint8_t SR1_WEL		= (1 << 1);
	const uint8_t SR1_WIP		= (1 << 0);

	// Configuration Register 1 Bits
	const uint8_t CR1_LC1		= (1 << 7);
	const uint8_t CR1_LC0		= (1 << 6);
	const uint8_t CR1_TBPROT	= (1 << 5);
	const uint8_t CR1_BPNV		= (1 << 3);
	const uint8_t CR1_QUAD		= (1 << 1);
	const uint8_t CR1_FREEZE	= (1 << 0);
}

#endif /* DATALOG_HPP_ */
