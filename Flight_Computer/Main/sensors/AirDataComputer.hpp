/*
 * AirDataComputer.hpp
 *
 *  Created on: Jan 22, 2018
 *      Author: Daniel
 */

#ifndef AIRDATACOMPUTER_HPP_
#define AIRDATACOMPUTER_HPP_

// Built-in modules
#include "stm32f7xx.h"
#include <cstdint>

// Own modules
#include "System.hpp"
#include "ArmHawk_math.hpp"

// Masks all possible error states in the I2C ISR register
#define	I2C4_ERROR_MASK		(I2C_ISR_TIMEOUT | I2C_ISR_ARLO | I2C_ISR_BERR | I2C_ISR_NACKF)
#define I2C4_ERROR_CLEAR		(I2C_ICR_TIMOUTCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_NACKCF)

// Sensor numeration for enabling the SDA line
#define	DISABLE_ALL		0
#define	SENSOR_1		1
#define	SENSOR_2		2
#define	SENSOR_3		3

// Class definitions
/**
 * Handles all I2C communication to the sensors on the sensor unit
 */
class SensorUnitBus
{
	// Boolean to indicate if I2C has already been initialized
	static char initialized;

public:
	void read(uint8_t address, uint8_t* buffer, uint8_t nBytes);
	void write(uint8_t address, uint8_t* buffer, uint8_t nBytes);
	void readRegister(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes);
	void enableSensor(char sensorNumber);
	char endOfConversion();

	SensorUnitBus();
};

/**
 * Handles one SMI SM9541 differential pressure sensor with the help of a 'SensorUnitBus'-object
 */
class SM9541
{
	// Maximum value of the pressure reading of the sensor [Pa]
	float scale;

	// Two status bits of the sensor
	char status;

	// Number of SDA line connected to the sensor
	char sensorNumber;

	// Bus object for handling the data transfer
	SensorUnitBus bus;

public:
	// Differential pressure measured between the two ports [Pa]
	float pressure;

	// Internal temperature reading of the sensor [C]
	float temperature;

	void update();

	SM9541(char sensorNumber, float scale);
};

/**
 * Handles the Bosch BMP085 barometer with the help of a 'SensorUnitBus'-object
 */
class BMP085
{
	// Calibration data
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int32_t B5;
	int16_t MC;
	int16_t MD;

	// Counter to balance pressure update (important) vs temperature update (not so)
	char cycle;

	// Bus object for handling the data transfer
	SensorUnitBus bus;

public:
	// Absolute pressure measured at the port [Pa]
	float pressure;

	// Internal temperature reading of the sensor [C]
	float temperature;

	void update();

	BMP085();
};

/**
 * Handles all sensors connected to the 5-hole probe
 */
class AirDataComputer
{
	// Calibration factor to account for Sensor Unit warming: T_true= C_temp * T_measured
	const float C_temp = 0.88;

	// Vector pointing from vehicle reference point to tip of probe location [m]
	const Vector<3> r_ADC_REF = Vector<3>({-0.160, 0.682, 0.077});

	// Vhicle CG position from tip of probe location [m] (is precomputed at initialization)
	Vector<3> r_CG_ADC;

	// Differential pressure sensor connected to the pressure ports 1 (+Z) & 2 (-Z)
	SM9541 diff_pressure_12;

	// Differential pressure sensor connected to the pressure ports 3 (+Y) & 4 (-Y)
	SM9541 diff_pressure_34;

	// Differential pressure sensor connected to the pressure ports 0 (+X) & S (static)
	SM9541 diff_pressure_0S;

	// Barometer for measuring static pressure
	BMP085 barometer;
public:
	// Static air pressure reduced to MSL [Pa]
	float QNH;

	// Static outside air temperature [C]; Butterworth lowpass 1st order 0.02Hz
	Filter<2, 2, float> T_stat;

	// Static air pressure [Pa]; Bessel lowpass 2nd order 0.3Hz
	Filter<3, 3, float> p_stat;

	// Air density [kg/m^3]
	float rho;

	// Airplane altitude over MSL due to static air pressure measurement [m]
	float altitude;

	// Angle of attack [rad]; Bessel lowpass 2nd order 2.2Hz
	Filter<3, 3, float> alpha;

	// Sideslip angle [rad]; Bessel lowpass 2nd order 2.2Hz
	Filter<3, 3, float> beta;

	// Total component of relative vehicle velocity [m/s]; Butterworth lowpass 1st order 0.8Hz
	Filter<2, 2, float> V_T;

	// Vector of vehicle velocity relative to air in vehicle coordinates [m/s]
	Vector<3> v_rel;

	void update(const Vector<3>& omega_v);

	AirDataComputer(float QNH, Vector<3> r_CG_REF);
};

// Variable definitions
namespace ADC_data
{
	extern const float pressure_step;

	extern const int p_0S_length;
	extern const int p_34_length;
	extern const int p_12_length;

	extern const float alpha[41][441];
	extern const float beta[41][441];
	extern const float q_T[41][441];
}

#endif /* AIRDATACOMPUTER_HPP_ */
