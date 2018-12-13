/*
 * InertialNavigationSystem.hpp
 *
 *  Created on: Apr 13, 2018
 *      Author: Daniel
 */

#ifndef INERTIALNAVIGATIONSYSTEM_HPP_
#define INERTIALNAVIGATIONSYSTEM_HPP_

// Built-in modules
#include "stm32f7xx.h"
#include <cstring>
#include <cstdint>

// Own modules
#include "System.hpp"
#include "ArmHawk_math.hpp"

// Define Venus638 GPS operational limits
#define GPS_MAX_CHECKSUM_ERRORS	2
#define GPS_MAX_RECEIVE_ERRORS	10

#define GPS_HDOP_LIMIT			2.0

// Define all possible errors on the USART
#define UART4_ERROR_MASK		(USART_ISR_ABRE | USART_ISR_NE | USART_ISR_FE)

// Masks all possible error states in the I2C ISR register
#define	I2C2_ERROR_MASK			(I2C_ISR_STOPF | I2C_ISR_TIMEOUT | I2C_ISR_ARLO | I2C_ISR_BERR | I2C_ISR_NACKF)
#define I2C2_ERROR_CLEAR		(I2C_ICR_STOPCF | I2C_ICR_TIMOUTCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_NACKCF)

// List all possible INS operation states
#define INS_RUNNING				0
#define INS_CALIBRATING			1
#define INS_WAIT_FOR_GPS		2
#define INS_INITIALIZING		3

// Class definitions
/**
 * Handles all I2C communication to the FXAS21002 gyroscope and FXOS8700 accelerometer/magnetometer
 */
class IMU_bus
{
	// Boolean to indicate if I2C has already been initialized
	static char initialized;

public:
	uint8_t readRegister(uint8_t address, uint8_t reg);
	void readRegister_multiple(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes);
	void writeRegister(uint8_t address, uint8_t reg, uint8_t data);
	void writeRegister_multiple(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes);

	IMU_bus();
};

/**
 * Interface for the FXAS21002 gyroscope from NXP via I2C
 */
class FXAS21002
{
	// Bus object for handling the data transfer
	IMU_bus bus;

	// Sensitivity factor to scale raw gyro data to rad/s
	const float sensitivity_gyro = 15.625e-3 * PI / 180.0;

	// Gyroscope calibration matrix
	const Matrix<3, 3> C_gyro = Matrix<3, 3>({1.0313e+00, -1.1339e-02, -7.2564e-03,
								  	 	 	 -7.8347e-03,  9.8760e-01,  7.4846e-03,
											  1.9856e-03,  6.5120e-04,  1.0242e+00});

	// Angular rate bias vector [rad/s]
	const Vector<3> omega_0 = Vector<3>({6.9040e-02, -4.8245e-02, 4.1435e-03});

	// G-force coupling matrix [rad*s/m]
	const Matrix<3, 3> G = Matrix<3, 3>({1.0539e-03, 		  0, 			0,
	            					              0, 9.4485e-04,            0,
												  0,          0,   1.2274e-03});

	// Turns 1, after first update poll has been made to keep track of overflows
	char running;

public:
	// Measured angular rates (transformed into IMU axes and calibrated) [rad/s]
	FIFO<32, Vector<3>> omega;

	void update(const Vector<3>& f);

	FXAS21002();
};

/**
 * Interface for the FXOS8700 accelerometer and magnetometer from NXP via I2C
 */
class FXOS8700
{
	// Bus object for handling the data transfer
	IMU_bus bus;

	// Sensitivity factor to scale raw accelerometer data to m/s^2
	const float sensitivity_acc = (1.0 / 1024.0) * 9.80665;

	// Accelerometer calibration matrix
	const Matrix<3, 3> C_acc = Matrix<3, 3>({9.8459e-01,  8.0660e-04, -2.3062e-02,
									  	  	-3.3592e-03,  1.0027e+00,  2.4024e-03,
											 4.9667e-03, -3.3054e-03,  1.0109e+00});

	// Acceleration bias vector [m/s^2]
	const Vector<3> f_0 = Vector<3>({-7.6722e-02, -8.5341e-02, -7.2458e-02});

	// Sensitivity factor to scale raw magnetometer data to T
	const float sensitivity_mag = 0.1e-6;

	// Magnetometer calibration matrix
	const Matrix<3, 3> C_mag = Matrix<3, 3>({1.0491e+00,  1.7427e-02,  1.7537e-02,
		  	  	  	  	  	  	  	  	  	-1.7268e-02,  1.0827e+00, -9.2422e-03,
											-4.1222e-02,  3.3487e-03,  1.0721e+00});

	// Hard-iron offset vector [T]
	const Vector<3> B_0 = Vector<3>({-1.0318e-04, -6.9476e-05, -7.3740e-05});

	// Current dependent hard-iron offset vector [T/A]
	const Vector<3> B_I = Vector<3>({8.2807e-09, 1.2813e-08, -2.3106e-08});

	// Turns 1, after first update poll has been made to keep track of overflows
	char running;

public:
	// Measured specific force (already transformed into IMU axes and calibrated) [m/s^2]
	FIFO<16, Vector<3>> f;

	// Measured magnetic flux density (already transformed into IMU axes and calibrated) [T]
	Vector<3> B;

	void update(float motor_current);

	FXOS8700();
};

/**
 * Holds a coordinate value in degrees and minutes (floats) for improved precision
 */
class Coordinate
{
public:
	// Degree part of the coordinate without any decimal places
	float degrees;

	// Minute part of the coordinate, includes seconds as decimals
	float minutes;

	double get_radians();
	double get_degrees();
	double get_minutes();
	double get_seconds();

	Coordinate();
	Coordinate(float deg, float min);
	Coordinate(double deg);
};

/**
 * Interface for the Venus638 GPS sensor from SkyTraq via UART
 */
class Venus638
{
	// Holds the system time of the last position update
	float update_time;

	// Amount of consecutive checksum errors
	int checksumErrors;

	int autoBaud();
	int sendCommand(char* payload, uint16_t payload_length);
	void setup();

	int getFieldStart(char* sentence, int field);
	int getFieldEnd(char* sentence, int field);

public:
	// 7 buffers for the received NMEA 0183 sentences
	static volatile char NMEA_sentence[7][80];

	// 1, if the corresponding buffer is filled and ready to be processed
	static volatile char NMEA_ready[7];

	// Indicates the buffer that the received characters must be written to (0 to 6)
	static volatile uint8_t rx_buffer;

	// Holds the current receiving position for NMEA_sentence buffer (0 to 79)
	static volatile uint8_t rx_position;

	// Amount of consecutive receive errors
	static volatile int receiveErrors;

	// Turns to 1, after the first 3D-position lock has been established
	char locked;

	// Geographical latitude [degree]
	Coordinate latitude;

	// Geographical longitude [degree]
	Coordinate longitude;

	// Altitude over MSL [m]
	float altitude;

	// Speed over ground [m/s]
	float speed;

	// Course over ground [degree]
	float course;

	// Operating mode of receiver (0~8); 1 == 3D position fix
	int mode;

	// Number of satellites in use
	int satellites;

	// Horizontal dilution of precision
	float hdop;

	void update();

	Venus638();
};

class InertialNavigationSystem
{
	// Vector pointing from vehicle reference point to IMU measurement point [m]
	const Vector<3> r_IMU_REF = Vector<3>({-0.060, -0.015, 0.038});

	// Vehicle CG position from IMU measurement point [m] (is precomputed at initialization)
	Vector<3> r_CG_IMU;

	// Flattening factor of the earth according to WGS84
	const double earth_f = 1.0 / 298.257223563;

	// Semi-major axis (equatorial radius) of the earth according to WGS84 [m]
	const double earth_a = 6378137.0;

	// Radius of curvature in the prime vertical of the earth [m] (is precomputed at initialization)
	double earth_RN;

	// Radius of curvature in the meridian of the earth [m] (is precomputed at initialization)
	double earth_RM;

	// Gravity vector in earth coordinates [m/s^2]
	const Vector<3> g_E = Vector<3>({0, 0, 9.8082});

	// Earth magnetic flux density vector in earth coordinates [T]
	const Vector<3> B_E_E = Vector<3>({2.1048e-5, 1.5839e-6, 4.3789e-5});

	// Reference (earth) basis for estimating attitude (is precomputed at initialization)
	Matrix<3, 3> E;

	// Counter for tracking the initialization timeline
	int init_step;

	// State vector of rotational kalman filter (rotation quaternion)
	Vector<4> q;

	// reduced state covariance matrix for rotational kalman filter
	Matrix<3, 3> P_rot;

	// Process covariance matrix for rotational kalman filter [(rad/s)^2]
	const Matrix<3, 3> Q_rot = Matrix<3, 3>({3.3e-2, 0, 	 0,
											 0,      7.4e-3, 0,
											 0, 	 0,      3.4e-4});

	// Measurement covariance matrix for rotational kalman filter [rad^2]
	const Matrix<3, 3> R_rot = Matrix<3, 3>({0.32, 0,     0,
											 0,    0.051, 0,
											 0,    0,     0.032});

	// From GPS and barometer estimated vehicle position and altitude
	Vector<3> r_V_E_measured;

	// State vector of translational kalman filter
	Vector<6> x_trans;

	// State covariance for translational kalman filter
	Matrix<6, 6> P_trans;

	// Process covariance matrix for translational kalman filter [(m/s^2)^2 / (rad/s)^2]
	const Matrix<3, 3> Q_trans = Matrix<3, 3>({4.4, 0,  0,
											   0,   44, 0,
											   0,   0,  15});

	// Measurement covariance matrix for translational kalman filter [m^2]
	const Matrix<3, 3> R_trans = Matrix<3, 3>({0.32f, 0,     0,
											   0,     0.32f, 0,
											   0,     0,     0.0625f});

	void initialize(float altitude_baro);

	void rotation_measure();
	void rotation_kalman_predict();
	void rotation_kalman_update();
	void rotation_output();

	void translation_measure(float altitude_baro);
	void translation_kalman_predict();
	void translation_kalman_update();
	void translation_output();
public:

	// Stores the current operational mode of the Inertial Navigation System
	int state;

	// Reference position for calculating flat-earth coordinates
	Coordinate latitude_ref;
	Coordinate longitude_ref;

	// GPS sensor
	Venus638 gps;

	// Angular rate sensor
	FXAS21002 gyro;

	// Accleration and magnetic field sensor
	FXOS8700 acc_mag;

	// Earth magnetic flux density vector in vehicle coordinates [T]; Bessel lowpass 2nd order 2.5Hz
	Filter<3, 3, Vector<3>> B_E_V;

	// Startup angular rate (in vehicle coordinates) bias [rad/s] (is measured at initialization)
	Vector<3> omega_V_V_bias;

	// Vehicle angular velocity in vehicle coordinates [rad/s]; Butterworth lowpass 3rd order 20Hz
	Filter<4, 4, Vector<3>> omega_V_V;

	// Attitude estimation covariance gain (quality of gravity vector estimation); Butterworth lowpass 1st order 0.2Hz
	Filter<2, 2, float> a_g;

	// From acceleration and magnetic field vector estimated vehicle attitude
	Vector<3> Phi_measured;

	// Vehicle attitude vector (roll; pitch; yaw) [rad]
	Vector<3> Phi;

	// Specific force in vehicle coordinates [m/s^2]; Bessel lowpass 3rd order 10Hz
	Filter<4, 4, Vector<3>> f_V;

	// Load factor [g]; Bessel lowpass 2nd order 2.5Hz
	Filter<3, 3, Vector<3>> n;

	// Vehicle velocity in vehicle coordinates [m/s]
	Vector<3> v_V_V;

	// Vehicle flat-earth position in NED earth coordinates [m]
	Vector<3> r_V_E;

	// Wind velocity in NED earth coordinates [m/s]; Butterowrth lowpass 1st order 0.01Hz
	Filter<3, 3, Vector<3>> v_W_E;

	// Transformation matrix from earth to vehicle coordinates
	Matrix<3, 3> T_VE;

	void update(float motor_current, float altitude_baro, Vector<3> v_rel);

	InertialNavigationSystem(Vector<3> r_CG_REF);
};

// Helper functions
double stringToDouble(const char* ascii, int length);
double stringToDouble(const char* ascii);

namespace FXAS21002_CMD
{
	// FXAS21002 I2C address
	const uint8_t ADDRESS			= 0x20;

	// FXAS21002 registers
	const uint8_t STATUS			= 0x00;
	const uint8_t OUT_X_MSB			= 0x01;
	const uint8_t OUT_X_LSB			= 0x02;
	const uint8_t OUT_Y_MSB			= 0x03;
	const uint8_t OUT_Y_LSB			= 0x04;
	const uint8_t OUT_Z_MSB			= 0x05;
	const uint8_t OUT_Z_LSB			= 0x06;
	const uint8_t DR_STATUS			= 0x07;
	const uint8_t F_STATUS			= 0x08;
	const uint8_t F_SETUP			= 0x09;
	const uint8_t F_EVENT			= 0x0A;
	const uint8_t INT_SRC_FLAG		= 0x0B;
	const uint8_t WHO_AM_I			= 0x0C;
	const uint8_t CTRL_REG0			= 0x0D;
	const uint8_t RT_CFG			= 0x0E;
	const uint8_t RT_SRC			= 0x0F;
	const uint8_t RT_THS			= 0x10;
	const uint8_t RT_COUNT			= 0x11;
	const uint8_t TEMP				= 0x12;
	const uint8_t CTRL_REG1			= 0x13;
	const uint8_t CTRL_REG2			= 0x14;
	const uint8_t CTRL_REG3			= 0x15;
}

namespace FXOS8700_CMD
{
	// FXOS8700 I2C address
	const uint8_t ADDRESS			= 0x1E;

	// FXOS8700 registers
	const uint8_t STATUS			= 0x00;
	const uint8_t OUT_X_MSB			= 0x01;
	const uint8_t OUT_X_LSB			= 0x02;
	const uint8_t OUT_Y_MSB			= 0x03;
	const uint8_t OUT_Y_LSB			= 0x04;
	const uint8_t OUT_Z_MSB			= 0x05;
	const uint8_t OUT_Z_LSB			= 0x06;
	const uint8_t F_SETUP			= 0x09;
	const uint8_t TRIG_CFG			= 0x0A;
	const uint8_t SYSMOD			= 0x0B;
	const uint8_t INT_SOURCE		= 0x0C;
	const uint8_t WHO_AM_I			= 0x0D;
	const uint8_t XYZ_DATA_CFG		= 0x0E;
	const uint8_t HP_FILTER_CUTOFF	= 0x0F;

	const uint8_t CTRL_REG1			= 0x2A;
	const uint8_t CTRL_REG2			= 0x2B;
	const uint8_t CTRL_REG3			= 0x2C;
	const uint8_t CTRL_REG4			= 0x2D;
	const uint8_t CTRL_REG5			= 0x2E;
	const uint8_t OFF_X				= 0x2F;
	const uint8_t OFF_Y				= 0x30;
	const uint8_t OFF_Z				= 0x31;
	const uint8_t M_DR_STATUS		= 0x32;
	const uint8_t M_OUT_X_MSB		= 0x33;
	const uint8_t M_OUT_X_LSB		= 0x34;
	const uint8_t M_OUT_Y_MSB		= 0x35;
	const uint8_t M_OUT_Y_LSB		= 0x36;
	const uint8_t M_OUT_Z_MSB		= 0x37;
	const uint8_t M_OUT_Z_LSB		= 0x38;
	const uint8_t CMP_X_MSB			= 0x39;
	const uint8_t CMP_X_LSB			= 0x3A;
	const uint8_t CMP_Y_MSB			= 0x3B;
	const uint8_t CMP_Y_LSB			= 0x3C;
	const uint8_t CMP_Z_MSB			= 0x3D;
	const uint8_t CMP_Z_LSB			= 0x3E;
	const uint8_t M_OFF_X_MSB		= 0x3F;
	const uint8_t M_OFF_X_LSB		= 0x40;
	const uint8_t M_OFF_Y_MSB		= 0x41;
	const uint8_t M_OFF_Y_LSB		= 0x42;
	const uint8_t M_OFF_Z_MSB		= 0x43;
	const uint8_t M_OFF_Z_LSB		= 0x44;
	const uint8_t MAX_X_MSB			= 0x45;
	const uint8_t MAX_X_LSB			= 0x46;
	const uint8_t MAX_Y_MSB			= 0x47;
	const uint8_t MAX_Y_LSB			= 0x48;
	const uint8_t MAX_Z_MSB			= 0x49;
	const uint8_t MAX_Z_LSB			= 0x4A;
	const uint8_t MIN_X_MSB			= 0x4B;
	const uint8_t MIN_X_LSB			= 0x4C;
	const uint8_t MIN_Y_MSB			= 0x4D;
	const uint8_t MIN_Y_LSB			= 0x4E;
	const uint8_t MIN_Z_MSB			= 0x4F;
	const uint8_t MIN_Z_LSB			= 0x50;
	const uint8_t TEMP				= 0x51;

	const uint8_t M_CTRL_REG1		= 0x5B;
	const uint8_t M_CTRL_REG2		= 0x5C;
	const uint8_t M_CTRL_REG3		= 0x5D;
	const uint8_t M_INT_SRC			= 0x5E;
}

namespace VENUS638_CMD
{
	// Venus638 register values
	namespace BAUD
	{
		const uint8_t SUCCESS		= -2;
		const uint8_t ERROR			= -1;
		const uint8_t B4800			= 0;
		const uint8_t B9600			= 1;
		const uint8_t B19200		= 2;
		const uint8_t B38400		= 3;
		const uint8_t B57600		= 4;
		const uint8_t B115200		= 5;
	}

	namespace MODE
	{
		const uint8_t NOFIX			= 0;
		const uint8_t FIX3D			= 1;
		const uint8_t DGPS			= 2;
		const uint8_t PPS			= 3;
		const uint8_t RTK			= 4;
		const uint8_t RTKF			= 5;
		const uint8_t DEADRECKONING	= 6;
		const uint8_t MANUAL		= 7;
		const uint8_t SIM			= 8;
	}

	// Serial communication satus
	const uint8_t TIMEOUT			= 0;
	const uint8_t ACK				= 1;
	const uint8_t NACK				= 2;
	const uint8_t BUS_ERROR			= 3;
}

#endif /* INERTIALNAVIGATIONSYSTEM_HPP_ */
