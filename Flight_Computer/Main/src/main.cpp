/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Daniel Pusztai
  * @version V1.0
  * @date    07-20-2016
  * @brief   Main function of ARM-Hawk.
  ******************************************************************************
*/

// Built in modules
#include "stm32f7xx.h"

// Own modules
#include "System.hpp"
#include "Datalog.hpp"

// !!! INCLUDE ALL OWN PERIPHERAL MODULES VIA 'ArmHawk.hpp' !!!
#include "ArmHawk.hpp"

// !!! DO NOT DO ANYTHING BEFOR INITIALISING 'System' OBJECT !!!
volatile MessageBuffer debug_messageBuffer;
volatile MessageBuffer datalog_messageBuffer;
System sys;

// Vehicle CG position from vehicle reference point [m]
const Vector<3> r_CG_REF({-0.017, 0.000, 0.012});

// Init all peripheral systems
AirDataComputer adc(101325, r_CG_REF);

InertialNavigationSystem ins(r_CG_REF);

Servo aileron(1, 1907, 1525, 1062);
Servo elevator(2, 1911, 1517, 1147);
Servo throttle(3, 1100, 1520, 1940);
Servo rudder(4, 1131, 1518, 2005);
Servo flaps(5, 1764, 1520, 1151);
Servo ch6(6);

Receiver receiver;

Propulsion propulsion(4000);

StabilityAugmentationSystem sas;

Datalog datalog;

int main(void)
{
	// Check if the user button is pressed and go into datalog readout mode if so
	if(sys.userButton_pressed())
	{
		// Read log and send to COM port
		datalog.read();

		// Erase log and wait until finished
		datalog.erase();

		// Enter an infinite loop when finished -> Restart required
		while(true);
	}

#ifdef DEBUG
	print("Debug configuration flashed", MESSAGE_NOTE);
#endif

	// Give all external systems some time to initialize
	waitMs(1000);

	// Last step of initialization
	sys.finish_initialization();
	// !!! DO NOT DO ANYTHING BELOW HERE UNTIL MAINLOOP !!!

	while(true)
	{
		// Update receiver with latest S.BUS data and switch modes according to TX CH 8
		receiver.update();

		if(receiver.failsafe || (receiver.channel[7] < -0.25f))
		{
			// Switch to Standby Mode
			if(sys.autopilot_mode != AUTOPILOT_MODE_STANDBY)
				print("Engaged Autopilot Standby Mode", MESSAGE_NOTE);

			sys.autopilot_mode = AUTOPILOT_MODE_STANDBY;
		}
		else if(sys.masterWarning || (receiver.channel[7] < 0.25f))
		{
			// Switch to Direct Mode
			if(sys.autopilot_mode != AUTOPILOT_MODE_DIRECT)
				print("Engaged Autopilot Direct Mode", MESSAGE_NOTE);

			sys.autopilot_mode = AUTOPILOT_MODE_DIRECT;
		}
		else
		{
			// Switch to Stability Mode
			if(sys.autopilot_mode != AUTOPILOT_MODE_STABILITY)
				print("Engaged Autopilot Stability Mode", MESSAGE_NOTE);

			sys.autopilot_mode = AUTOPILOT_MODE_STABILITY;
		}

		// Update the propulsion state according to the system mode
		if(sys.autopilot_mode == AUTOPILOT_MODE_STANDBY)
			propulsion.update(receiver.channel[2]);
		else
			propulsion.update(throttle.position);

		// Update Air Data Computer with last cycle INS data (1 sample lag)
		adc.update(ins.omega_V_V.filtered);

		// Update Inertial Navigation System (GPS + GYRO + ACC + MAG) with ADC altitude data and motor current
		ins.update( propulsion.I, adc.altitude, adc.v_rel);

		// Update the stability augmentation system mode and controller gains
		sas.update(adc.alpha.filtered, adc.V_T.filtered, ins.Phi, ins.n.filtered(2), propulsion.omega_P.filtered, receiver.channel[4], sys.autopilot_mode);

		// Handle autopilot
		if(sys.autopilot_mode == AUTOPILOT_MODE_STABILITY)
		{
			if((sas.ground_lock) && (receiver.channel[12] > 0.25f) && (receiver.channel[2] < -0.95f))
			{
				// Perform self-test (activated via TX channel 13, only if throttle down)
				aileron.update(sas.self_test_roll(receiver.channel[8]));
				elevator.update(receiver.channel[1]);
				throttle.update(receiver.channel[2]);
				rudder.update(sas.self_test_yaw(receiver.channel[11]));
			}
			else
			{
				// Apply feedback control to flight controls from channel inputs 9~12
				aileron.update(sas.stabilize_roll(ins.Phi(0), ins.omega_V_V.filtered(0), receiver.channel[8]));
				elevator.update(sas.stabilize_pitch(ins.Phi(1), ins.omega_V_V.filtered(1), adc.alpha.filtered, adc.V_T.filtered, 0.7f * receiver.channel[9]));
				throttle.update(sas.stabilize_speed(receiver.channel[10]));
				rudder.update(sas.stabilize_yaw(adc.beta.filtered, ins.omega_V_V.filtered(2), receiver.channel[11]));
			}
		}
		else if(sys.autopilot_mode == AUTOPILOT_MODE_DIRECT)
		{
			// Update flight controls directly from channel inputs 1~4 (inc. trim/travel modifications)
			aileron.update(receiver.channel[0]);
			elevator.update(receiver.channel[1]);
			throttle.update(receiver.channel[2]);
			rudder.update(receiver.channel[3]);
		}
		else
		{
			// Standby mode => idle flight controls (disconnected via external channel multiplexer)
			aileron.update(0.0);
			elevator.update(0.0);
			throttle.update(-1.0);
			rudder.update(0.0);
		}

		// Update the onboard log => store the latest variables and messages in the external flash
		if(sys.userButton_pressed() && (!datalog.running))
		{
			print("Datalog manual start by user button", MESSAGE_NOTE);
			datalog.start();
		}
		else if((adc.V_T.filtered > 15.0) && (!datalog.running))
		{
			print("Datalog autostart due to airspeed > 15m/s", MESSAGE_NOTE);
			datalog.start();
		}

		datalog.update();

		// Visualize Master-Warnings and Master-Cautions
		if(sys.masterWarning)
		{
			sys.LedRed_set();
		}
		else if(sys.masterCaution)
		{
			if(!(sys.tick % 50))
				sys.LedRed_toggle();
		}
		else
		{
			sys.LedRed_clear();
		}

		// Visualize the state of the Inertial Navigation System
		if((ins.state == INS_CALIBRATING) || (ins.state == INS_WAIT_FOR_GPS) || (ins.state == INS_INITIALIZING))
		{
			if(!(sys.tick % 50))
				sys.LedBlue_toggle();
		}
		else
		{
			if(!(sys.tick % 2))
				sys.LedBlue_toggle();
		}

		// Finish the mainloop => handle remaining time in 50Hz-Mainloop, if there is any
		sys.finish_mainloop();
	}
}
