/*
 * ArmHawk.hpp
 *
 *  Created on: Apr 4, 2018
 *      Author: Daniel
 */

#ifndef ARMHAWK_HPP_
#define ARMHAWK_HPP_

// !!! ONLY INCLUDE PERIPHERAL MODULES !!!
#include "AirDataComputer.hpp"
#include "InertialNavigationSystem.hpp"
#include "FlightControls.hpp"
#include "Autoflight.hpp"

// Declare global variables; Definition in 'main.cpp'
extern AirDataComputer adc;

extern InertialNavigationSystem ins;

extern Servo aileron;
extern Servo elevator;
extern Servo throttle;
extern Servo rudder;
extern Servo ch5;
extern Servo ch6;

extern Receiver receiver;

extern Propulsion propulsion;

extern StabilityAugmentationSystem sas;

#endif /* ARMHAWK_HPP_ */
