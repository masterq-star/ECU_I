/*

This function provides a Variable Valve Timing controller via a PWM signal.

The algorithm simply provides a PWM signal proportional to the engine's RPM. The PWM signal controls
a valve that adjusts the inlet camshaft timing. 

Uses PWM output #2

A PWM power vs RPM slope is defined at initialisation.


This software/firmware source code or executable program is copyright of
Just Technology (North West) Ltd (http://www.just-technology.co.uk) 2020

This software/firmware source code or executable program is provided as free software:
you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version. The license is available at https://www.gnu.org/licenses/gpl-3.0.html

The software/firmware source code or executable program is distributed in the hope that
it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "vvt_controller.h"
#include "cfg_data.h"
#include "ecu_services.h"
#include "utility_functions.h"


// coefficient of the RPM - VVT slope
static float gradient;
static float offset;

// PWM1 & PWM2 are supplied as a % of maximum power, so must be in the range 0 - 100%
void vvInitialise() {

	// set the output signal to 0% duty signal - ie OFF
	setDutyCyclePWM1(0.0);
	
	// calculate gradient & offset for PWM vs RPM calc.
	gradient = (cfPage1.p1.vvtPWM2 - cfPage1.p1.vvtPWM1) / (cfPage1.p1.vvtRPM2 - cfPage1.p1.vvtRPM1);
	offset = cfPage1.p1.vvtPWM2 - gradient * cfPage1.p1.vvtRPM2;
}

// set the PWM output in accordance with PWM-RPM slope
float vvSetVVT(float RPM) {
	float PWM = rangeF(gradient * RPM + offset, cfPage1.p1.vvtPWM2, cfPage1.p1.vvtPWM1);
	setDutyCyclePWM1(PWM);
	return PWM;
}

