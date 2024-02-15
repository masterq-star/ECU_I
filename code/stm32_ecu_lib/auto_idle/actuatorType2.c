
/*

This module is intended for a throttle body with an idle speed adjust motor, such as that fitted to a VW Golf Mk3.
The motor is controlled by a PWM signal on ECU pin STEPPER_A. When power is switched off to the motor, the throttle
butterfly is returned to the closed position by a spring.

On the VW Golf Mk3 Throttle Body, there is a switch to indicate idle (or foot-off-pedal). Switch closed = idle,
open = not idle. This must be connected directly to the ECU's auxiliary digital input. Active control of the idle
throttle position is only in effect when the idle switch is closed.




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


#include "auto_idle.h"
#include "cfg_data.h"
#include "utility_functions.h"
#include "ecu_services.h"
#include "main.h"
#include "math.h"


float aiType2ActuatorSetIdle(float TPS, float targetTPS){
	static float demand = 0.0F;
	static int delay = 0;

	

	// control the idle setting only if the idle switch is ON and after a short delay
	if ( TEST_IDLE_SWITCH_ON == TRUE ) {

		// delay a number of cycles to allow the TPS value to settle
		if (delay == 8) {

			// get the idle demand from the PID controller and limit the output to range 0% to 100% duty cycle
			demand = limitF(aiGetDemand(TPS, targetTPS), 0.0F, 100.0F);

			// if the demand is very small, set it to zero
			if (fabsf(demand) < 5.0F) {
				demand = 0.0F;
			}

			// set the actuator
			setDutyCyclePWM2(demand);
		}

		else {
			delay++;
		}

	}
	else {
		// reset the delay
		delay = 0;
	}
	return demand;
}


// set PID coefficients for Golf Mk3 TB Idle Motor
void aiType2ActuatorInit(){
	aiKi = 0.15F;
	aiKd = 0.9F;
}


/*+++REVISION_HISTORY+++
1) 11 Feb 2021 No longer test for zero demand, power to the idle motor is now not set to zero when
   not at idle and the cumulative error is no longer cleared. These changes required to assist in
   quickly restoring the correct idle setting when the trottle is abruptly closed.
2) 12 Feb 2021 Call to aiGetDemand() added to individual actuator functions. Called inside idle switch test to
   avoid cumulative error wind-up. Added a delay before acting on idle switch to allow the TPS value to catch up.
3) 16 Feb 2021 If the demand is very small, it is set to zero. "HoldPower" removed from algorithm.
4) 15 Feb 2021 Fixed error at line 53: was fabsf(demand < 5.0F), should have been: fabsf(demand) < 5.0F
+++REVISION_HISTORY_ENDS+++*/
