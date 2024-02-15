/*

These functions are reserved for a single port Bosch Throttle Body Injector, as fitted to the Mini SPi and
early VWs, etc, with an integral stepper motor for idle control.

However, these functions are not yet available on this configuration of MPU.


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


float aiType1ActuatorSetIdle(float TPS, float targetTPS){
	return 0.0;
}

void aiType1ActuatorInit(){

}


/*+++REVISION_HISTORY+++
1) 12 Feb 2021 aiType1ActuatorSetIdle() arguments changed.
+++REVISION_HISTORY_ENDS+++*/
