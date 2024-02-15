/*
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


#include "ignition.h"
#include "fuel_injection.h"
#include "math.h"



// the interpolated ignition advance value
float igAdvance;


/*
 * Gets the interpolated ignition angle.
 *
 * Note that veMapLookup() in fuel ingection must be run before using this function,
 * as it computes important variables required by the interpolation algorithm.
 *
 */
float igGetIgnitionAngle(){

	// save the interpolated advance value from the ignition map
	igAdvance = getMapInterpolatedValue(cfPage1.ignitionMap);
	
	// return the interpolated ignition advance value
	return igAdvance;
}

