#ifndef _fuelInjection
#define _fuelInjection

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

#include "cfg_data.h"


// a structure to hold the current load, rpm cell indices
typedef struct {
	int rpmIndex;
	int loadIndex;
} currentCellStruct;

// defines a compensation curve for temperature
// two points on the curve t1, comp1 & t2, comp2 define the gradient (a) and offset (b)
// compensation is then calculated by comp = a * Temp + b
typedef struct {
	float t1; // lower temperature limit
	float t2; // upper limit
	float a;
	float b;
} temperatureCompDefn;


// general initialisation
extern void fuInitialise(float cyclicPeriod);

// gets the required injector pulse width (in micro-seconds) using MAP as engine load.
extern float getInjectorPulseWidth(float RPM, float load, float TPS, float engineTemperature, float airTemperature);

// gets the interpolated value from a map
extern float getMapInterpolatedValue(float map[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);

// Air-Fuel ratio correction array
// This provides an adjustment to the interpolated VE value for each cell. The AFRCorrection
// array must be computed externally to this class and written to directly, if AFR correction is required.
extern float AFRCorrection[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];

// pre-computes map indices and other variables common to interpolation routines for ignition and
// injection calcs. Must be called once prior to using injector pulse width calcs and ignition advance calcs
extern void mapLookup(float _RPM, float _load);

// for use by other functions
extern currentCellStruct currentCell;
extern float interpolatedVE;
extern float accelCompensationValue;
extern float tempComp;

#endif
