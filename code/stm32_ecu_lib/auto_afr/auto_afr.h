#ifndef _autoAFRClass
#define _autoAFRClass

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


typedef struct {
	float lambdaAverages[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];	// the long term average Lambda sensor reading in millivolts.
	float lambdaSamples[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];		// the number of samples for the cell, 1 = 100 samples.
	float cumulativeError[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];	// cumulative error/1000.
} afrDataStruct;


extern void afInitialise(float cyclicPeriod, float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);
extern HAL_StatusTypeDef afResetAFR(float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);
extern int afSaveAFRData(float RPM, float engineTemp);
extern void afGetSample(float *correction, float *average, float *samples, float *AFRIndex, float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);
extern void afComputeCorrection(float RPM, float engineTemp, int loadIndex, int rpmIndex, float lambdaVoltage, float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);
extern afrDataStruct afrData;

#endif
