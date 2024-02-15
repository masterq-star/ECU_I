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


#include "fuel_injection.h"
#include "utility_functions.h"
#include "math.h"

// the applied acceleration compensation value
float accelCompensationValue;

// the product of air temp comp and engine temp comp
float tempComp;

// acceleration compensation variables
// Absolute limit accel comp value
static float accelCompLimit;

// acceleration comp amplitude, multiplies dTPS
static float accelCompAmplitude;

// accel comp time constant for the compensation decay
static float accelCompTC;

// internal control variables
static float lowPassTPS_1;
static float accelCompPeak;

// temp compensation co-efficients for engine temperature [0] and air temperature [1]
temperatureCompDefn tempCompCoeff[2];

// pre-computed reciprocals used to speed up real-time calcs by allowing mult vs divide
static float rpmDeltaReciprocal;
static float loadDeltaReciprocal;

// saved values used by getMapInterpolatedValue(), computed by veMapLookup()
static float relRPM;
static float relLoad;
static int r1, r2, l1, l2;

// the current applied indexes
currentCellStruct currentCell;

// the interpolated VE value
float interpolatedVE;

// post-start enrichment - an decaying enrichment factor that applies just after the engine has started
// i.e. RPM exceeds the cranking threshold RPM. pse is initialised to PSEStart and is reduced each cycle
// by PSEDecay until reaching 1.0
static float PSE;

// post-start enrichment start value, defined as a multiplier, e.g. 1.2 provides 20% enrichment
static float PSEStart;

// defines the amount of decay applied to PSE per cycle
// calculated from (PSEStart - 1) * CYCLIC_EVENT_PERIOD / T
// where T is the time required in milliseconds
static float PSEDecay;


// Air-Fuel ratio correction array
// This provides an adjustment to the interpolated VE value for the cell. The AFRCorrection
// array must be computed externally to this class and written to directly, if AFR correction is required.
float AFRCorrection[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];

// VE Map with the AFR correction applied
float veMapCorrected[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];


// initialise post-start enrichment
// startValue is a percentage fuel enrichment. timePeriod is in seconds and cyclicPeriod is in milliseconds
void initPostStartEnrichment(float startValue, float timePeriod, float cyclicPeriod) {
	PSEStart = 1.0F + 0.01F * startValue; // turn percentage into a multiplying factor
	PSE = PSEStart;
	PSEDecay = (PSEStart - 1) * cyclicPeriod / (1000 * timePeriod);
}

// Note that if AFR corrections are required, the calling function must ensure that the array AFRCorrection is
// set prior to calling this function.
void resetCorrectionArray() {
	for (int r=0; r < VE_MAP_SIZE_LOAD; r++)
		for (int c=0; c < VE_MAP_SIZE_RPM; c++) {
			veMapCorrected[r][c] = cfPage1.veMap[r][c] + AFRCorrection[r][c];
		}
}

void initAccelCompensation(float limit, float amplitudeFactor, float time, float cyclicPeriod) {

	// time constant based on required time (in millseconds) taken to fall to 10% for a step input
	accelCompTC = -logf(0.1F / amplitudeFactor) * cyclicPeriod / time ;

	// the amplitude factor defines the peak amplitude of the compensation
	// for a given step input of TPS. i.e. 2 give compensation at twice the 
	// impulse level of a TPS input
	// this is converted into a simplifed amplitude required internally
	accelCompAmplitude = 2 * amplitudeFactor / (1 - accelCompTC);

	// absolute limit is times 2 as the output is clipped to 50% of peak
	accelCompLimit = 2 * limit;	

	lowPassTPS_1 = 0;
	accelCompPeak = 0;
	accelCompensationValue = 0;
}

// accel comp using with decay filter & clip
float accelCompensation1(float TPS) {

	// get a low-pass filtered version of TPS
	float lowPassTPS = (TPS - lowPassTPS_1) * accelCompTC + lowPassTPS_1;
	lowPassTPS_1 = lowPassTPS;

	// calculate a new compensation value
	float newAccelComp = accelCompAmplitude * (TPS - lowPassTPS);

	// only +ve values allowed
	newAccelComp = newAccelComp > 0 ? newAccelComp : 0;

	// capture the peak value, if new accel comp greater than last peak
	// limit the peak to the absolute compensation limit
	if (newAccelComp > accelCompPeak)
	{
		accelCompPeak = newAccelComp > accelCompLimit ? accelCompLimit : newAccelComp;
	}

	// set the o/p clip at X% of the peak value
	float clip = 0.5F * accelCompPeak;

	if (newAccelComp > clip)
	{
		// new accel value greater than the clip value, so apply clip
		accelCompensationValue = clip;
	}
	else
	{
		// pass new acceleration comp to o/p
		accelCompensationValue = newAccelComp;

		// and since the accel comp is below the clip level, set the new peak at 
		// level to ensure no clipping until new peak captured
		accelCompPeak = 2 * newAccelComp;
	}

	return accelCompensationValue;
}

// initialises a temperature compensation slope. 
// Compensation values are provided as a percent. i.e. 10% = 10% increase in fuel flow.
// index provides for 2 slopes:
//   0 = engine temp comp
//   1 = air temp comp
void initTempCompSlope(float T1, float comp1_percent, float T2, float comp2_percent, int index) {
	float comp1 = 0.01F * comp1_percent; // turn percentages into fractions of 1
	float comp2 = 0.01F * comp2_percent;
	tempCompCoeff[index].t1 = T1;
	tempCompCoeff[index].t2 = T2;
	tempCompCoeff[index].a = (comp2 - comp1) / (T2 - T1);
	tempCompCoeff[index].b = comp1 - tempCompCoeff[index].a * T1;
}

// provides a temperature compensation value, based on a previously defined gradient & offset
// the value returned is a multiplier, where a value of 1 = neutral compensation and, for example, 1.1 = +10%
float temperatureCompensation(float temp, int tccIndex){
	return 1 + (tempCompCoeff[tccIndex].a * limitF(temp, tempCompCoeff[tccIndex].t1, tempCompCoeff[tccIndex].t2) + tempCompCoeff[tccIndex].b);
}

// find the height of a point in between 4 heights at each corner of a rectangle
// x, y are the distances along width and depth of the edges of the rectangle
// h1-h4 are the heights at each corner

float findHeightInsideRectangle(float x, float y, float widthR, float depthR, float h1, float h2, float h3, float h4) {

    // find the position of x as a proportion of the width
    float xD = x * widthR;

    // find the height of a point between h1 and h2
    float h12 = xD * (h2 - h1) + h1;

    // find the height of a point between h3 and h4
    float h34 = xD * (h4 - h3) + h3;

    // the final height is the partial distance along the line h12 - h34
    return y * depthR * (h34 - h12) + h12;
}

// gets the interpolated value from the specified map
// *** mapLookup() must be called prior to calling this function ***
// getMapInterpolatedValue can find interpolated values from any map for the current RPM and Load values

float getMapInterpolatedValue(float map[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]){

    // find the interpolated value
    return findHeightInsideRectangle(relRPM, relLoad, rpmDeltaReciprocal, loadDeltaReciprocal, map[l1][r1], map[l1][r2], map[l2][r1], map[l2][r2]);
	
}

void mapLookup(float _RPM, float _load){

    // make sure RPM and Load used in this method fall within the absolute limits of the map
    float RPM = limitF(_RPM, cfPage1.p2.rpmAxisStart, cfPage1.p2.rpmAxisStart + (VE_MAP_SIZE_RPM - 1) * cfPage1.p2.rpmAxisDelta);
    float load = limitF(_load, cfPage1.p2.loadAxisStart, cfPage1.p2.loadAxisStart + (VE_MAP_SIZE_LOAD - 1) * cfPage1.p2.loadAxisDelta);

    // partial calc to avoid repeats
    float tempR = (RPM - cfPage1.p2.rpmAxisStart) * rpmDeltaReciprocal;
    float tempL = (load - cfPage1.p2.loadAxisStart) * loadDeltaReciprocal;

    // calculate a map index for RPM and Load
    // The round function selects an index corresponding to the central axis value of the cell
    // e.g. if the RPM axis is defined by start, incr of 750, 700, then the centre values of the RPM 
    // axis are 750, 1450, 2150, 2850, etc and have a range of 400-1100, 1100-1800, 1800-2500, etc.
    // so an RPM value of 1850 will select index 2.
    currentCell.rpmIndex = limitI((int)roundf(tempR), 0, VE_MAP_SIZE_RPM - 1);
    currentCell.loadIndex = limitI((int)roundf(tempL), 0, VE_MAP_SIZE_LOAD - 1);

    // the next part of this method interpolates the map value between map cells

    // select indexes interpreting the axis values as lower bounds rather than central values
    // i.e. using the above example, the axis bands are interpreted as 750, 1450, 2150, 2850, etc
    // so an RPM value of 1850 will select index 1
    int rpmIndex = limitI((int)(tempR), 0, VE_MAP_SIZE_RPM - 1);
    int loadIndex = limitI((int)(tempL), 0, VE_MAP_SIZE_LOAD - 1);

    // find the pattern of cells encompassing the current RPM, Load values
    r1 = rpmIndex <= (VE_MAP_SIZE_RPM - 2) ? rpmIndex : rpmIndex - 1;
    r2 = r1 + 1;

    l1 = loadIndex <= (VE_MAP_SIZE_LOAD - 2) ? loadIndex : loadIndex - 1;
    l2 = l1 + 1;

    // find the rpm & load values relative to the cell
    relRPM = RPM - (cfPage1.p2.rpmAxisStart + r1 * cfPage1.p2.rpmAxisDelta);
    relLoad = load - (cfPage1.p2.loadAxisStart + l1 * cfPage1.p2.loadAxisDelta);

}


/*
 * calculates the injector PW for the current load, rpm cell
 *
 * *** Note that before calling this function, the calling function must call mapLookup(RPM, load) to set the map interpolation variables
 *
 */

float getInjectorPulseWidth(float RPM, float load, float TPS, float engineTemperature, float airTemperature) {

	// Pulse Width in micro-seconds in FP format
	float PWf;
		
	// calculate an adjusted MAP value, adjusted to compensate for acceleration demands
	float adjustedMAP = ( load + accelCompensation1(TPS) ) * 0.01F;
	
	// apply the VE map correction for the current load, rpm cell
	veMapCorrected[currentCell.loadIndex][currentCell.rpmIndex] = cfPage1.veMap[currentCell.loadIndex][currentCell.rpmIndex] + AFRCorrection[currentCell.loadIndex][currentCell.rpmIndex];

	// get the interpolated VE value
	interpolatedVE = getMapInterpolatedValue(veMapCorrected);

	// get the combined temperature compensation value
	tempComp = temperatureCompensation(engineTemperature, 0) * temperatureCompensation(airTemperature, 1);
	
	// calculate PW, depending if engine is running or cranking
	
	if (RPM < cfPage1.p1.crankingThreshold) {
	
		// engine cranking on starter
		
		// reset the PSE value
		PSE = PSEStart;
	
		if (TPS < 60) {
		
			// if throttle opening low, provide PW from cranking PW x 2 x engine temperature comp.
			PWf = cfPage1.p1.crankingPW * (1 + 2 * (temperatureCompensation(engineTemperature, 0) - 1));
			
		}
		else {
		
			// if throttle wide open, provide a tiny PW to help clear a flooded engine
			PWf = 100;
		}
	}
	else {
	
		// normally running engine - calculate the required pulse width (in microseconds) from basic fuel equation
		
		PWf = 1000.0F * (cfPage1.p2.requiredFuel * interpolatedVE * 0.01F * adjustedMAP * tempComp * PSE + cfPage1.p2.injectorLatency);
		
		// apply the decay factor to PSE
		
		PSE = PSE > 1.0F ? PSE - PSEDecay : 1.0F;

	}
	
	// return the required pulse width
	return PWf;

}



void fuInitialise(float cyclicPeriod) {

	// save the reciprocal of rpm and load cell spacing for use in calculations (saves a lenghty divide operation)
	rpmDeltaReciprocal = 1 / cfPage1.p2.rpmAxisDelta;
	loadDeltaReciprocal = 1 / cfPage1.p2.loadAxisDelta;

	// initialise the engine temp - correction slope
	initTempCompSlope(cfPage1.p1.engTempCompT1, cfPage1.p1.engTempCompC1, cfPage1.p1.engTempCompT2, cfPage1.p1.engTempCompC2, 0);

	// initialise the air temperature vs density correction curve
	initTempCompSlope(cfPage1.p1.airTempCompT1, cfPage1.p1.airTempCompC1, cfPage1.p1.airTempCompT2, cfPage1.p1.airTempCompC2, 1);

	// PSIT
	initPostStartEnrichment(cfPage1.p1.pseStartValue, cfPage1.p1.pseDecayTime, cyclicPeriod);

	// accel comp
	initAccelCompensation(cfPage1.p1.accelCompLimit, cfPage1.p1.accelCompAmplitude, cfPage1.p1.accelCompDuration, cyclicPeriod);

	// restore the correction array (assumes the array AFRCorrection has been initialised first by the calling function).
	resetCorrectionArray();
}


/*+++REVISION_HISTORY+++
1) 12 Feb 2021 Changed use of math.h round() to roundf() as this is required for float types.
+++REVISION_HISTORY_ENDS+++*/
