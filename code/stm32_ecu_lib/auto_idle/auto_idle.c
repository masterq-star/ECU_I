
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


#include "auto_idle.h"
#include "math.h"
#include "utility_functions.h"
#include "global.h"
#include "cfg_data.h"
#include "ecu_services.h"


// PID controller variables
float aiTpsError;
float aiErrorDot;
float aiErrorSum;
float aiError_1;
float aiKi;
float aiKd;

// PSIT is the idle throttle adjust value and decays by PSITDecayStep until zero
static float PSIT;
static float PSITDecayStep;

// gradient & offset for the target idle speed vs engine temperature slope
static float gradient;
static float offset;

// adjustment value applied to the target TPS - allows external functions to modify idle speed
float aiTargetTPSAdjust;

// prototypes
float aiGetDemand(float TPS, float targetTPS);


/*
 * Initialise the temperature vs TPS slope and Post-Start Idle Throttle (PSIT) variables
 * PSIT provides a short period increase to the idle throttle to aid starting
 * The start value is the throttle opening required when the engine first starts
 * The start throttle opening reduces to zero over the time period specified
 * The cyclic update period is in milli-seconds
 */
void aiInitialise(float cyclicPeriod) {

	// calculate the PSIT decay step value & save the start value
	PSITDecayStep = cfPage1.p1.psitStartValue * cyclicPeriod / (1000 * cfPage1.p1.psitDecayTime);
	PSIT = cfPage1.p1.psitDecayTime;
	
	// calculate gradient & offset for engine temperature vs idle throttle slope
	gradient = (cfPage1.p1.tpsFastIdleValue - cfPage1.p1.tpsNormalIdleValue) / (cfPage1.p1.tpsFastIdleTemp - cfPage1.p1.tpsNormalIdleTemp);
	offset = cfPage1.p1.tpsNormalIdleValue - gradient * cfPage1.p1.tpsNormalIdleTemp;

	// targetTPSAdjust can be set by an external function (e.g. via command from the host) to manually adjust the Target TPS
	aiTargetTPSAdjust = 0;
	
	// run the initialisation code particular to each type of idle actuator
	switch (cfPage1.p2.idleActuatorType){
	case 1:
		aiType1ActuatorInit();
		break;
	case 2:
		aiType2ActuatorInit();
		break;
	}
}


/*
 * Set the idle actuator output. Calls the actuator function acorrding to the selected actuator type.
 * Returns actuator output.
 *
 */
float aiSetIdleActuator(float TPS, float targetTPS){

//	switch (cfPage1.p2.idleActuatorType){
//		case 1:
//			return aiType1ActuatorSetIdle(TPS, targetTPS);
//		case 2:
//			return aiType2ActuatorSetIdle(TPS, targetTPS);
//	}
	if(TPS == 0 ) setDutyCyclePWM2(50);
	else setDutyCyclePWM2(1);
	
	return 0.0F;
}


/*
 * Gets the target TPS from the temperature vs TPS slope, limited by TPS1 & TPS2 and adjusted by PSIT
 * Returns the target TPS from the slope (+ targetTPSAdjust) or PSIT, whichever is greater.
 *
 */
float aiGetTargetTPS(float engineTemp) {
	
	// determine if engine running
	if (keyData.v.RPM > cfPage1.p1.crankingThreshold) {
		// if so, decrement the post-start throttle adjust until it gets to zero
		if (PSIT > 0) {
			PSIT -= PSITDecayStep;
		}
	}
	else {
		// otherwise reset PSIT to it's start value
		PSIT = cfPage1.p1.psitStartValue;
	}

	// get the target TPS from the temp slope
	float targetTPS = limitF(gradient * engineTemp + offset, cfPage1.p1.tpsNormalIdleValue, cfPage1.p1.tpsFastIdleValue) + aiTargetTPSAdjust;
	
	// return the target TPS or PSIT, whichever is greater
	return fmaxf(targetTPS, PSIT);
}


/* aiGetDemand() uses a simple PID controller with fixed gains for Ki & Kp but a programmable overall gain.

actuator demand = G * (error + Kd * errorDot + Ki * errorSum)

where:
error = targetTPS - TPS
G is overall gain, Kd & Ki are fixed proportions for derivative & integral gain
errorDot is the rate of change of error and errorSum is the cumulative error

*/
float aiGetDemand(float TPS, float targetTPS) {

	// calculate error
	aiTpsError = targetTPS - TPS;

	// calculate cumulative error
	aiErrorSum += aiTpsError;

	// calculate rate of change of error
	aiErrorDot = aiTpsError - aiError_1;
	aiError_1 = aiTpsError;

	return cfPage1.p1.idleActuatorGain * (aiTpsError + aiKi * aiErrorSum + aiKd * aiErrorDot);
}


/*+++REVISION_HISTORY+++
1) 12 Feb 2021 Math.h fmax() works on double but should be float. Changed to fmaxf() for float types.
2) 12 Feb 2021 Call to aiGetDemand() removed from aiSetIdleActuator(). TPS, targetTPS now passed to specific actuator functions.
+++REVISION_HISTORY_ENDS+++*/
