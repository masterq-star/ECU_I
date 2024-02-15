/*
 *
 * Handle a pulse from the crankshaft trigger wheel sensor.
 *
 * F303K8 Measured CPU time: 25 to 30 uS @ 2,300 RPM on trigger wheel simulator. (2nd Dec 2019)
 * G431KB Measured CPU time: 3-5uS @ 8,800 RPM on trigger wheel simulator. (12th Jan 2020)
 *
 *
 *
 *
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


#include "trigger_wheel_handler.h"
#include "ecu_services.h"
#include "cfg_data.h"
#include "utility_functions.h"
#include "global.h"
#include "string.h"
#include <stdio.h>


// prototypes
void injectorPowerOnA(void);
void injectorPowerOffA(void);
void injectorPowerOnB(void);
void injectorPowerOffB(void);
void injectorPowerOnC(void);
void injectorPowerOffC(void);
void injectorPowerOnD(void);
void injectorPowerOffD(void);
void injectorPowerOnALL(void);
void injectorPowerOffALL(void);
void ignitionPowerOff(void);
void twSetInjectionTiming(float PW);
void twSetIgnitionTiming(float advance);

// pulse period, excludes the missing pulse period (uS)
volatile int crankPulsePeriodR = 1E6;

// filtered pulse period, based on crankPulsePeriodR (uS)
volatile int crankPulsePeriodF = 1E6;

static volatile int injectorFiringIndex1 = 0;	// injector firing tooth no. near TDC
static volatile int injectorFiringIndex2 = 18;	// injector firing tooth no. near TDC + 180
static float injectorVernier = 0;				// injectorVernier is set by setInjectionAngle() and defines the time from the injectorFiringIndex,
												// expressed as a fraction of the tooth period. It is used to calculate the absolute timing of the
												// injector opening, based on the measured tooth period.
static volatile int injectorDelay = 1;	 		// the fine timing in uS of the start of injection
static volatile int injectorPW = 2000; 			// Injector pulse width in uS

volatile unsigned int triggerWheelInSync = 0;
volatile int currentTooth = 0;


// trigger wheel configuration variables, set by a call to setTriggerWheelConfig()
int triggerWheelTeethHalf;
float triggerWheelToothSpacingReciprocal;
float rpmToTeethPerMillisecond;
float rpmFromPeriod;

// the index into the injector sequence array
static volatile int injectorIndex = 0;

// injector indexes dedicated to injection timers A & B
static volatile int injectorIndexA, injectorIndexB,injectorIndexC, injectorIndexD;

// injector sequence - holds the injector control pin numbers
static int injectorSequence[NUM_INJECTORS];

// ignition index 1 (near TDC) and index 2 (near TDC+180)
static volatile int ignitionFiringIndex1 = 0;
static volatile int ignitionFiringIndex2 = 0;

// ignition dwell index for near TDC and TDC+180 (defines when coil power is turned on)
static volatile int dwellIndex1 = 0;
static volatile int dwellIndex2 = 0;

// ignition start delay (in uS) provides fine adjustment of the ignition timing
static volatile int ignitionDelay = 1;

// holds the coil pin number for switching power off - this action generates the spark
static volatile int activeCoil;

// defines OFF and ON for ignition coil (polarity can be changed by NVM settings)
GPIO_PinState coilON = GPIO_PIN_SET;
GPIO_PinState coilOFF = GPIO_PIN_RESET;

// this flag is set by the camshaft handler to request an injector sequence reset. Set to non-zero resets the sequence.
volatile int twResetFlag = 0;

// the injector reset index number
static int injectorSequenceReset = 0;
uint8_t check_ig1=0;
uint8_t check_ig2=0;

// A filter time constant (nvmPage1.filters.crankshaftPulseFilter) provides a smoothed pulse period, used
// in next pulse period estimation for detecting a missing pulse. The filter TC is defined as a power of 2 and
// right/left shifting is used in the filter calc instead of multiply & divide.


// handle a crankshaft trigger wheel pulse. the period provided is in micro-seconds (uS)
void crankshaftPulseHandler(int crankPulsePeriod) {

	
//	char buffe[100];
//		
//		sprintf(buffe, "Period is :%d \n", crankPulsePeriod);
//     HAL_UART_Transmit(&huart1,(unsigned char *)buffe, strlen (buffe), 30);
	// for measuring CPU time taken
	#if MEASURE_TW_TASKS == 1
		HAL_GPIO_WritePin(Fan_Control_GPIO_Port, Fan_Control_Pin, GPIO_PIN_SET);
	#endif

	static int crankPulsePeriodFN_1 = 0; 				// period N-1 value for filter (uS)
	static int crankPulsePeriodFtN_1 = 0;				// last filtered value (uS)
	static int crankPulsePeriodEstimate = 0;			// an estimate of the period to the next pulse (uS)
	
	// increment the tooth index
	currentTooth++;	
	
	// if at TDC or TDC + 180, update injection & ignition timing variables
	if ( (currentTooth == cfPage1.p2.twTeeth) || (currentTooth == triggerWheelTeethHalf) ) {
		 twSetInjectionTiming(keyData.v.injectorPW);
		 twSetIgnitionTiming(keyData.v.interpolatedAdvance);
	}


	// if at TDC test, for an injector index reset condition
	if (currentTooth == cfPage1.p2.twTeeth) {
		// condition #1 - if the injector sequence reset value is < 0, set the index to zero
		if (injectorSequenceReset < 0){
			injectorIndex = 0;
		}
		// condition #2 - the injector sequence reset value is not -1, so test for receipt of a camshaft position pulse (i.e. twResetFlag > 0)
		else {
			if (twResetFlag != 0) {
				injectorIndex = injectorSequenceReset;
				twResetFlag = 0;
			}
		}
	}


	// if the pulse period is longer than a missing pulse threshold, then the missing tooth is deemed to have just passed the sensor.
	// the missing pulse threshold is 1.5 x the estimated period

	// test for the missing tooth/pulse AND the current tooth > halfway round the trigger wheel
	if ( (currentTooth > triggerWheelTeethHalf) && (crankPulsePeriod > (crankPulsePeriodEstimate + (crankPulsePeriodEstimate >> 1))) ) {
		
		// missing tooth detected, check to ensure the index is correct
		if (currentTooth != cfPage1.p2.twTeeth) {

			// incorrect index, so record error
			keyData.v.errorTooth = currentTooth;
			keyData.v.syncErrors++;
     
		}
		else {
			// record number of in-sync revolutions
			triggerWheelInSync++;
		}

		// reset the tooth index
		// the missing tooth index is 0, so this tooth must be 1 (for a 36-1 wheel)
		// for a 60-2 wheel, the missing teeth indicies are 0 and 1, so this must be set to 2.
		// i.e. the number of missing teeth
		currentTooth = cfPage1.p2.twMissingTeeth;

	}
	else {
		// not a missing pulse, so capture the pulse period for use in subsequent calcs
		// this measurement excludes the missing pulse period
		crankPulsePeriodR = crankPulsePeriod;
	}
	
	if (triggerWheelInSync > 0) {
	
		// Injection ....
		
		// set up the timing for the injector pulse if the current tooth is at the defined firing index
		// user timer A for index1 (near TDC) injection events
		if ( currentTooth == injectorFiringIndex1 ) {
		
			// check if the engine is running (RPM > cranking threshold)
			// if running, switch on the injector in sequence. Otherwise, switch ALL injectors ON simultaneously
			if(HAL_GPIO_ReadPin(CMP_SIGNAL_CHECK_GPIO_Port,CMP_SIGNAL_CHECK_Pin) == GPIO_PIN_SET){
					if (keyData.v.RPM > cfPage1.p1.crankingThreshold) {
						startInjectionTimerA(injectorDelay, injectorPW, injectorPowerOnA, injectorPowerOffA);
					}
					else {
						startInjectionTimerA(injectorDelay, injectorPW, injectorPowerOnALL, injectorPowerOffALL);
					}
			}
			else{
					if (keyData.v.RPM > cfPage1.p1.crankingThreshold) {
						startInjectionTimerA(injectorDelay, injectorPW, injectorPowerOnD, injectorPowerOffD);
					}
					else {
						startInjectionTimerA(injectorDelay, injectorPW, injectorPowerOnALL, injectorPowerOffALL);
					}
			
			}
			

		}

		// user timer B for index2 (near TDC + 180) injection events
		if ( currentTooth == injectorFiringIndex2 ) {
			if(HAL_GPIO_ReadPin(CMP_SIGNAL_CHECK_GPIO_Port,CMP_SIGNAL_CHECK_Pin) == GPIO_PIN_SET){
					if (keyData.v.RPM > cfPage1.p1.crankingThreshold) {
						startInjectionTimerB(injectorDelay, injectorPW, injectorPowerOnC, injectorPowerOffC);
					}
					else {
						startInjectionTimerB(injectorDelay, injectorPW, injectorPowerOnALL, injectorPowerOffALL);
					}
			}
			else{
					if (keyData.v.RPM > cfPage1.p1.crankingThreshold) {
						startInjectionTimerB(injectorDelay, injectorPW, injectorPowerOnB, injectorPowerOffB);
					}
					else {
						startInjectionTimerB(injectorDelay, injectorPW, injectorPowerOnALL, injectorPowerOffALL);
					}
			
			}
		}
		
		// Ignition ...

		if (currentTooth == dwellIndex1) {
			if(HAL_GPIO_ReadPin(CMP_SIGNAL_CHECK_GPIO_Port,CMP_SIGNAL_CHECK_Pin) == GPIO_PIN_SET)
			// energise COIL A   1
			HAL_GPIO_WritePin(coilIO[0].port, coilIO[0].pin, coilON);
			
			else
				//energise COIL D   4
				HAL_GPIO_WritePin(coilIO[3].port, coilIO[3].pin, coilON);
		}

		if (currentTooth == dwellIndex2) {
			// energise COIL B
			//HAL_GPIO_WritePin(coilIO[1].port, coilIO[1].pin, coilON);
			if(HAL_GPIO_ReadPin(CMP_SIGNAL_CHECK_GPIO_Port,CMP_SIGNAL_CHECK_Pin) == GPIO_PIN_SET)
			// energise COIL C   3
			HAL_GPIO_WritePin(coilIO[2].port, coilIO[2].pin, coilON);
			
			else
				//energise COIL B   2
				HAL_GPIO_WritePin(coilIO[1].port, coilIO[1].pin, coilON);
		}
			
		if (currentTooth == ignitionFiringIndex1) {
			if(HAL_GPIO_ReadPin(CMP_SIGNAL_CHECK_GPIO_Port,CMP_SIGNAL_CHECK_Pin) == GPIO_PIN_SET){
					check_ig1 = ignitionFiringIndex1;
					// trigger COIL A
					activeCoil = 0;
					startIgnitionTimer(ignitionDelay, ignitionPowerOff);
			
			}
			else {
					//check_ig1 = ignitionFiringIndex1;
							// trigger COIL D
							activeCoil = 3;
							startIgnitionTimer(ignitionDelay, ignitionPowerOff);
			
			}
			
		}

		if (currentTooth == ignitionFiringIndex2) {
			
			if(HAL_GPIO_ReadPin(CMP_SIGNAL_CHECK_GPIO_Port,CMP_SIGNAL_CHECK_Pin) == GPIO_PIN_SET){
					check_ig2 = ignitionFiringIndex2;
			// trigger COIL C
			activeCoil = 2;
			startIgnitionTimer(ignitionDelay, ignitionPowerOff);
			
			}
			else {
			//		check_ig2 = ignitionFiringIndex2;
			// trigger COIL B
			activeCoil = 1;
			startIgnitionTimer(ignitionDelay, ignitionPowerOff);
			
			}
		}
		
	} // end if triggerWheelInSync

	// provide a filtered crankshaft pulse period
	// Note the input period is scaled (left shifted) for use in the filter calculation to preserve the fractional part of the calculation
	int crankPulsePeriodFTemp = (((crankPulsePeriodR << cfPage1.filters.crankshaftPulseFilter) - crankPulsePeriodFN_1) >> cfPage1.filters.crankshaftPulseFilter) + crankPulsePeriodFN_1;
	crankPulsePeriodFN_1 = crankPulsePeriodFTemp;
	
	// normalise the filtered pulse period
	crankPulsePeriodF = crankPulsePeriodFTemp >> cfPage1.filters.crankshaftPulseFilter;

	// calculate the next pulse period for missing tooth detection
	// this calculation takes account of accel/decel of the crankshaft		
	crankPulsePeriodEstimate = 2 * crankPulsePeriodF - crankPulsePeriodFtN_1;
	crankPulsePeriodFtN_1 = crankPulsePeriodF; 
	
	#if MEASURE_TW_TASKS == 1
		HAL_GPIO_WritePin(Fan_Control_GPIO_Port, Fan_Control_Pin, GPIO_PIN_RESET);
	#endif	

} // end crankshaftPulse()


// this turns the power off to the specified coil - effectively generates the spark
void ignitionPowerOff(){
	HAL_GPIO_WritePin(coilIO[activeCoil].port, coilIO[activeCoil].pin, coilOFF);
}

// timer A callback - switches an injector ON
void injectorPowerOnA() {
	//HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndex]].port, injectorIO[injectorSequence[injectorIndex]].pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(injectorIO[0].port, injectorIO[0].pin, GPIO_PIN_SET);
	// capture the injector index
	injectorIndexA = injectorIndex;
	// increment the sequence no and reset if overflow
	if (++injectorIndex >= NUM_INJECTORS) {
		injectorIndex = 0;
	}
}

// timer B callback - switches an injector ON
void injectorPowerOnB() {
	//HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndex]].port, injectorIO[injectorSequence[injectorIndex]].pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(injectorIO[1].port, injectorIO[1].pin, GPIO_PIN_SET);
	// capture the injector index
	injectorIndexB = injectorIndex;
	// increment the sequence no and reset if overflow
	if (++injectorIndex >= NUM_INJECTORS) {
		injectorIndex = 0;
	}	
}
// timer c callback - switches an injector ON
void injectorPowerOnC() {
	//HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndex]].port, injectorIO[injectorSequence[injectorIndex]].pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(injectorIO[2].port, injectorIO[2].pin, GPIO_PIN_SET);
	// capture the injector index
	injectorIndexC = injectorIndex;
	// increment the sequence no and reset if overflow
	if (++injectorIndex >= NUM_INJECTORS) {
		injectorIndex = 0;
	}
}

// timer d callback - switches an injector ON
void injectorPowerOnD() {
	//HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndex]].port, injectorIO[injectorSequence[injectorIndex]].pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(injectorIO[3].port, injectorIO[3].pin, GPIO_PIN_SET);
	// capture the injector index
	injectorIndexD= injectorIndex;
	// increment the sequence no and reset if overflow
	if (++injectorIndex >= NUM_INJECTORS) {
		injectorIndex = 0;
	}	
}


// timer A callback - switches off injector specified by injectorIndexA
void injectorPowerOffA() {
	//HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndexA]].port, injectorIO[injectorSequence[injectorIndexA]].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[0].port, injectorIO[0].pin, GPIO_PIN_RESET);
}

// timer B callback - switches off injector specified by injectorIndexB
void injectorPowerOffB() {
	//HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndexB]].port, injectorIO[injectorSequence[injectorIndexB]].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[1].port, injectorIO[1].pin, GPIO_PIN_RESET);
}

// timer C callback - switches off injector specified by injectorIndexA
void injectorPowerOffC() {
	//HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndexC]].port, injectorIO[injectorSequence[injectorIndexC]].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[2].port, injectorIO[2].pin, GPIO_PIN_RESET);
}

// timer D callback - switches off injector specified by injectorIndexB
void injectorPowerOffD() {
  //HAL_GPIO_WritePin(injectorIO[injectorSequence[injectorIndexD]].port, injectorIO[injectorSequence[injectorIndexD]].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[3].port, injectorIO[3].pin, GPIO_PIN_RESET);
}

// switch ALL injectors ON
void injectorPowerOnALL() {
//	startInjectionTimer(injectorPW, injectorPowerOffALL);
	HAL_GPIO_WritePin(injectorIO[0].port, injectorIO[0].pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(injectorIO[1].port, injectorIO[1].pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(injectorIO[2].port, injectorIO[2].pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(injectorIO[3].port, injectorIO[3].pin, GPIO_PIN_SET);
}

// switch ALL injectors OFF
void injectorPowerOffALL() {
	HAL_GPIO_WritePin(injectorIO[0].port, injectorIO[0].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[1].port, injectorIO[1].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[2].port, injectorIO[2].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[3].port, injectorIO[3].pin, GPIO_PIN_RESET);
}


// this function converts an angle (referenced at the first missing tooth) to a tooth index number and the proportional distance between teeth (vernier)
// e.g. for a 36 tooth wheel @ 10 deg spacing, and angle of 42 would give tooth index 4 and a vernier of 0.2
void angleToIndexAndVernier(float angle, int *index, float *vernier) {
	// calculate the tooth number from angle
	*index = angle * triggerWheelToothSpacingReciprocal;
	// calculate the proportional distance to next tooth
	*vernier = angle * triggerWheelToothSpacingReciprocal - *index;
}

// sets the angle relative to TDC for injector power on. The power on, or firing time is defined by a tooth number
// for the TDC and TDC + 180 events and a vernier (time delay) to define the precise angle at a finer resolution than
// can be achieved by just the tooth number.
void setInjectionAngle(float injectorAngle) {
	// calculate the injector firing tooth index and firing vernier
	int tooth;
	angleToIndexAndVernier(cfPage1.p2.twTDCAngle - injectorAngle, &tooth, &injectorVernier);
	injectorFiringIndex1 = tooth;
	injectorFiringIndex2 = injectorFiringIndex1 + triggerWheelTeethHalf;
}


// de-energise the injectors & coils
void injectorPowerReset(){
	HAL_GPIO_WritePin(injectorIO[0].port, injectorIO[0].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[1].port, injectorIO[1].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[2].port, injectorIO[2].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(injectorIO[3].port, injectorIO[3].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(coilIO[0].port, coilIO[0].pin, coilOFF);
	HAL_GPIO_WritePin(coilIO[1].port, coilIO[1].pin, coilOFF);
	HAL_GPIO_WritePin(coilIO[2].port, coilIO[2].pin, coilOFF);
	HAL_GPIO_WritePin(coilIO[3].port, coilIO[3].pin, coilOFF);
}


/*
 * There are 4 injector channels. The channels represent the intake strokes of the engine, and are sequenced as follows:
 *
 * Channel 	Cylinder	Crankshaft Pos
 *    0			1						About 	 TDC
 *    1			3						About		 TDC + 180
 *    2			4						About 	 TDC
 *    3			2						About 	 TDC + 180
 *
 * The injector channels are defined by the array injectorSequence and the injector ready to be fired is pointed to by injectorIndex.
 * The physical injector is mapped onto the channel by NVM settings in Parameters #2. For multiport fully-sequential applications,
 * the injector channel (and physical injector) is synchronised to the cylinder by a camshaft pulse that determines which cylinder is
 * on its intake stroke.
 *
 * The channel can be mapped onto a physical injector/cylinder differently to that shown above but it's important
 * to note that the injector firing is always 180 degrees apart for a normal 4 stroke cycle. From this, we can see that in a
 * single injector application, the injector ON period must always be contained in less than 180 degrees of swept crankshaft angle.
 * For multiport applications, it is possible to open the injector for more than 180 degrees - intake valve open timings are often
 * 200 degrees or more. For this reason two different timers are required, one for TDC events and the other for TDC + 180 events.
 *
 */

// specifies the physical injector for each channel. Injector specified in the range 1 to NUM_INJECTORS.
// the injectorSequenceReset value can be -1 (for non-fully sequential operation) or a value between 1 & NUM_INJECTORS
void setInjectorSequence(int a, int b, int c, int d, int sequenceResetValue){
	// Injector ID stored as range to 0 to NUM_INJECTORS-1
	injectorSequence[0] = limitI(a, 1, NUM_INJECTORS) - 1;
	injectorSequence[1] = limitI(b, 1, NUM_INJECTORS) - 1;
	injectorSequence[2] = limitI(c, 1, NUM_INJECTORS) - 1;
	injectorSequence[3] = limitI(d, 1, NUM_INJECTORS) - 1;
	// if the provided injector sequence reset value is greater than zero, set the internal value to it but within the limits of the injector sequence array size
	// if less than zero, set the internal value to the same value
	injectorSequenceReset = sequenceResetValue > 0 ? limitI(sequenceResetValue, 1, NUM_INJECTORS) - 1 : sequenceResetValue;
}

void setTriggerWheelConfig(){

	triggerWheelTeethHalf = cfPage1.p2.twTeeth / 2;

	triggerWheelToothSpacingReciprocal = ((float) cfPage1.p2.twTeeth) / 360.0F;

	// converts RPM to Teeth / milli-second - used to calculate number of dwell teeth for the dwell time
	rpmToTeethPerMillisecond = ((float) cfPage1.p2.twTeeth) / 60000.0F;

	// used to convert the pulse period in microseconds to RPM
	rpmFromPeriod = 60000000.0F / ((float) cfPage1.p2.twTeeth);

}

void twInitialise() {

	// power everything off
	injectorPowerReset();

	setInjectorSequence(cfPage1.p2.injectorIndex0, cfPage1.p2.injectorIndex1, cfPage1.p2.injectorIndex2, cfPage1.p2.injectorIndex3, cfPage1.p2.injectorSequenceReset);
	setTriggerWheelConfig();
	setInjectionAngle(cfPage1.p2.injectorStartAngle);

	// Set the firing sense for the ignition coils
	// Note that a high output (SET) from the CPU turns the output transistor ON, a low output (RESET) turns the output transistor OFF.
	// "Normal" coils need the power transistor ON during the dwell period and OFF to generate the spark.
	// Electronic coils with built-in power amplifiers usually need the signal inverted.
	if (cfPage1.p2.ignitionFiringSense > 0) {
		// normal coil
		coilON = GPIO_PIN_SET;
		coilOFF = GPIO_PIN_RESET;
	}
	else {
		// electronic coil, inverted ouput
		coilON = GPIO_PIN_RESET;
		coilOFF = GPIO_PIN_SET;
	}
	keyData.v.errorTooth = 0;
	keyData.v.syncErrors = 0;
}

// Sets the injection timing.
void twSetInjectionTiming(float PW){
	// set the injector pulse width
	injectorPW = PW;
	// calculate the injector delay in uS, using the vernier adjustment
	injectorDelay = crankPulsePeriodF * injectorVernier;
}


// sets the ignition timing
void twSetIgnitionTiming(float advance){

	// convert the ignition angle into a tooth index and vernier
	float ignitionVernier;
	int ignitionTooth;
	angleToIndexAndVernier(cfPage1.p2.twTDCAngle - advance, &ignitionTooth, &ignitionVernier);
	ignitionFiringIndex1 = ignitionTooth;
	ignitionFiringIndex2 = ignitionFiringIndex1 + triggerWheelTeethHalf;

	// convert the ignition vernier to a time delay in microseconds
	ignitionDelay = crankPulsePeriodF * ignitionVernier;

	// calculate tooth indexes required to achieve the specified dwell (power on time)
	int dwellTeeth = keyData.v.RPM * rpmToTeethPerMillisecond * cfPage1.p2.ignitionDwell;

	if (dwellTeeth < 1) {
		// must be at least 1
		dwellTeeth = 1;
	}

	// calculate dwell index
	int dwellIndex = ignitionFiringIndex1 - dwellTeeth;

	if (dwellIndex < 0) {
		// can't be negative
		dwellIndex += cfPage1.p2.twTeeth;
	}

	// update working variables (making sure its not zero as there's no event at zero - it's the missing tooth)
	// if zero, set to the event before the missing tooth, e.g. 35 for a 36 tooth wheel
	dwellIndex1 = dwellIndex != 0 ? dwellIndex : cfPage1.p2.twTeeth - 1;

	// calculate dwell index 2
	dwellIndex += triggerWheelTeethHalf;

	if (dwellIndex >= cfPage1.p2.twTeeth) {
		// make sure it doesn't overflow
		dwellIndex -= cfPage1.p2.twTeeth;
	}

	// copy across, making sure its not zero
	dwellIndex2 = dwellIndex != 0 ? dwellIndex : cfPage1.p2.twTeeth - 1;
}


/*+++REVISION_HISTORY+++
1) 11 May 2021 Included "global.h"
2) 19 May 2021 Fixes error in setTriggerWheelConfig() - parameters are no longer required.
+++REVISION_HISTORY_ENDS+++*/
