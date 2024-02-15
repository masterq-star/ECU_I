/*

Automatic Air Fuel Ratio (AFR) controller

Provides a Proportional / Integral AFR correction controller. The correction is applied to the AFRCorrection array
in fuel_injection.c. The correction is computed as follows:

Correction{load, RPM} = gainP * error{load, RPM} + gainI * cumulativeError{load, RPM}

Where:

The target & actual AFR are actually lambda sensor voltages in milli-volts, ranging from 0 (lean) to 1000 (rich). 500 is considered to be 14.7 AFR.

gainP & gainI are proportional gain and integral gain respectively.

error{load, RPM} = Target AFR{load, RPM} - Actual AFR{load, RPM}

cumulativeError is the sum of error / 1000 and is limited to 10,000.


Also provides long-term average Lambda voltage and number of samples for each load/RPM cell to aid tuning.

Note:
Previous versions could periodically write & restore the afr data arrays to/from Flash memory. This facility has been
removed as writing to Flash inhibits programme execution, resulting in sync errors and misfires. Later versions
implement NV storage for AFR data using an external EEPROM.

----


This software/firmware source code or executable program is copyright of
Just Technology (North West) Ltd (http://www.just-technology.co.uk) 2020

This software/firmware source code or executable program is provided as free software:
you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version. The license is available at https://www.gnu.org/licenses/gpl-3.0.html

The software/firmware source code or executable program is distributed in the hope that
it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.

*/

#include "auto_afr.h"
#include "math.h"
#include "cfg_data.h"
#include "global.h"
#include "nvm.h"

// save period in number of cycles
static int savePeriod;

// counts the period between saves to NVM
static float savePeriodCounter;

// filter N-1 values
static float filterN_1[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];

// these are used to cycle through the lambda sample array in the getSample method
static int loadIndex1, rpmIndex1;

// a data structure containing the auto AFR arrays
afrDataStruct afrData;

// dataLock set to 1 inhibits updating the average & sample count arrays until a save/restore is completed.
static int dataLock = 0;

// prototypes
void afUpdateCorrectionArray(int loadIndex, int rpmIndex, float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);
void afReComputeCorrections(float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);
HAL_StatusTypeDef afSaveAFRDataToNVM(void);
void afResetAFRNoSave(float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]);


/*
 * afInitialise():
 * 1) sets the long term average Lambda sensor values to the target AFR values
 * 2) zeros the correction, cum error & number of samples arrays
 * 3) resets the averaging low pass filter n-1 values
 * 4) sets the save period in number of cycles and clears the getSample() load & rpm indices.
 *
 * cyclicPeriod is the period of calls to afSaveLambdaData()
 *
 * If an external EEPROM is available, the long-term Lambda sensor averages and number of
 * samples arrays are restored from EEPROM.
 *
 */
void afInitialise(float cyclicPeriod, float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]){

	loadIndex1 = 0;
	rpmIndex1 = 0;

	// Calculate the save period in number of cycles. afrAveragingSavePeriod is in minutes.
	savePeriod = (int)(60000.0F * cfPage1.p1.afrDataSavePeriod / cyclicPeriod);
	savePeriodCounter = 0;

	// reset the AFR data arrays
	afResetAFRNoSave(correctionArray);

	// test if an EEPROM is available
	if (TEST_EEPROM_AVAILABLE == 1) {
		// inhibit updates to the afr data arrays while a restore is in progress
		dataLock = 1;
		// restore the AFR data (averages + sample counts) from EEPROM
		nvEEPROMBlockRead((uint8_t *) &afrData.lambdaAverages[0][0], AFR_DATA_NVM_ADDR, sizeof(afrData.lambdaAverages) + sizeof(afrData.lambdaSamples));
		dataLock = 0;
	}
}


// in response to an "ra#" command from the host, this resets the AFR data arrays
// and updates the saved AFR data held in NVM, if EEPROM is available
HAL_StatusTypeDef afResetAFR(float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]){

	// set the AFR arrays to their initial values
	afResetAFRNoSave(correctionArray);

	HAL_StatusTypeDef result = HAL_OK;

	if (TEST_EEPROM_AVAILABLE == 1) {
		// save the number of samples & average arrays to NVM
		result = afSaveAFRDataToNVM();
	}

	// returns success if no EEPROM available
	return result;
}


/*
 * Sets the NVM average lambda sensor voltages to the target AFR voltages, clears the cumulative error, sets the
 * sample counts to zero, resets the filter N-1 values and sets the correction array to all zeroes
 */
void afResetAFRNoSave(float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]){

	float *avPtr = &afrData.lambdaAverages[0][0];
	float *tgPtr = &cfPage1.targetAFRMap[0][0];
	float *corr = &correctionArray[0][0];
	float *nSmpls = &afrData.lambdaSamples[0][0];
	float *cumErr = &afrData.cumulativeError[0][0];
	float *filt = &filterN_1[0][0];

	for (int i = 0; i <  VE_MAP_SIZE_LOAD * VE_MAP_SIZE_RPM; i++) {
		*avPtr++ = *tgPtr;
		*filt++ = *tgPtr++;
		*corr++ = 0.0F;
		*nSmpls++ = 0;
		*cumErr++ = 0;
	}
}


/*
 * Saves the average Lambda sensor readings and number of samples to NVM when the
 * savePeriodCounter reaches the savePeriod. Returns 1 if a complete data array
 * has been saved. Saving is only done if engine running and warmed-up.
*/
int afSaveAFRData(float RPM, float engineTemp) {

	int saved = 0;
	
	if (TEST_EEPROM_AVAILABLE == 1) {

		if ( (engineTemp > cfPage1.p1.engTempCompT2) && (RPM > cfPage1.p1.crankingThreshold) ) {

			// if the save period is set to zero or less than zero, do nothing
			if (savePeriod > 0) {
				
				// increment the save period counter and compare to save period
				if (++savePeriodCounter >= savePeriod) {

					// reset the counter
					savePeriodCounter = 0;

					// do the save operation
					HAL_StatusTypeDef result = afSaveAFRDataToNVM();

					if (result == HAL_OK){

						// also indicate that a save operation has been completed
						saved = 1;

					}
				}
			}
		}
	}
	return saved;
}


/* Saves the lambda sensor average voltage and number of samples to NVM
 * Before the save operation is initiated, the data lock flag is set to ensure the data arrays are not updated while saving.
 * Returns the success of the operation.
 */
HAL_StatusTypeDef afSaveAFRDataToNVM(){
	dataLock = 1;	// inhibit updates to the AFR data arrays as an update during the save process can result in an incorrect checksum
	HAL_StatusTypeDef res = nvEEPROMBlockWrite((uint8_t*)&afrData.lambdaAverages[0][0], AFR_DATA_NVM_ADDR, sizeof(afrData.lambdaAverages) + sizeof(afrData.lambdaSamples));
	dataLock = 0; 	// allow updates to resume
	return res;
}


/*
 * For the cell defined by rpm and load indices, computes the correction, updates the long term average Lambda voltage
 *
 */
void afComputeCorrection(float RPM, float engineTemp, int loadIndex, int rpmIndex, float lambdaVoltage, float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]){

	// only execute correction calcs if engine is running normally and warmed up
	if ( (engineTemp > cfPage1.p1.engTempCompT2) && (RPM > cfPage1.p1.crankingThreshold) ) {

		// indicate active control over AFR in the ecu status word
		SET_AFR_ACTIVE_CONTROL;
	
		// if saveInProgress is not zero, don't update the average & sample count arrays
		if (dataLock == 0) {

			// calculate a long term average using a low pass (averaging) filter with a very long TC
			afrData.lambdaAverages[loadIndex][rpmIndex] = cfPage1.p1.afrAveragingFilterTC * (lambdaVoltage - filterN_1[loadIndex][rpmIndex]) + filterN_1[loadIndex][rpmIndex];
			filterN_1[loadIndex][rpmIndex] = afrData.lambdaAverages[loadIndex][rpmIndex];

			// increment the number of samples by 0.01
			afrData.lambdaSamples[loadIndex][rpmIndex] += 0.01F;
		}
		
		// calculate the error
		float e = cfPage1.targetAFRMap[loadIndex][rpmIndex] - lambdaVoltage;

		// update cumulative error if not already saturated
		if (fabsf(afrData.cumulativeError[loadIndex][rpmIndex]) < 10000.0F) {
			// integrate 1000th of the proportional error
			afrData.cumulativeError[loadIndex][rpmIndex] += 0.001F * e;
		}

		// calculate final correction
		correctionArray[loadIndex][rpmIndex] = cfPage1.p1.afrCorrectionGainP * e + cfPage1.p1.afrCorrectionGainI * afrData.cumulativeError[loadIndex][rpmIndex];
	}
	else {
		// not controlling AFR
		CLEAR_AFR_ACTIVE_CONTROL;
	}
}


/*
 * Get one sample from the AFR long-term Lambda voltage and correction data array. The samples are
 * provided in sequence until the end of the array, then the sequence is repeated. This is used to transfer
 * the array to the host computer without using up all the bandwidth
 */
void afGetSample(float *correction, float *average, float *samples, float *AFRIndex, float correctionArray[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM]){

	*correction = correctionArray[loadIndex1][rpmIndex1];
	*average = afrData.lambdaAverages[loadIndex1][rpmIndex1];
	*samples = afrData.lambdaSamples[loadIndex1][rpmIndex1];
		
	// encode the indices
	*AFRIndex = (float)(loadIndex1 * VE_MAP_SIZE_RPM + rpmIndex1);

	// increment the indices
	if (++rpmIndex1 >= VE_MAP_SIZE_RPM) {
		rpmIndex1 = 0;
		if (++loadIndex1 >= VE_MAP_SIZE_LOAD) {
			loadIndex1 = 0;
		}
	}	
}


/*+++REVISION_HISTORY+++
1) 05 Jan 2021 NVM Block Read & Write now require an EEPROM on-device address.
2) 10 Jan 2021 Updates to AFR data arrays inhibited while an NVM save or restore is in progress. Updates during save/restore can corrupt the checksum.
3) 12 Feb 2021 Changed use of fabs() to fabsf() as fabsf() works on float types, which is what's needed.
+++REVISION_HISTORY_ENDS+++*/
