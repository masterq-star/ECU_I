

/*

Provides an auxiliary data output on for remote terminals. Transmits a subset of the keyData array for
use in digital instruments and limited diagnostics. Tx rate is faster than host rate to provide
improved update speed for digital instruments.

Transmits the data in a floating point array in ASCII format, where each value is separated by
a comma and the data set starts with a "*" and is terminated by a LF.

On each call to auxSerialTransmit(), one data item is transmitted.

---

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


#include "aux_serial.h"
#include "fuel_injection.h"
#include "global.h"
#include "ecu_services.h"
#include "auto_afr.h"
#include "utility_functions.h"
#include "cfg_data.h"
#include "string.h"
#include "stdio.h"


// the number of data items in a message: include the ecu status word + the number of items sent from the keyData array
#define NUMBER_OF_DATA_ITEMS 20


/*
 * Identifies the data items to be transmitted on aux serial
 *
 */

static float AFRIndex;
static float AFRCorrect;
static float AFRSamples;
static float AFRAveLambda;

static float *reqdData[NUMBER_OF_DATA_ITEMS - 1] = {
		&keyData.v.MAP,						// 0
		&keyData.v.lambdaVoltage,			// 1
		&keyData.v.coolantTemperature,		// 2
		&keyData.v.airTemperature,			// 3
		&keyData.v.TPSVoltage,				// 4
		&keyData.v.voltage2,				// 5
		&keyData.v.RPM,						// 6
		&keyData.v.injectorPW,				// 7
		&keyData.v.TPS,						// 8
		&keyData.v.targetTPS,				// 9
		&keyData.v.thermistorResistance,	// 10
		&keyData.v.currentCell,				// 11
		&keyData.v.errorTooth,				// 12
		&keyData.v.syncErrors,				// 13
		&keyData.v.vvtPwr,					// 14
		&AFRIndex,							// 15
		&AFRSamples,						// 16
		&AFRCorrect,						// 17
		&AFRAveLambda };					// 18


// holds the data decimal precision for string conversion for each of the data items in reqdData
static char dataPrecision[][1] = {
								"1",	// 0 MAP
								"0",	// 1 Lambda
								"1",	// 2 Coolant
								"1",	// 3 Air
								"0",	// 4 TPSV
								"1",	// 5 Volts
								"0",	// 6 RPM
								"0",	// 7 PW
								"1",	// 8 TPS
								"1",	// 9 TTPS
								"0",	// 10 THERM
								"0",	// 11 Current cell
								"0",	// 12 Error Tooth
								"0",	// 13 Sync Error
								"0",	// 14 VVT
								"0",	// 15 AFR Index
								"0",	// 16 AFR Samples
								"1",	// 17 AFR Corrn
								"0"		// 18 AFR Ave
								};

// holds the complete data item to be transmitted
static char itemStr[30] = "";

// default data format for all items - this gets modified by the decimal precision above for each item
static char dataFormat[5] = ",%.1f";

// the message prefix. The 2nd char will contain a single digit representing the current configuration
static char msgPrefix[] = "*0,";


/*
 * Sends one item from the keyData array, formatted in accordance with formatDataMessage() in data_message.c
 */
void auxSerialTransmit(){
 
	// holds the load & rpm indices used to access the AFR correction values & lambda readings
	static int loadIndex1 = 0;
	static int rpmIndex1 = 0;
	// points to the data item due to be transmitted
	static int index = 0;

	char tempStr[15] = "";

	// clear the output string
	itemStr[0] = 0;

	// if first item, send the message prefix and the ecu status word
	// the message prefix now contains the current configuration number
	if (index == 0) {

		msgPrefix[1] = configurationDescriptor.currentConfiguration + '0';
		strcat(itemStr, msgPrefix);
		sprintf(tempStr, "%u", (unsigned int)ecuStatus);
	}
	
	else {

		// not the first item, so send data from the keyData array - use index minus 1 as the first item is the ecu status message
		// modify the data format and convert the data item to string
		dataFormat[3] = dataPrecision[index - 1][0];
		sprintf(tempStr, dataFormat, limitF(*reqdData[index - 1], -9999, 9999));

	}

	// append the data item to the o/p string
	strcat(itemStr, tempStr);

	// test for last item. If so, send a terminator
	if (++index >= NUMBER_OF_DATA_ITEMS) {
		index = 0;
		strcat(itemStr, "\n");

		// also, update the AFR parameters and increment the indices to the next value
		AFRCorrect = AFRCorrection[loadIndex1][rpmIndex1];
		AFRAveLambda = afrData.lambdaAverages[loadIndex1][rpmIndex1];
		AFRSamples = afrData.lambdaSamples[loadIndex1][rpmIndex1];

		// encode the indices
		AFRIndex = loadIndex1 * VE_MAP_SIZE_RPM + rpmIndex1;

		// increment the indices
		if (++rpmIndex1 >= VE_MAP_SIZE_RPM) {
			rpmIndex1 = 0;
			if (++loadIndex1 >= VE_MAP_SIZE_LOAD) {
				loadIndex1 = 0;
			}
		}
	}

	// transmit data item
	auxPrint(itemStr, strlen(itemStr));

} // end transmit

/*+++REVISION_HISTORY+++
1) 11 Jan 2021 Type conversion added since ecuStatus type changed to uint32_t from unsigned int.
2) 10 May 2021 Message prefix now contains the current configuration.
+++REVISION_HISTORY_ENDS+++*/

