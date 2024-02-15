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

#include "data_message.h"
#include "global.h"
#include "utility_functions.h"
#include "stdio.h"
#include "string.h"

// digits after the DP in the data message for each item of key data
// item index							 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
uint8_t dmDADP[KEY_DATA_STRUCT_SIZE] = { 3, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 2, 2, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0 };

// String length for a single converted data item.
// Note this is sized for format +23456.890 - i.e. sign + 5 digits + dp + 3 digits chars + null = 11 in total.
// hence any data item must be limited to +/-99999 in range
// In normal operations, no data item should exceed this limit, unless, for example, the NVM config data hasn't yet been initialised
#define TMP_STR_LEN 20


// provide a separate buffer for NVM messages and data / info messages. They need to be kept separate as an NVM message request can
// be received & formatted while a data message is being sent (& vice-versa), hence possibility for message corruption.
char nvmTxBuffer[NVM_TX_BUFFER_SIZE];
char dataTxBuffer[DATA_TX_BUFFER_SIZE];

// sets the data range for string conversion
static float dataRange = 9999.9F;

// data message and nvm data message identifiers
static char dmPreamble[] = "*";
static char nvPreamble[] = "$";
static char dataFormat[] = ",%.1f";

// used to access data in either float or int format
typedef union {
	uint32_t i;
	float f;
} nvBinaryData;


/*
 * Format the data message. Populates dataTxBuffer with a formatted string consisting
 * of a preamble, the ecu status word, and each of the data items in the supplied data array.
 * Each value is comma separated and the string is terminated with CR LF (char codes 13 & 10) followed by a null char code [0] implicitly inserted by the strcat() function.
 * Every data item is formatted as a floating point value. The digits after the point are individually
 * set, defined by the array dmDADP.
 * The data items are limited to the +/- range specified by dataRange. This is to constrain the length of the string to avoid overflow.
 *
 */
int formatDataMessage(float dataArray[], int nItems) {

	// holds individual item strings
	char tempStr[TMP_STR_LEN];

	// insert the preamable
	strcpy(dataTxBuffer, dmPreamble);

	// insert the ecu status word
	sprintf(tempStr, ",%u", (unsigned int)ecuStatus);
	strcat(dataTxBuffer, tempStr);

	// now format the complete data array
	for (int i=0; i < nItems; i++) {
		// set the number of digits after the point (char 3 in data format)
		dataFormat[3] = dmDADP[i] + 48; // change number into ascii digit
		sprintf(tempStr, dataFormat, limitF(dataArray[i], -dataRange, dataRange));
		strcat(dataTxBuffer, tempStr);
	}
	strcat(dataTxBuffer, "\r\n");
	return strlen(dataTxBuffer);
}

/*
 * Formats an NVM configuration data message, in the form $E:X.Y,N,B,D1,D2,...,Dn, where E is the ECU id number,
 * X.Y is the main and sub firmware version number, N is the number of data items in the message and B is the block ID.
 * D1 to Dn are the data items.
 * The data items are limited to the +/- range specified by dataRange. This is to constrain the length of the string to avoid overflow.
 * The formatted message is sent to nvmTxBuffer.
 * The formatted string is converted according to the data type, defined by the data type arrays in cgf_data.
 * The string is terminated with CR LF (char codes 13 & 10) followed by a null char code [0] implicitly inserted by the strcat() function.
 *
 */
int formatCfgDataMessage(cfBlockID blockID) {

	nvBinaryData d;
	int nItems = 0;
	uint32_t *dataPtr;
	char floatDataFmt[] = ",%.1f";
	char *typePtr;

	switch (blockID) {
	case FILTER_BLK:
		nItems = FILTER_ITEMS;
		dataPtr = (uint32_t*) &cfPage1.filters.mapFilter;
		typePtr = filterDataTypes;
		strcpy(floatDataFmt, ",%.2f");
		break;
	case PARAMETER_1_BLK:
		nItems = PARAMETER_1_ITEMS;
		dataPtr = (uint32_t*) &cfPage1.p1.engTempCompT1;
		typePtr = p1DataTypes;
		strcpy(floatDataFmt, ",%.4f");
		break;
	case PARAMETER_2_BLK:
		nItems = PARAMETER_2_ITEMS;
		dataPtr = (uint32_t*) &cfPage1.p2.ecuID;
		typePtr = p2DataTypes;
		break;
	case VE_MAP_BLK:
		nItems = VE_MAP_ITEMS;
		dataPtr = (uint32_t*) &cfPage1.veMap;
		typePtr = veMapDataTypes;
		break;
	case IGN_MAP_BLK:
		nItems = IGN_MAP_ITEMS;
		dataPtr = (uint32_t*) &cfPage1.ignitionMap;
		typePtr = ignMapDataTypes;
		break;
	case TGT_AFR_BLK:
		nItems = TGT_AFR_ITEMS;
		dataPtr = (uint32_t*) &cfPage1.targetAFRMap;
		typePtr = tgtAFRMapDataTypes;
		strcpy(floatDataFmt, ",%.0f");
		break;

		default:
		// data block not identified, so do nothing
		return 0;
	}

	char tempStr[TMP_STR_LEN];

	strcpy(nvmTxBuffer, nvPreamble);

	sprintf(tempStr, "%i", cfPage1.p2.ecuID);
	strcat(nvmTxBuffer, tempStr);

	sprintf(tempStr, ":%.3f", MAIN_VERSION);
	strcat(nvmTxBuffer, tempStr);

	sprintf(tempStr, ":%i", configurationDescriptor.currentConfiguration);
	strcat(nvmTxBuffer, tempStr);

	sprintf(tempStr, ",%i", nItems);
	strcat(nvmTxBuffer, tempStr);

	sprintf(tempStr, ",%i", blockID);
	strcat(nvmTxBuffer, tempStr);

	for (int i=0; i < nItems; i++) {

		// gets the underlying binary representation of the value
		d.i = *dataPtr++;

		switch ( typePtr[typePtr[0] == '*' ? 1 : i] ) {
		case 'I':
			sprintf(tempStr, ",%i", limitI(d.i, -dataRange, dataRange));
			break;
		case 'F':
		default:
			sprintf(tempStr, floatDataFmt, limitF(d.f, -dataRange, dataRange));
			break;
		}

		strcat(nvmTxBuffer, tempStr);
	}
	strcat(nvmTxBuffer, "\r\n");
	return strlen(nvmTxBuffer);

}


/*+++REVISION_HISTORY+++
1) 06 Jan 2021 enum type cfDataBlockItems used instead of cfDataStats.
2) 08 Jan 2021 Comments updated.
3) 11 Jan 2021 Type conversion added since ecuStatus type changed to uint32_t from unsigned int.
4) 28 Feb 2021 Outputs currently selected configuration in NVM configuration message.
5) 29 Apr 2021 Corrected error in line 116 - was PARAMETER_1_ITEMS, corrected to PARAMETER_2_ITEMS
+++REVISION_HISTORY_ENDS+++*/

