 /*

Decodes & executes command messages received from the host.

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


#include "command_decoder.h"
#include "data_message.h"
#include "ecu_services.h"
#include "sensors.h"
#include "nvm.h"
#include "auto_idle.h"
#include "fuel_injection.h"
#include "auto_afr.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
//	keyData.v.TPSVoltage = 500.0F;
//	keyData.v.TPS = 20.0F;
//	keyData.v.MAP = 80.0F;
//	keyData.v.lambdaVoltage = 750.0F;
//	keyData.v.airTemperature = 25.0F;
//	keyData.v.coolantTemperature = 10.0F;
//	keyData.v.voltage2 = 0.0F;
// command identifiers
char TARGET_IDLE_CMD[] 		= "tt";
char SEND_ID_CMD[]			= "si";
char SEND_DATA_CMD[]		= "sd";
char SEND_SYNC_CMD[]		= "sy";
char SET_CONFIG_CMD[]		= "sc";
char SEND_NVM_CMD[]			= "sn";
char WRITE_FORMAT_CMD[]		= "wf";
char RESET_AVE_CMD[]		= "ra";
char SET_LAMBDA[] = "sl";
char SET_AIR_TEMP[] = "sa";
char SET_COOLANT[] = "so";
char SET_MAP[] = "sm";
char SET_TPS[] = "st";
// error & status messages
char NVM_DATA_ERROR_MSG[]			= ">NVM: Error in number of data items received\r\n";
char NVM_DATA_ERROR_MSG2[]			= ">NVM: Number of data items does not match data block\r\n";
char NVM_CHECKSUM_ERROR_MSG[] 		= ">NVM: Error in checksum\r\n";
char NVM_UNKNOWN_BLK_ERROR_MSG[] 	= ">NVM: Data Block ID invalid\r\n";
char NVM_SUCCESS_FI_MSG[] 			= ">NVM: FILTER written successfully\r\n";
char NVM_SUCCESS_P1_MSG[] 			= ">NVM: PAR 1 written successfully\r\n";
char NVM_SUCCESS_P2_MSG[] 			= ">NVM: PAR 2 written successfully\r\n";
char NVM_SUCCESS_VE_MSG[] 			= ">NVM: VE MAP written successfully\r\n";
char NVM_SUCCESS_IG_MSG[] 			= ">NVM: IG MAP written successfully\r\n";
char NVM_SUCCESS_TA_MSG[] 			= ">NVM: TGT AFR written successfully\r\n";
char NVM_SUCCESS_MSG[] 				= ">NVM: Data written successfully\r\n";
char NVM_ERASE_ERROR_MSG[] 			= ">NVM: Page erase error\r\n";
char NVM_WRITE_ERROR_MSG[] 			= ">NVM: Page write error\r\n";
char NVM_CONFIG_INVALID[]			= " | NVM CONFIGURATION DATA INVALID";
char CURRENT_CONFIG_MSG[]			= ", selected configuration ";
char LAMBDA_AFR_RESET_MSG[] 		= ">Lambda Sensor AFR reset success\r\n";
char LAMBDA_AFR_RESET_FAILED_MSG[]	= ">Lambda Sensor AFR data failed to store data to NVM\r\n";
char EFI_VERSION_MSG[]				= ">EFI Controller, stm32 MPU: ";
char SENSORS_DISABLED_MSG[]			= " | SENSORS DISABLED";
char SYNC_MSG[] 					= "<\r\n";
char CRLF[]							= "\r\n";

// prototypes
int stringStartsWith(char str[], char compare[]);
void sendIdentificationMessage(void);
int getParameters(char cmdLine[], int cmdLineLength, paramType data[], int maxParams);
int findSeparator(char str[], int strlen, int startFrom);
int checksumNVMData(paramType *data, int length, float expectedChecksum);


// All commands are terminated with a "#" and start with a two character code

void cdExecuteCommand(char cmd[], int length) {

	// local parameter storage
	#define MAX_NUM_PARAMETERS 70
	paramType dataParams[MAX_NUM_PARAMETERS];
	
	// TARGET_IDLE_CMD
	// e.g. tt-0.5# - reduces the target idle by 0.5%
	
	if (stringStartsWith(cmd, TARGET_IDLE_CMD) > 0) {
		getParameters(cmd, length, dataParams, 1);
		aiTargetTPSAdjust += dataParams[0].f;
		return;
	}

	
	// SEND_SYNC_CMD Send a sync message

	if (stringStartsWith(cmd, SEND_SYNC_CMD) > 0) {
		// the sync message is a single open bracket
		hostPrint(SYNC_MSG, sizeof(SYNC_MSG));
		return;
	}


	// SEND_ID_CMD Send Identification Messages
	
	if (stringStartsWith(cmd, SEND_ID_CMD) > 0) {
		sendIdentificationMessage();
		return;
	}
	if(stringStartsWith(cmd, SET_LAMBDA) > 0){
	getParameters(cmd, length, dataParams, 1);
	keyData.v.lambdaVoltage = dataParams[0].f *10;
		return;
	
	}
	if(stringStartsWith(cmd, SET_AIR_TEMP) > 0){
	getParameters(cmd, length, dataParams, 1);
	keyData.v.airTemperature = dataParams[0].f;
	return;
	}
	if(stringStartsWith(cmd, SET_COOLANT) > 0){
	getParameters(cmd, length, dataParams, 1);
	keyData.v.coolantTemperature = dataParams[0].f;
		return;
	}
	if(stringStartsWith(cmd, SET_MAP) > 0){
	getParameters(cmd, length, dataParams, 1);
	keyData.v.MAP = dataParams[0].i;
		return;
	}
	if(stringStartsWith(cmd, SET_TPS) > 0){
	getParameters(cmd, length, dataParams, 1);
	keyData.v.TPS = dataParams[0].f;
		return;
	}
	// SET_CONFIG_CMD Set the configuration
	// e.g. sc<N>#
	
	if (stringStartsWith(cmd, SET_CONFIG_CMD) > 0) {
		getParameters(cmd, length, dataParams, 1);
		cfSetCurrentConfig(dataParams[0].i);
		return;
	}
	

	// SEND_DATA_CMD Send current data to the host
	
	if (stringStartsWith(cmd, SEND_DATA_CMD) > 0) {
		// update the AFR values in the key data array
		afGetSample(&keyData.v.AFRCorrection, &keyData.v.lambdaVoltageAverage, &keyData.v.lambdaVoltageSamples, &keyData.v.AFRIndex, AFRCorrection);
		// get & format the data to be sent to the terminal
		int sz = formatDataMessage(keyData.dataArray, KEY_DATA_STRUCT_SIZE);
		hostPrint(dataTxBuffer, sz);
		return;
	}

	
	// WRITE_FORMAT_CMD Write configuration data format to non-volatile memory
	// the 1st parameter must be an integer that specifies the id of the data block
	// the 2nd parameter is the number of data items to be written
	// the 3rd parameter is the checksum of the data - simply the sum of all the data items
	// the 4th parameter onwards is the floating point data items
	// e.g. wf0,2,300.5,100.5,200.0#
	
	if (stringStartsWith(cmd, WRITE_FORMAT_CMD) > 0) {
	
		// get the maximum number of parameters
		int n = getParameters(cmd, length, dataParams, MAX_NUM_PARAMETERS);
		
		// the first 3 params are id, number of data items and checksum
		cfBlockID block = dataParams[0].i;		// the id of the NVM block to be modified
		int nItems = dataParams[1].i;			// the number of NVM data items to be written to the store
		float checksum = dataParams[2].f;		// the checksum
		
		// if the number of parameters obtained - 3 does not match the number of data items, then flag an error
		if ((n - 3) != nItems) {
			hostPrint(NVM_DATA_ERROR_MSG, sizeof(NVM_DATA_ERROR_MSG));
			return;
		}
		else
			// if the checksums don't match, then flag an error
			if (checksumNVMData(&dataParams[3], nItems, checksum) != 0) {
				hostPrint(NVM_CHECKSUM_ERROR_MSG, sizeof(NVM_CHECKSUM_ERROR_MSG));
				return;
			}
		
		// write data parameters to NVM. The NVM data starts at index 3 in dataParams
		cfErrorCode result = cfProcessNVMMessage(block, nItems, &dataParams[3]);

		switch (result) {
		case CF_DATA_SIZE_MISMATCH:
			hostPrint(NVM_DATA_ERROR_MSG2, sizeof(NVM_DATA_ERROR_MSG2));
			break;
		case CF_UNKNOWN_BLOCK_ID:
			hostPrint(NVM_UNKNOWN_BLK_ERROR_MSG, sizeof(NVM_UNKNOWN_BLK_ERROR_MSG));
			break;
		case CF_ERASE_ERROR:
			hostPrint(NVM_ERASE_ERROR_MSG, sizeof(NVM_ERASE_ERROR_MSG));
			break;
		case CF_WRITE_ERROR:
			hostPrint(NVM_WRITE_ERROR_MSG, sizeof(NVM_WRITE_ERROR_MSG));
			break;
		case CF_SUCCESS:
		default:
			// send success message
			switch (block){
			case FILTER_BLK:
				hostPrint(NVM_SUCCESS_FI_MSG, sizeof(NVM_SUCCESS_FI_MSG));
				break;
			case PARAMETER_1_BLK:
				hostPrint(NVM_SUCCESS_P1_MSG, sizeof(NVM_SUCCESS_P1_MSG));
				break;
			case PARAMETER_2_BLK:
				hostPrint(NVM_SUCCESS_P2_MSG, sizeof(NVM_SUCCESS_P2_MSG));
				break;
			case VE_MAP_BLK:
				hostPrint(NVM_SUCCESS_VE_MSG, sizeof(NVM_SUCCESS_VE_MSG));
				break;
			case IGN_MAP_BLK:
				hostPrint(NVM_SUCCESS_IG_MSG, sizeof(NVM_SUCCESS_IG_MSG));
				break;
			case TGT_AFR_BLK:
				hostPrint(NVM_SUCCESS_TA_MSG, sizeof(NVM_SUCCESS_TA_MSG));
				break;
			}
		}
		return;
	}

	
	// SEND_NVM_CMD Send nvm memory data to host
	// the 1st parameter is the start index of the NVM block
	// the 2nd parameter is the number of values to send
	// e.g. sn0,12#
	
	// number of values is now ignored - the number of items is now derived from the sized of the data block

	if (stringStartsWith(cmd, SEND_NVM_CMD) > 0) {
		// read page1 NVM data back into the data structures
		//cfRestoreConfiguration();
		// get the block id from the command parameter parameter
		getParameters(cmd, length, dataParams, 2);
		// format the data block & send to host
		int sz = formatCfgDataMessage(dataParams[0].i);
		hostPrint(nvmTxBuffer, sz);
		return;
	}


	// RESET_AVE_CMD Reset AFR system

	if (stringStartsWith(cmd, RESET_AVE_CMD) > 0) {
		if (afResetAFR(AFRCorrection) == HAL_OK) {
			hostPrint(LAMBDA_AFR_RESET_MSG, sizeof(LAMBDA_AFR_RESET_MSG));
		}
		else {
			hostPrint(LAMBDA_AFR_RESET_FAILED_MSG, sizeof(LAMBDA_AFR_RESET_FAILED_MSG));
		}
		return;
	}

	// no command found
	return;

}


/*
 * Gets the comma separated parameters in a command line into an array of floating point
 * and integer values. Both floats and integers are needed to suit the destination data type.
 * Returns the number of parameters found up to the specified maximum number.
 *
 */

int getParameters(char cmdLine[], int cmdLineLength, paramType *pArray, int maxParams) {

	int paramCount = 0;
    int from = 2; 					// start at 3rd character to omit the 2 char code

	while (from < cmdLineLength) {
		sscanf(&cmdLine[from], "%f", &pArray->f);
		sscanf(&cmdLine[from], "%i", &pArray->i);
		pArray++;
		paramCount++;
		if (paramCount < maxParams){
			int sep = findSeparator(cmdLine, cmdLineLength, from);
			if (sep != -1) {
				// there's another value, so move index up to that point
				from = sep + 1;
			}
			else {
				break;
			}
		}
		else {
			break;
		}
	}
	return paramCount;
}


// checks the data contained in a comma-separated string for the expected checksum and data count.
// returns 0 if checksums match, non-zero otherwise.
int checksumNVMData(paramType data[], int length, float expectedChecksum) {
	float checksum = 0;
	for (int i = 0; i < length; i++){
		checksum += data[i].f;
	}
	if (fabs(checksum - expectedChecksum) > 0.05) {
		// checksums don't match
		return 1;
	}
	else {
		return 0;
	}
}


// send the identification and config info. messages
void sendIdentificationMessage() {
	char tempStr[20];
	strcpy(dataTxBuffer, EFI_VERSION_MSG);
	sprintf(tempStr, "%.3f ", MAIN_VERSION);
	strcat(dataTxBuffer, tempStr);
	strcat(dataTxBuffer, VERSION_DATE);
	strcat(dataTxBuffer, CURRENT_CONFIG_MSG);
	sprintf(tempStr, "%i", configurationDescriptor.currentConfiguration);
	strcat(dataTxBuffer, tempStr);
	if ( (ecuStatus & INVALID_CONFIG) != 0){
		strcat(dataTxBuffer, NVM_CONFIG_INVALID);
	}
	if (sensorsDisabled != 0) {
		strcat(dataTxBuffer, SENSORS_DISABLED_MSG);
	}
	strcat(dataTxBuffer, CRLF);
	hostPrint(dataTxBuffer, strlen(dataTxBuffer));
}


int stringStartsWith(char str[], char compare[]){
	if ( (str[0] == compare[0]) && (str[1] == compare[1]) ){
		return 1;
	}
	else {
		return 0;
	}
}


int findSeparator(char *str, int strlen, int startFrom){
	int sepPos = -1;
	for (int i = startFrom; i < strlen; i++){
		if (*(str + i) == ',') {
			sepPos = i;
			break;
		}
	}
	return sepPos;
}


/*+++REVISION_HISTORY+++
1) 09 Jan 2021 The sn command sends data from the requested data block in RAM. No longer reads the configuration data from NVM.
2) 09 Jan 2021 NVM write success message explicitly identifies which block.
3) 11 Jan 2021 EEPROM Available and I2C Enabled messages deleted. Reports invalid config if INVALID_CONFIG is set in ecu status word.
4) 02 Mar 2021 SEND_SYNC command deleted. SET_CONFIG_CMD added. Identification message now includes current configuration parameter.
5) 29 Apr 2021 SEND_SYNC command re-instated.
6) 02 May 2021 sendIdentificationMessage() modified to send only one line. From now on, all ECU commands must only return a one line response (if any).
+++REVISION_HISTORY_ENDS+++*/
