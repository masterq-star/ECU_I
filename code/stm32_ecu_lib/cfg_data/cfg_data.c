/*
 * Defines the ECU's configuration data and provides utilities to store to & retrieve from Non-Volatile Memory (NVM).
 * In this version, the NVM utilised is an external EEPROM accessed via the nvm package.
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


#include "cfg_data.h"
#include "ecu_main.h"
#include "fuel_injection.h"
#include "auto_afr.h"
#include "trigger_wheel_handler.h"
#include "auto_idle.h"
#include "sensors.h"
#include "vvt_controller.h"
#include "nvm.h"

// prototypes
static uint16_t absAddr(int relAddr);


// NVM space has been allocated for up to 8 different configurations
// The following data block provides information about the currently selected configuration.
// The currently selected configuration is specified by a single integer in the range 1 to 8.
configurationDesciptorStruct configurationDescriptor = {.currentConfiguration = 1,
														.unused = {1,2,3,4,5,6,7,8,9,10,11,12,13,14}};

// The ECU configuration data is currently contained in one "page".
// Default values taken from Mini Nissan CR14DE ECU February 2021
page1Struct cfPage1 = {
	.filters = 		{0.5F, 0.5F, 0.01F, 0.1F, 0.5F, 0.01F, 0, 3},
	.p1 = 			{0.0F,10.0F,50.0F,-5.0F,0.0F,7.3F,60.0F,-11.5F,0.0F,1000.0F,100.0F,4000.0F,500.0F,6000.0F,7.5F,0.50F,500.0F,88.0F,10.0F,5.0F,0.0F,50.0F,3.0F,0.0F,0.0F,0.0F,0.010F,0.00100F,0.0005F,10.0F,30.0F,15.0F,6.0F,15.0F},
	.p2 = 			{32,8,8,750.0F,700.0F,30.0F,10.0F,6.10F,0.5F,-1,4.0F,36,1,138.0F,15.0F,1,2,1,2,-1,7.0F,2800.0F,100.0F,180.0F,3248.0F,628.0F,2},
	.ignitionMap =	{	{2.0F,5.0F,22.5F,28.8F,29.8F,30.8F,31.8F,32.8F},
						{5.0F,10.0F,21.7F,27.5F,28.7F,30.0F,31.2F,32.4F},
						{10.0F,15.4F,20.8F,26.3F,27.7F,29.1F,30.6F,32.0F},
						{10.0F,15.0F,20.0F,25.0F,26.7F,28.3F,30.0F,31.6F},
						{10.0F,14.6F,19.2F,23.8F,25.6F,27.5F,29.3F,31.2F},
						{10.0F,14.2F,18.3F,22.5F,24.6F,26.7F,28.7F,30.8F},
						{10.0F,13.8F,17.5F,21.3F,23.5F,25.8F,28.1F,30.4F},
						{10.0F,13.3F,16.7F,20.0F,22.5F,25.0F,27.5F,30.0F}},
	.veMap =		{	{43.9F,45.0F,45.5F,46.7F,45.4F,41.6F,32.9F,30.7F},
						{48.0F,49.1F,50.9F,53.9F,57.9F,57.8F,47.6F,43.4F},
						{52.0F,55.9F,57.2F,59.8F,62.4F,64.9F,60.4F,57.5F},
						{58.0F,63.3F,61.8F,63.8F,65.5F,69.4F,68.3F,64.8F},
						{65.0F,67.5F,65.6F,67.3F,71.1F,75.1F,74.5F,70.0F},
						{70.0F,72.9F,71.6F,72.1F,74.0F,78.4F,79.4F,77.8F},
						{75.0F,76.1F,74.7F,75.1F,77.1F,83.4F,86.6F,86.4F},
						{80.7F,80.4F,80.1F,80.6F,81.6F,87.2F,90.3F,89.4F}},
	.targetAFRMap =	{	{480.0F,480.0F,480.0F,480.0F,480.0F,480.0F,480.0F,480.0F},
						{482.0F,484.0F,486.0F,488.0F,490.0F,492.0F,494.0F,497.0F},
						{484.0F,488.0F,492.0F,496.0F,500.0F,504.0F,508.0F,514.0F},
						{486.0F,492.0F,498.0F,504.0F,510.0F,516.0F,522.0F,531.0F},
						{488.0F,496.0F,504.0F,512.0F,520.0F,528.0F,536.0F,548.0F},
						{490.0F,500.0F,510.0F,520.0F,530.0F,540.0F,550.0F,565.0F},
						{492.0F,504.0F,516.0F,528.0F,540.0F,552.0F,564.0F,582.0F},
						{500.0F,514.0F,528.0F,542.0F,556.0F,570.0F,584.0F,600.0F}}};

// Specifies data types for the configuration data.
// '*' in the first character position means every item has the same type,
// and the 2nd character specifies the type.
char filterDataTypes[] = "FFFFFFII";
char p1DataTypes[] = "*F";
char p2DataTypes[] = "IIIFFFFFFIFIIFFIIIIIFFFFFFI";
char veMapDataTypes[] = "*F";
char ignMapDataTypes[] = "*F";
char tgtAFRMapDataTypes[] = "*F";


// used to access data in either float or int format
typedef union {
	uint32_t i;
	float f;
} nvBinaryData;


// Restore the configuration data from the NVM device. The configuration page restored is determined by
// the currentConfiguration parameter, held in the configurationDescriptor structure at EEPROM address 0. This is restored first.
// If a valid currentConfiguration cannot be restored, no further action is taken and a result of 1 is returned.
// If an EEPROM read or checksum error is set in the ecu status word, then the INVALID_CONFIG bit in the ecu status word is set and a result of 2 is returned.
// If the configuration was restored with no errors, 0 is returned.
// Note that the ecu status word should be cleared before calling this function.

int cfRestoreConfiguration(){

	int result = 0;

	// restore the configuration descriptor data block
	nvEEPROMBlockRead((uint8_t *)&configurationDescriptor, 0, sizeof(configurationDescriptor));
	if ( ((ecuStatus & EEPROM_DATA_READ_ERROR) != 0) || ((ecuStatus & EEPROM_CHECKSUM_ERROR) != 0) || (configurationDescriptor.currentConfiguration < 1) || (configurationDescriptor.currentConfiguration > 8)){
		SET_INVALID_CONFIG;
		result = 1;
		return result;
	}

	// restore each configuration data block
	nvEEPROMBlockRead((uint8_t *)&cfPage1.filters, 		absAddr(FILTERS_NVM_ADDR),		sizeof(cfPage1.filters));
	nvEEPROMBlockRead((uint8_t *)&cfPage1.p1, 			absAddr(PARAMETERS_1_NVM_ADDR),	sizeof(cfPage1.p1));
	nvEEPROMBlockRead((uint8_t *)&cfPage1.p2, 			absAddr(PARAMETERS_2_NVM_ADDR),	sizeof(cfPage1.p2));
	nvEEPROMBlockRead((uint8_t *)&cfPage1.veMap, 		absAddr(VE_MAP_NVM_ADDR),		sizeof(cfPage1.veMap));
	nvEEPROMBlockRead((uint8_t *)&cfPage1.ignitionMap, 	absAddr(IGNITION_MAP_NVM_ADDR),	sizeof(cfPage1.ignitionMap));
	nvEEPROMBlockRead((uint8_t *)&cfPage1.targetAFRMap, absAddr(TGT_AFR_MAP_NVM_ADDR),	sizeof(cfPage1.targetAFRMap));

	// check for errors
	if ( ((ecuStatus & EEPROM_DATA_READ_ERROR) != 0) || ((ecuStatus & EEPROM_CHECKSUM_ERROR) != 0) ){
		SET_INVALID_CONFIG;
		result = 2;
	}
	return result;
}


// updates a data block within configuration data from data received from the host via a "wf" command
// data from the "wf" command is held in float and integer formats and only the required data type, defined by
// the array typeStr, is copied to the data set.
void copyTypedData(uint32_t *blkPtr, paramType data[], int n, char typeStr[]){
	nvBinaryData d;
	for (int i=0; i < n; i++){
		switch ( typeStr[typeStr[0] == '*' ? 1 : i] ) {
		case 'I':
			*blkPtr++ = data[i].i;
			break;
		case 'F':
		default:
			d.f = data[i].f; // use a union to access the binary representation of the float
			*blkPtr++ = d.i; // this is needed to copy the binary representation, without conversion to int
			break;
		}
	}
}


/*

cfsaveConfig()

 1) Updates the specified data block within the configuration data structure
 2) Then attempts to write the supplied configuration data block to EEPROM.

 *blkPtr is a pointer to the first byte in the data block
 blkSize is the size of the block in bytes
 newDataItems is an array of data obtained from the host
 nItemsSupplied is the number of data items obtained from the host
 itemsExpected is the number of items expected in the data block
 eepromAddress is the on-device address of the data block
 dataTypes defines the data type of each data item

 */

cfErrorCode cfSaveConfig(uint32_t *blkPtr, int blkSize, paramType newDataItems[], int nItemsSupplied, int itemsExpected, uint16_t eepromAddress, char dataTypes[]){

	cfErrorCode statusFlag = CF_SUCCESS;

	// check if the number of data items supplied is the same as the number of items in the data block
	if (nItemsSupplied == itemsExpected){

		// size matches, so update the data block
		copyTypedData(blkPtr, newDataItems, nItemsSupplied, dataTypes);

		// save to NVM
		if (nvEEPROMBlockWrite((uint8_t *) blkPtr, eepromAddress, blkSize) != HAL_OK){
			statusFlag = CF_WRITE_ERROR;
		}

	}
	else {
		statusFlag = CF_DATA_SIZE_MISMATCH;
	}
	return statusFlag;
}



/*
 * Process a write to NVM (wf#) command message from the host computer.
 * block is the block ID (same as the EEPROM address in this case).
 * nItems is the number of items supplied from the host computer - use this as part of the validity checking procedure.
 * dataItems is the data in both float and int types to suit the configuration data type in each of the data structures.
 */
cfErrorCode cfProcessNVMMessage(cfBlockID block, int nItems, paramType *data){
	cfErrorCode status = CF_SUCCESS;
	switch (block) {
	case FILTER_BLK:
		status = cfSaveConfig((uint32_t *)&cfPage1.filters, sizeof(cfPage1.filters), data, nItems, FILTER_ITEMS, absAddr(FILTERS_NVM_ADDR), filterDataTypes);
		cfSoftwareResetFilters();
		break;
	case PARAMETER_1_BLK:
		status = cfSaveConfig((uint32_t *)&cfPage1.p1, sizeof(cfPage1.p1), data, nItems, PARAMETER_1_ITEMS, absAddr(PARAMETERS_1_NVM_ADDR), p1DataTypes);
		cfSoftwareReset();
		break;
	case PARAMETER_2_BLK:
		status = cfSaveConfig((uint32_t *)&cfPage1.p2, sizeof(cfPage1.p2), data, nItems, PARAMETER_2_ITEMS, absAddr(PARAMETERS_2_NVM_ADDR), p2DataTypes);
		cfSoftwareReset();
		break;
	case VE_MAP_BLK:
		status = cfSaveConfig((uint32_t *)&cfPage1.veMap, sizeof(cfPage1.veMap), data, nItems, VE_MAP_ITEMS, absAddr(VE_MAP_NVM_ADDR), veMapDataTypes);
		cfSoftwareResetMaps();
		break;
	case IGN_MAP_BLK:
		status = cfSaveConfig((uint32_t *)&cfPage1.ignitionMap, sizeof(cfPage1.ignitionMap), data, nItems, IGN_MAP_ITEMS, absAddr(IGNITION_MAP_NVM_ADDR), ignMapDataTypes);
		break;
	case TGT_AFR_BLK:
		status = cfSaveConfig((uint32_t *)&cfPage1.targetAFRMap, sizeof(cfPage1.targetAFRMap), data, nItems, TGT_AFR_ITEMS, absAddr(TGT_AFR_MAP_NVM_ADDR), tgtAFRMapDataTypes);
		cfSoftwareResetMaps();
		break;
	default:
		status = CF_UNKNOWN_BLOCK_ID;
		break;
	}
	return status;
}



// filters software reset, called from nvm.c after filters parameters changed
void cfSoftwareResetFilters(){
	seInitialise(DIAGNOSTIC_MODE);
}

// complete software reset called after power-up / hardware reset and from nvm.c after changes to either Parameters 1 & Parameters 2
void cfSoftwareReset(){
	nvTestEEPROMReady();										// test for an external EEPROM available and set the ecu status word accordingly
																// note that this must be done before initialising any of the EEPROM users (e.g. AFR functions)
	twInitialise();												// trigger wheel
	afInitialise(CYCLIC_PROCESSING_VLF_PERIOD, AFRCorrection);	// afr correction
	fuInitialise(CYCLIC_PROCESSING_HF_PERIOD);					// fuel injection
	seInitialise(DIAGNOSTIC_MODE);								// sensors
	aiInitialise(CYCLIC_PROCESSING_LF_PERIOD);					// auto idle
	vvInitialise();												// vvt controller
}

// software reset called from nvm.c after changes to the VE map or Target AFR map and indirectly after a reset AFR command (ra#)
void cfSoftwareResetMaps(){
	afInitialise(CYCLIC_PROCESSING_VLF_PERIOD, AFRCorrection);	// afr correction
	fuInitialise(CYCLIC_PROCESSING_HF_PERIOD);					// fuel injection
}


// Sets the currentConfiguration parameter, restores the configuration data for the new configuration and invokes a software reset.
// The supplied configNumber must be in the range 1 to 8 otherwise it is ignored and no action is taken.
// If the parameter is in range, the configuration descriptor structure is stored to NVM.
// Returns 0 if successful, otherwise 1
int cfSetCurrentConfig(int configNumber){
	int result = 1;
	if ((configNumber >= 1) && (configNumber <= 8)) {
		configurationDescriptor.currentConfiguration = configNumber;
		ecuStatus = 0;
		nvEEPROMBlockWrite((uint8_t *)&configurationDescriptor, 0, sizeof(configurationDescriptor));
		cfRestoreConfiguration();
		cfSoftwareReset();
		result = 0;
	}
	return result;
}


// Returns the EEPROM absolute address from a config block relative address, determined by the current configuration parameter
static uint16_t absAddr(int relAddr){
	return (uint16_t) ((configurationDescriptor.currentConfiguration - 1) * CONFIGURATION_PAGE_SIZE + CONFIGURATION_PAGE_START_ADDR + relAddr);
}


/*+++REVISION_HISTORY+++
1) 05 Jan 2021 Preparing to remove Flash memory as an NVM data store.
2) 06 Jan 2021 cfGetConfigDataStats(configDataStatsType *statPtr) and cfDataStats removed.
3) 09 Jan 2021 Configuration data now exclusively saved to / restored from external EEPROM. Flash memory no longer used.
4) 14 Jan 2021 ecuStatu now includes INVALID_CONFIG bit, set by cfRestoreConfiguration().
5) 25 Feb 2021 Default values added to configuration data. Allows ECU to operate even if EEPROM not present.
6) 02 Mar 2021 Multiple configuration capability added. Up to N different configuration pages can be saved to / restored from an address
   based on a new "current configuration" parameter. EEPROM addressing revised. Config Block ID's revised - no longer compatible with Arduino.
   New function added cfSetCurrentConfig() - sets the new config, restores data from new config addresses and invokes a software reset.
+++REVISION_HISTORY_ENDS+++*/
