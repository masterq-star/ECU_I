#ifndef _cfg_data
#define _cfg_data

/*
 * The ECU now has the capability to hold up to 8 (or more, depending on type of EEPROM fitted) different configurations:
 * That is, 8 x configuration pages where each page consists of Filters + Parameters 1 + Parameters 2 + VE_Map + Ignition_Map + Target_AFR_Map
 *
 * A new configuration descriptor structure describes the currently selected configuration that can be
 * modified by a command from the host computer.
 *
 * However, only one set of AFR averaging data is maintained, common to all configurations.
 *
 * All NVM read/write operations act on the currently selected configuration.
 *
 * Refer to the spreadsheet "eeprom_addressing_V2.ods" for details on configuration data sizes and address allocation within the EEPROM.
 *
 */


#include "nvm.h"
#include "global.h"


// map axes, hence map data array size, are fixed at present
#define VE_MAP_SIZE_RPM 8
#define VE_MAP_SIZE_LOAD 8

// This 64 byte block contains information about the selected configuration.
// Only currentConfiguration is utilised at present, but 64 bytes (including the checksum) are reserved for future use.
// NB the checksum is appended at the end of the data block by the NVM block write function, so is not explicitly specified here.
typedef struct {
	int currentConfiguration;
	int unused[14];
} configurationDesciptorStruct;


// Note that the order of data blocks & data items in the following data structures must not be changed
// as the host computer depends on the ordering to identify individual data items.

typedef struct {
	float mapFilter;
	float lambdaSensorFilter;
	float coolantTempFilter;
	float airTempFilter;
	float tpsFilter;
	float voltageFilter;
	int	  reserved;
	int   crankshaftPulseFilter;
} filtersStruct;

typedef struct {
	float engTempCompT1;
	float engTempCompC1;
	float engTempCompT2;
	float engTempCompC2;
	float airTempCompT1;
	float airTempCompC1;
	float airTempCompT2;
	float airTempCompC2;
	float vvtPWM1;
	float vvtRPM1;
	float vvtPWM2;
	float vvtRPM2;
	float crankingThreshold;
	float crankingPW;
	float accelCompLimit;
	float accelCompAmplitude;
	float accelCompDuration;
	float coolingFanOnTemp;
	float tpsFastIdleValue;
	float tpsFastIdleTemp;
	float tpsNormalIdleValue;
	float tpsNormalIdleTemp;
	float idleActuatorGain;
	float reserved;
	float idleControlThreshold;
	float idleControlDelay;
	float afrCorrectionGainP;
	float afrCorrectionGainI;
	float afrAveragingFilterTC;
	float afrDataSavePeriod;
	float pseStartValue;
	float pseDecayTime;
	float psitStartValue;
	float psitDecayTime;
} parameters1Struct;

typedef struct {
	int   ecuID;
	int   numberRpmCells;
	int   numberLoadCells;
	float rpmAxisStart;
	float rpmAxisDelta;
	float loadAxisStart;
	float loadAxisDelta;
	float requiredFuel;
	float injectorLatency;
	int   ignitionFiringSense;
	float ignitionDwell;
	int   twTeeth;
	int   twMissingTeeth;
	float twTDCAngle;
	float injectorStartAngle;
	int   injectorIndex0;
	int   injectorIndex1;
	int   injectorIndex2;
	int   injectorIndex3;
	int   injectorSequenceReset;
	float thermistorT1;
	float thermistorR1;
	float thermistorT2;
	float thermistorR2;
	float tpsFullyClosedVoltage;
	float tpsFullyOpenVoltage;
	int   idleActuatorType;
} parameters2Struct;

typedef struct {
	filtersStruct filters;
	parameters1Struct p1;
	parameters2Struct p2;
	float veMap[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];
	float ignitionMap[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];
	float targetAFRMap[VE_MAP_SIZE_LOAD][VE_MAP_SIZE_RPM];
} page1Struct;



/*
 * Defines the EEPROM on-device relative addresses for each of the configuration data blocks. Note that an additional 4 bytes must
 * be allowed in the EEPROM space allocation to hold the checksum. Also, each block must start on a 64-byte boundary as the Page Write
 * method is utilised for the selected EEPROM device. e.g. Only 32 bytes will be written to the device in the filters block but
 * 64 bytes (one page) must be be allocated.
 */
#define FILTERS_NVM_ADDR 		   0
#define PARAMETERS_1_NVM_ADDR 	  64
#define PARAMETERS_2_NVM_ADDR 	 256
#define VE_MAP_NVM_ADDR 		 384
#define IGNITION_MAP_NVM_ADDR 	 704
#define TGT_AFR_MAP_NVM_ADDR 	1024


/*
 * The configuration blocks start at this EEPROM address
 */
#define CONFIGURATION_PAGE_START_ADDR 640

/*
 * Each configuration page is the following size. Note that the size takes into account space for checksums,
 * unused space due to EEPROM "Page Write" technique and provision for some growth potential.
 *
 * The address for a particular configuration page is therefore: (selectedConfigurationNumber - 1) x CONFIGURATION_PAGE_SIZE + CONFIGURATION_PAGE_START_ADDR + <block address>
 *
 */
#define CONFIGURATION_PAGE_SIZE 1344


/*
 * The AFR data is not part of the configuration data set but the address for the required
 * non-volatile AFR data is allocated here:
 */
#define AFR_DATA_NVM_ADDR		64

/*
 * A code number for each data block is used to identify the data block to/from the host computer.
 * The data block ID's are arbitrary but used to be the EEPROM address in the original Arduino code,
 * retained here to maintain host computer compatibility with Arduino-based ECUs.
 */
typedef enum {	FILTER_BLK 		= 100,
				PARAMETER_1_BLK = 200,
				PARAMETER_2_BLK = 300,
				VE_MAP_BLK 		= 400,
				IGN_MAP_BLK 	= 500,
				TGT_AFR_BLK 	= 600 } cfBlockID;

/*
 * Number of items in each configuration block
 */
typedef enum { 	FILTER_ITEMS 		= 8,
				PARAMETER_1_ITEMS 	= 34,
				PARAMETER_2_ITEMS 	= 27,
				VE_MAP_ITEMS 		= 64,
				IGN_MAP_ITEMS 		= 64,
				TGT_AFR_ITEMS 		= 64 } cfDataBlockItems;

// result type from a config operation
typedef enum { CF_SUCCESS, CF_INVALID, CF_ERASE_ERROR, CF_WRITE_ERROR, CF_DATA_SIZE_MISMATCH, CF_UNKNOWN_BLOCK_ID } cfErrorCode;

extern char filterDataTypes[];
extern char p1DataTypes[];
extern char p2DataTypes[];
extern char veMapDataTypes[];
extern char ignMapDataTypes[];
extern char tgtAFRMapDataTypes[];

extern configurationDesciptorStruct configurationDescriptor;
extern page1Struct cfPage1;
extern cfErrorCode cfSaveConfig(uint32_t *blkPtr, int blkSize, paramType newDataItems[], int nItemsSupplied, int itemsExpected, uint16_t eepromAddress, char dataTypes[]);
extern int cfRestoreConfiguration(void);
extern cfErrorCode cfProcessNVMMessage(cfBlockID block, int nItems, paramType *dataItems);
extern void cfSoftwareResetFilters(void);
extern void cfSoftwareReset(void);
extern void cfSoftwareResetMaps(void);
extern int cfSetCurrentConfig(int configNumber);

#endif


/*+++REVISION_HISTORY+++
1) 05 Jan 2021 Preparing to remove Flash memory as an NVM data store.
2) 06 Jan 2021 cfGetConfigDataStats(configDataStatsType *statPtr) and cfDataStats removed.
3) 06 Jan 2021 Padding no longer required in the config data structures.
4) 28 Feb 2021 Configuration descriptor data block added.
5) 02 Mar 2021 EEPROM addressing and block codes revised. New function cfSetCurrentConfig() added.
6) 06 Mar 2021 Idle actuator "Hold Power" variable in parameters1Struct changed to "reserved" as it's no longer utilised.
7) 11 May 2021 Included "global.h"
+++REVISION_HISTORY_ENDS+++*/


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
