#ifndef _global
#define _global

/*
 * +++REVISION_HISTORY+++
 *
 * Revision History
 * ================
 *
 * V3200.01 31st Jan 2020
 *
 * 1)	First prototype with hand wired stm32 NUCLEO G431KB adapter board.
 * 		Configured to suit the Arduino-based CR14DE configuration.
 *
 * V3200.02 7th Feb 2020
 *
 * 1)	Separate timers allocated for injection events at TDC & TDC + 180 to cater for
 * 		injector pulse widths that take more than 180 degrees of crankshaft angle
 *
 * V3200.03 20th Feb 2020
 *
 * 1)	Moves the mechanics of injector & ignition timing from cyclic_tasks to trigger_wheel_handler.
 * 		trigger_wheel's only dependency on cyclic_tasks is now to calculate the injection PW and ignition advance
 *
 * V3200.04 14th Mar 2020
 *
 * 1)	Revised AFR correction algorithm, using PI controller.
 *
 * V3200.05 27th March 2020
 *
 * 1) 	AFR correction now modifies VE Map prior to interpolation rather than modifying the PW calculation directly. This smooths
 * 		the transition of the injector PW when moving through corrected cells.
 *
 * V3200.06 10th April 2020
 *
 * 1)	Changes to aux_serial to specify data format for each individual data item.
 * 2)	Fixed an error where the crankshaft filter TC set in NVM was not being used.
 * 3)	Changed keyData.v.loadIndex to keyData.v.toothNoAtMT to aid diagnosis of sync errors.
 * 4)	Changed keyData.v.rpmIndex to keyData.v.currentCell, encoded as per AFRIndex.
 * 5) 	In ecu_sevices, TIM2 crankshaft pulse interrupt changed to highest priority.
 * 6)	keyData.v.stepperSteps changed to keyData.v.idleActuatorCmd.
 *
 *
 * V3200.061 27th April 2020
 *
 * 1)	ADC & PWM pin assignments for the PCB version of the G431 adapter board - sensors & actuatorType2
 * 2)	Aux baud rate set to 38400 in ecu_services
 *
 *
 * V3200.062 29th April 2020
 *
 * 1)	Ecu status word introduced to capture ecu health and certain operating flags. ecuStatus is transmitted on the host & aux serial interfaces.
 * 2)	Aux serial data message now same format as host data message i.e. prefix data with "*" & use LF as EOM.
 * 3)	Does a partial reset after VE Map or Target AFR map data is updated in NVM
 * 4)	Delete keyData coolingFanPower, EWPPower and reduce the size of the data message by two. (cooling fan power is now obtained from the ecu status word).
 *
 *
 * V3200.063 6th May 2020
 *
 * 1)	All channels on both ADCs now set to 640.5 cycles sampling time.
 * 2)	On both ADC's End Of Conversion flag changed to Sequence instead of single conversion.
 * 3)	*** These changes were made in main.c - the configuration tool was not used ***
 *
 *
 * V3200.064 10th May 2020
 *
 * 1)	Host commands are now accepted on the auxiliary serial channel. This is to allow simple commands (such as "ra#" & "tt<N>#") to
 * 		be issued e.g. via a Bluetooth device, without the need for the Windows desktop app. Commands are processed in exactly the same
 * 		way as from the host. Configuration changes still require the Windows app.
 *
 * 2)	Input filter on crankshaft trigger wheel pulse detector (TIM2 / input capture on CH1) set to 15 in main.c / MX_TIM2_Init().
 * 		Therefore a trigger pulse needs to be at least 32 / 85Mhz * 8 (~3 uS) long before it is recognised.
 * 		This is to improve noise rejection in the trigger wheel processing.
 *
 *
 * V3200.065 14th May 2020
 *
 * 1) 	On occasion, reading the ADC's in blocking mode, using HAL_ADCEx_InjectedPollForConversion() can cause an endless loop, freezing the whole
 * 		application. ADC routines now changed to read the ADCs directly without polling with the conversion being started on the previous cycle. This
 * 		inserts some lag into the analog signals.
 *
 * 2)	Corrected an error in ecuISRAuxUART() where the buffer overflow check used RX_BUFFER_SIZE instead of AUX_RX_BUFFER_SIZE.
 *
 * 3)	CubeMX configuration tool brought into line with changes to main.c in Versions V3200.063 & V3200.064.
 *
 *
 * V3200.066 16th May 2020
 *
 * 1)	ADC2 now used in regular conversion DMA mode for all 6 analogue inputs. ADC1 no longer required.
 *
 * 2)	ADC_TIMEOUT	bit introduced into ecuStatus word
 *
 * 3)	Unwanted HAL interrupt service callback code generation suppressed from configuration tool. No longer need to
 * 		comment out or otherwise edit the stm32g4xx_it.c file after making changes from the config tool.
 *
 * 4)	In the keyData structure, toothNoAtMT changed to "errorTooth" - affects trigger_wheel_handler, aux_serial
 *
 * 5)	Data precision on aux serial data items made specific to each item
 *
 *
 * V3200.067 23rd May 2020
 *
 * 1)	Revised injector sequence reset logic in trigger_wheel_handler - see configuration guide jet_config_guide_v3200.06.odt for revised operation.
 *
 *
 * V3200.068 22nd July 2020
 *
 * 1)	Major change to reorganise NVM. Flash is used for configuration data, as before. AFR tuning data can only be stored in an external
 * 		I2C EEPROM and cumulative error data is no longer stored, irrespective of storage medium.
 *		To enable external EEPROM, the CPU needs to be configured in CubeMX with the I2C #2 interface enabled and the macro I2C_INTERFACE defined in main.h
 *		which holds the HAL I2C interface handle. Note pinout changes required (to Injector A and AUX_TX) to utilise the I2C interface.
 *		The identification message (response to the si# command from the host) will contain "I2C enabled" if the code is configured with the I2C interface and
 *		"EEPROM available" if the external EEPROM is fitted and working. The EEPROM status is also reflected in the ecu status word.
 * 2)	ECU status word revised.
 * 3)	Revised usage of #include directive - affects nearly all files. Mainly removes #includes from .h files where possible.
 * 4)	To improve the factory 1% accuracy of the main internal clock, temporary code has been tested in ecu_main.c that adjusts the
 * 		main clock (HSI) by a value from the reserved slot in Filters configuration data. This has since been commented out but could be
 * 		enabled in a future update, if required.
 * 5)	I2C builds use different pinouts for analogue inputs - conditional added to sensors.c
 *
 *
 * V3200.069 2nd August 2020
 *
 * 1) 	aiType2ActuatorSetIdle() now uses ecu status word macro (TEST_IDLE_SWITCH_ON == TRUE) instead of accessing the hardware directly.
 * 2)	To improve accuracy and SNR, ADC resolution changed to 10 bit in cubeMX configurator. Conversion factors in sensors.c modified accordingly.
 * 		Comments in ecu_services.c modified accordingly.
 * 3) 	Analogue inputs in sensors.c permanently set for pinouts in the EEPROM version of the adapter board.
 * 4)	To reduce ambiguity, sensors.c now uses direct variable names & mnemonic references rather than numeric offsets, where possible.
 * 5)	In sensors.c conversions from ADC to real-world units simplifed. Conversions defined in separate spreadsheet "conversions.ods"
 * 6)	"Fast-Mode" on Injector B output was previously enabled in CubeMX. Now disabled as this mode appears to be unrelated to GPIO functions.
 * 7)	ecu_services now uses HAL ADC Complete callback to indicate end of conversion & completed DMA transfer rather that direct interrogation of registers.
 *
 *
 * 		*** Important ***
 * 		The recent hardware change makes the MAP signal Rank 1 in the ADC conversion process. This has revealed a fault in the ADC conversion / DMA transfer
 * 		process that makes the Rank 1 conversion erratic, rendering the ECU unusable.
 *
 *
 * V3200.070 4th August 2020
 *
 * 1) 	The MAP signal input Rank (previously Rank 1) was swapped with the Voltage signal (previously Rank 4). This has cured the erratic MAP signal,
 * 		but Voltage now suffers the same issue. Reducing the ADC clock frequency by setting the ADC Clock Prescaler to “Divided By 8” signifcantly increases
 * 		the stability of the conversion, presumably by allowing allowing a longer sample time. This increases total conversion time but does not impact the
 * 		overall cycle time. In summary, the signal at Rank 1 seems to need a longer settling time than the subsequent ranks. Also, slowing down the ADC clock
 * 		(hence increasing the sample time) can significantly improve stability of the digital output from ADC conversion process.
 *
 *
 * V3200.080 4th August 2020
 *
 * 1) 	I2C interface #2 configured in CubeMX. Macro definition "I2C_INTERFACE &hi2c2" in main.h enables attempt to use EEPROM for AFR data storage.
 *
 *
 * V3200.081 4th August 2020
 *
 * 1)	Float constants suffixed with "F" and conversions explicity stated (where appropriate). Affects sensors.c, fuel_injection.c, cyclic_tasks.c, auto_afr.c, actuatorType2.c,
 * 		data_message.c, trigger_wheel.c
 * 2)	Where type float results are required from Math.h functions, float versions are used instead of double to avoid unecessary conversion. Affects: fuel_injection.c, sensors.c (log to logf).
 *
 *
 * V3200.082 4th November 2020
 *
 * 1)	Replaces host & aux serial comms mechanics in ecu_services with the async_serial package.
 *
 *
 * V3200.083 21st November 2020
 *
 * 1) Timeout added to assSend() in async_serial.
 *
 *
 * V3200.084 9th Jan 2021
 *
 * 1) EEPROM_CHECKSUM_ERROR added to ECU status word. Checksum capability added to EEPROM block read & write operations.
 * 2) Flash memory no longer used to store configuration data. All non-volatile storage requirements now provided by external EEPROM device via the I2C data bus.
 * 3) INVALID_CONFIG bit added to ECU status word.
 *
 *
 * V3200.085 12th Feb 2021
 *
 * 1) auto idle modified to recover and maintain the required idle setting when transitioning from speed to idle.
 * 2) A number of changes made to select the correct functions in math.h for float data types, e.g. round() to roundf().
 *
 *
 * V3201.01 16th Feb 2021
 *
 * 1) The version number scheme will be rationalised as follows:
 *
 * 		XXXX.YY
 *
 *    Where XXXX is the major version (i.e. incremented when a significant change in functionality or a change to the external interface is made) and
 *    YY is the subversion (i.e. no significant change in functionality but a refinement or bug fix that does not change any external interface)
 *
 * 2) Auto idle actuatorType2 no longer adds "Hold Power" to actuator output. Also actuator demand set to zero if very small.
 *
 *
 * V3201.02 26th Feb 2021
 *
 * 1) Default values added to configuration data to allow ECU to operarate without a working external EEPROM.
 * 2) Experimental multi-configuration capability added.
 *
 *
 * V3202.01 5th Mar 2021
 *
 * 1) Major revision to add multi-configuration capability. Set Configuration "sc<N>#" command added. Send Sync "sy#" command removed.
 *    No longer compatible with last version of Arduino code.
 *
 *
 * V3202.02 10th May 2021
 *
 * 1) SEND_SYNC command in command_decoder.c re-instated.
 * 2) Corrected error in data_message.c at line 116 - was PARAMETER_1_ITEMS, corrected to PARAMETER_2_ITEMS.
 * 3) From now on, all ECU commands must only return a one line response. Hence, sendIdentificationMessage() in command_decoder.c
 *    modified to send only one line. Single-line responses simplify the host software and ensure a more reliable COMMAND-ACKNOWLEDGE handshake.
 * 4) Sync message timing used to be set by cyclicProcessingVLFTasks(). Now timing is determined in ecu_main.c using HAL_GetTick().
 * 5) aux_serial now includes the current configuration in the ID field of the data message, i.e. "*N,D1,D2...Dn"
 *
 *
 * V3202.03 20th May 2021
 *
 * 1) *** Note: HAL drivers have been updated at this version ***
 *   o STM32CubeMX IDE is now at Version: 1.6.1.
 *   o STM32CubeMX - STM32 Device Configuration Tool -  is now at Version: 6.2.1-RC2
 *
 * 2) Fixes error in actuatorType2.c.
 * 3) Fixes non-functional error in trigger_wheel_config.c.
 * 4) ecu_services: adcReadyFlag made volatile. waitForADCCompletion() no longer uses HAL to time timeout loop.
 * 5) async_serial.c asseTimeout type change.
 *
 *
 *
 *
 *+++REVISION_HISTORY_ENDS+++*/


#define MAIN_VERSION 	3202.03
#define VERSION_DATE 	"20 May 2021"



/*
 * Remember to set DIAGNOSTIC_MODE to 0 for production builds!
 * When set to 1, this inhibits reading of sensor data (in sensors.c), allowing
 * simulated sensor inputs in a test harness and calls testCodeInitialise()
 */

#define DIAGNOSTIC_MODE 1


#include "main.h"


/*
 * This struct defines the key operational data used throughout the ECU software.
 * Key data members transmitted to the host computer in the data message and represent
 * and represent important sensor and control data used and generated by the application.
 *
 * *** The order of the data items transmitted in the data message is the same as
 * they appear in the data structure, hence the order should not be changed without a
 * corresponding change to the host computer ***
 *
 */

typedef struct {
  float timestamp;         		//0 - Seconds from last power on
  float MAP;               		//1 - Manifold Absolute Pressure
  float lambdaVoltage;     		//2 - Current lambda (o2) sensor voltage
  float coolantTemperature;		//3 - Coolant (engine) temperature
  float airTemperature;    		//4 - Air temp
  float TPSVoltage;        		//5 - Raw voltage from the Throttle Position Sensor
  float voltage2;          		//6 - Voltage measurement
  float RPM;               		//7 - Current RPM
  float injectorPW;        		//8  - Fuel injector Pulse Width in microseconds
  float interpolatedVE;    		//9  - Interpolated VE value from the VE map
  float TPS;               		//10 - Throttle Position 0 - 100%
  float targetTPS;         		//11 - Target TPS for idle
  float tempCompensation;  		//12 - The product of air temp comp and engine temp comp
  float accelCompensation; 		//13 - The acceleration compensation value
  float thermistorResistance;   //14 - Thermistor resistance
  float currentCell;       		//15 - The RPM & Load indices of the current cell, encoded as defined below.

  float errorTooth;       		//16 - After a sync error is detected this is updated with the tooth number at the sync error. Can be used to help determine
  	  	  	  	  	  	  	  	//	   if there are too many pulses which may indicate excessive electrical noise. Or too few, possibly indicating a weak
  	  	  	  	  	  	  	  	//     signal or defective components.

  float syncErrors; 		   	//17 - Number of trigger wheel sync errors
  float vvtPwr;					//18 - Power applied to the VVT actuator
  float	interpolatedAdvance;	//19 - Interpolated ignition advance from the ignition map
  float idleActuatorCmd;		//20 - Either the number of steps in a stepper motor configuration or a duty cycle in a PWM configuration.
  float spare23; 				//21 - Unused
  float AFRCorrection;			//22 - Applied AFR correction for VE Map cell defined by AFR Index
  float lambdaVoltageAverage;	//23 - Average Lambda voltage for VE Map cell defined by AFR Index
  float AFRIndex;				//24 - The map cell for which AFRCorrection, lambdaAverageVoltage & lambdaVoltageSamples applies, encoded as defined below
  float correctionSavedTime;	//25 - The number of times the AFR correction array was saved
  float lambdaVoltageSamples;	//26 - The number of samples of lambda voltage in each cell
} keyDataStruct;

/*
 * cell index encoding: cell = loadIndex * VE_MAP_SIZE_RPM + rpmIndex
 *
 */


// number of items in the key data structure
#define KEY_DATA_STRUCT_SIZE 27

// allows key data to be accessed as an array or as individual named items
typedef union {
  keyDataStruct v;
  float dataArray[KEY_DATA_STRUCT_SIZE];
} keyDataUnion;



// data type used to hold parameters decoded from a "wf" message
typedef struct {
	float f;
	int i;
} paramType;



// define the ecu status word, which is used to capture ecu cpu faults and certain application flags

typedef enum {

	// This flag group indicates CPU faults & exceptions
	// CPU_FAULT flags are set in the relevant exception handler in stm32g4xx_it.c
	// ADC TIMEOUT if the ADC fails to convert within a specified period - ecu_services.c
	CPU_HARD_FAULT 			= 0x00000001,
	CPU_MEM_MGT_FAULT 		= 0x00000002,
	CPU_BUS_FAULT			= 0x00000004,
	CPU_USAGE_FAULT			= 0x00000008,
	ADC_TIMEOUT				= 0x00000010,

	// Flags in this group are related to the external EEPROM
	EEPROM_AVAILABLE		= 0x00000100,		// set by nvm.c if external EEPROM is available
	EEPROM_DATA_SAVE_ERROR	= 0x00000200,
	EEPROM_DATA_READ_ERROR	= 0x00000400,
	EEPROM_CHECKSUM_ERROR	= 0x00000800,

	// This flag group is application-specific:
	IDLE_SWITCH_ON 			= 0x00001000,
	COOLING_FAN_ON 			= 0x00002000,
	AFR_ACTIVE_CONTROL		= 0x00004000,
	INVALID_CONFIG			= 0x00008000

} ecuStatusEnum;


#define STAT ecuStatus

#define SET_CPU_HARD_FAULT				STAT |= CPU_HARD_FAULT
#define SET_CPU_MEM_MGT_FAULT	 		STAT |= CPU_MEM_MGT_FAULT
#define SET_CPU_BUS_FAULT				STAT |= CPU_BUS_FAULT
#define SET_CPU_USAGE_FAULT				STAT |= CPU_USAGE_FAULT
#define SET_ADC_TIMEOUT					STAT |= ADC_TIMEOUT
#define SET_EEPROM_WRITE_ERROR			STAT |= EEPROM_DATA_SAVE_ERROR
#define SET_EEPROM_READ_ERROR			STAT |= EEPROM_DATA_READ_ERROR
#define SET_EEPROM_CHECKSUM_ERROR		STAT |= EEPROM_CHECKSUM_ERROR
#define SET_EEPROM_AVAILABLE			STAT |= EEPROM_AVAILABLE
#define SET_INVALID_CONFIG				STAT |= INVALID_CONFIG

#define SET_AFR_ACTIVE_CONTROL			STAT |= AFR_ACTIVE_CONTROL
#define CLEAR_AFR_ACTIVE_CONTROL		STAT &= ~AFR_ACTIVE_CONTROL

#define SET_IDLE_SWITCH_ON				STAT |= IDLE_SWITCH_ON
#define CLEAR_IDLE_SWITCH_ON			STAT &= ~IDLE_SWITCH_ON

#define SET_COOLING_FAN_ON				STAT |= COOLING_FAN_ON
#define CLEAR_COOLING_FAN_ON			STAT &= ~COOLING_FAN_ON



#define TRUE 1
#define FALSE 0

#define TEST_IDLE_SWITCH_ON				(STAT & IDLE_SWITCH_ON) == 0 ? FALSE : TRUE
#define TEST_EEPROM_AVAILABLE			(STAT & EEPROM_AVAILABLE) == 0 ? FALSE : TRUE

extern keyDataUnion keyData;
extern uint32_t ecuStatus;


#endif


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
