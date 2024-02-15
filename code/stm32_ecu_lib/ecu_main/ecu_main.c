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


// ******************************************************************************************************
// Remember to set DIAGNOSTIC_MODE to 0 in global.h for production builds!
// When set to 1, this inhibits reading of sensor data (in sensors.c), allowing simulated sensor
// inputs in a test harness. If set, calls to testCodeInitialisation() and testCodeCyclic() are made
// from ecuInitialisation() and void ecuLoop() respectively.
// ******************************************************************************************************


#include "ecu_main.h"
#include "main.h"
#include "global.h"
#include "cfg_data.h"
#include "ecu_services.h"
#include "nvm.h"
#include "command_decoder.h"
#include "cyclic_tasks.h"
#include "scheduler.h"
#include "aux_serial.h"
#include "auto_afr.h"
#include "string.h"
#include <stdio.h>
#if DIAGNOSTIC_MODE == 1
	#include "test_code.h"
#endif



int sendAuxMessageFlag = 0;
uint32_t syncMsgTime = 0;
int saveAFRFlag = 0;


// prototypes
void ecuLoop(void);


/*
 *
 * ecuInitialisation() must called from main.c to initialise & run the ecu software
 *
 */

void ecuInitialisation() {

	// clear the ecu status word
	ecuStatus = 0;

	// restore the configuration data from NVM
	//cfRestoreConfiguration();

	// start all of the services provided by ecu_services.c
	ecuServicesStart();
	// reset the software
	cfSoftwareReset();
  
	// initialise the scheduler with a tick period of 1 milli-second
	scInitialise(1);

	// HF cyclic tasks
	scAddTask(CYCLIC_PROCESSING_HF_TASKS, cyclicProcessingHFTasks, CYCLIC_PROCESSING_HF_PERIOD);

	// LF cyclic tasks
	scAddTask(CYCLIC_PROCESSING_LF_TASKS, cyclicProcessingLFTasks, CYCLIC_PROCESSING_LF_PERIOD);

	// VLF cyclic tasks
	scAddTask(CYCLIC_PROCESSING_VLF_TASKS, cyclicProcessingVLFTasks, CYCLIC_PROCESSING_VLF_PERIOD);

	// start the cyclic events
	scStartScheduler();

	#if DIAGNOSTIC_MODE == 1
		testCodeInitialise();
	#endif

	// enter the main loop
	
	ecuLoop();
	

} // end ecuInit



// **********************************
// Low priority background processing
// **********************************


void ecuLoop(){

	while(1){

		// process commands from host
		if (hostIO.rxMsgLength > 0){
			cdExecuteCommand(hostIO.rxBufferBase, hostIO.rxMsgLength);
			hostIO.rxMsgLength = 0;
		
		}

		// process commands from auxiliary serial
		if (auxIO.rxMsgLength > 0) {
			cdExecuteCommand(auxIO.rxBufferBase, auxIO.rxMsgLength);
			auxIO.rxMsgLength = 0;
		}

		// if the flag is set, send a data item on the auxSerial channel
		if (sendAuxMessageFlag > 0 ){
			sendAuxMessageFlag = 0;
			//auxSerialTransmit();      // send one items from the key data array
		}
		
    

		// send a sync message every second
		if ((HAL_GetTick() - syncMsgTime) >= 1000) {
			syncMsgTime = HAL_GetTick();
       char buffe[20];
		    sprintf(buffe, "Px,%d,%d#", (int)keyData.v.interpolatedAdvance,(int)keyData.v.interpolatedVE);
       //HAL_UART_Transmit(&huart1,(unsigned char *)buffe, strlen(buffe), 30);
			   hostPrint(buffe,strlen(buffe));
			char buffe2[20];
		    sprintf(buffe2, "Pd,%d,0#", (int)keyData.v.injectorPW);
			  hostPrint(buffe2,strlen(buffe2));
       //HAL_UART_Transmit(&huart1,(unsigned char *)buffe, strlen(buffe), 30);
			// note that async_serial allows a queue of two Tx messages - one in-progress and another waiting to be sent.
			// if there is a message in progress, we need to discard this sync message to keep the channel free for any
			// higher priority messages - such as a command response.
			//if (hostIO.txInProgress == 0){
			//	hostPrint(SYNC_MSG, strlen(SYNC_MSG));
			//}
		}

		// saving AFR data to NVM must be be done as a background task
		if (saveAFRFlag > 0) {
			saveAFRFlag = 0;
			// correctionSavedTime will be incremented if the whole data set was saved
			keyData.v.correctionSavedTime += afSaveAFRData(keyData.v.RPM, keyData.v.coolantTemperature);
		}

		#if DIAGNOSTIC_MODE == 1
			testCodeLoop();
		#endif

	} // end while

}

/*+++REVISION_HISTORY+++
1) 04 Nov 2020 ecuLoop() modified to accomodate async_serial package.
2) 05 Jan 2021 Changes to comments.
3) 06 Jan 2021 call to cfGetConfigDataStats() removed as no longer required.
4) 11 Jan 2021 Restore config data now first operation in ecuInitialisatio() after clearing the ecu status word.
5) 03 May 2021 Period sync message timing no longer set by cyclicProcessingVLFTasks(). Instead, sync message timing obtained using HAL_GetTick().
6) 11 May 2021 Removed all code relating to HSI adjustment as this was never fully tested or implemented (and probably not required).
+++REVISION_HISTORY_ENDS+++*/
