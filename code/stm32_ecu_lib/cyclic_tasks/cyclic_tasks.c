/*
 *
 *
 * F303K8 CPU time for HF tasks:
 * 0.56 mS @ 3,000RPM with the trigger wheel sim. AFR correction not yet implemented. (2nd Dec 2019)
 *
 * FG431KB CPU time for HF tasks:
 * 0.200 mS @ 3,000RPM with the trigger wheel sim. AFR correction not yet implemented. (8th Dec 2020)
 * 0.160 mS @ ~8,800RPM with the trigger wheel sim. All functions operative. (12th Jan 2020)
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


#include "cyclic_tasks.h"
#include "ecu_main.h"
#include "global.h"
#include "trigger_wheel_handler.h"
#include "fuel_injection.h"
#include "ignition.h"
#include "scheduler.h"
#include "sensors.h"
#include "auto_afr.h"
#include "auto_idle.h"
#include "vvt_controller.h"
#include "aux_canbus.h"
#include "ecu_services.h"
#include <stdio.h>
#include <string.h>

// prototypes
void coolingFanControl(float engineTemp);


void cyclicProcessingHFTasks() {
	

	// Get systick count from HAL (1 tick every milli-second).
	keyData.v.timestamp = 0.001F * (float)HAL_GetTick(); // store as seconds

	// use trigger wheel "In Sync" count to determine if the engine is running
	if (triggerWheelInSync > 0) {
		// calculate RPM, avoiding divide by 0
		keyData.v.RPM = crankPulsePeriodF > 0 ? rpmFromPeriod / (float)crankPulsePeriodF : 0.0F;
	}

	// read the analog inputs & store filtered results into the the keyData data array starting at the 2nd element.
	// i.e. the 1st input is MAP, 2nd lambda, etc.
	// if sensorsDisabled is set, then reading sensors is skipped, allowing the controlling function to simulate 
	// sensor inputs by modifying keyData.dataArray
	readAnalog(&keyData.dataArray[1]);
	
	// call to fuel injection mapLookup() is essential to set up the RPM & Load indices (and other variables used in the iterpolation
	// process) for	downstream procedures in the efi, ignition and autoAFR objects
	mapLookup(keyData.v.RPM, keyData.v.MAP);
		
	// update the Lambda voltage averaging array & compute the correction value for the cell and update the AFR correction array in the fuel object
    afComputeCorrection(keyData.v.RPM, keyData.v.coolantTemperature, currentCell.loadIndex, currentCell.rpmIndex, keyData.v.lambdaVoltage, AFRCorrection);
	
	// get the fuel injector Pulse Width in microseconds
	keyData.v.injectorPW = getInjectorPulseWidth(keyData.v.RPM, keyData.v.MAP, keyData.v.TPS, keyData.v.coolantTemperature, keyData.v.airTemperature);

	// update the ignition timing
	keyData.v.interpolatedAdvance = igGetIgnitionAngle();
	
	// update the key variables object from fuel_injection
	keyData.v.currentCell = (float)(currentCell.loadIndex * VE_MAP_SIZE_RPM + currentCell.rpmIndex);
	keyData.v.interpolatedVE = interpolatedVE;
	keyData.v.tempCompensation = tempComp;
	keyData.v.accelCompensation = accelCompensationValue;

	// tell the background process to send a data item on the auxiliary serial channel
	sendAuxMessageFlag = 1;

	// tell the scheduler that HF tasks are complete
	scCompleted(CYCLIC_PROCESSING_HF_TASKS);
	
}

// *** cyclic processing, low frequency tasks ***
	
void cyclicProcessingLFTasks(){

	// read the idle switch and set the ecu status flag accordingly
	if(DIAGNOSTIC_MODE == 1 )
	{
	if (keyData.v.TPS ==0) 
		{
		SET_IDLE_SWITCH_ON;
	}
	else {
		CLEAR_IDLE_SWITCH_ON;
	}
	}
	else{
	if (HAL_GPIO_ReadPin(Throttle_Closed_Switch_GPIO_Port, Throttle_Closed_Switch_Pin) == GPIO_PIN_RESET) 
		{
		SET_IDLE_SWITCH_ON;
	}
	else {
		CLEAR_IDLE_SWITCH_ON;
	}
}

	// auto-idle
	// get the target TPS value i.a.w. engine temperature
	keyData.v.targetTPS = aiGetTargetTPS(keyData.v.coolantTemperature);

	// set the idle actuator. Use stepperSteps to record actuator power.
	keyData.v.idleActuatorCmd = aiSetIdleActuator(keyData.v.TPS, keyData.v.targetTPS);

	// VVT controller
	keyData.v.vvtPwr = vvSetVVT(keyData.v.RPM);
	// update the estimated thermistor resistance value
	keyData.v.thermistorResistance = thermistorRt;
  	// send message to canbus

	CAN_SEND_MESS_RPM(&hcan1,keyData.v.RPM);

  
	// tell the scheduler that LF tasks are complete
	scCompleted(CYCLIC_PROCESSING_LF_TASKS);
	

	
} // low freq tasks
  
  
// *** cyclic processing, very low frequency tasks ***
	
void cyclicProcessingVLFTasks() 
	{
	// N-1 in-sync count
	static unsigned int triggerWheelInSyncLast = 0;

	// test for changes in trigger wheel sync to determine if the engine is turning
	if ( triggerWheelInSync == triggerWheelInSyncLast ) {

		// no change, so crankshaft must be stationary (or the trigger wheel is excessively out of sync)
		// clear RPM, the sync count & sync error count
		triggerWheelInSync = 0;
		keyData.v.syncErrors = 0;
		keyData.v.RPM = 0;

		// and make sure power controls are turned off
		injectorPowerReset();
	}
	triggerWheelInSyncLast = triggerWheelInSync;

	// run the cooling fan control
	coolingFanControl(keyData.v.coolantTemperature);

	// send message to background process to save AFR data
	// it can't be done here as saving to Flash will block for a considerable time and potentially cause overrun problems
	saveAFRFlag = 1;
	CAN_SEND_THROTTLE_SW(&hcan1,TEST_IDLE_SWITCH_ON);
	CAN_SEND_DATA_SENSOR(&hcan1,keyData.v.MAP,keyData.v.lambdaVoltage,keyData.v.airTemperature,keyData.v.coolantTemperature);
	CAN_SEND_DATA_PW(&hcan1,keyData.v.injectorPW,keyData.v.interpolatedAdvance);
	// toggle the nucleo LED to indicate we're alive
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	// tell the scheduler that VLF tasks are complete
	scCompleted(CYCLIC_PROCESSING_VLF_TASKS);

} // end v.low frequ tasks



/*
 * Control the cooling fan.
 * Sets the cooling fan power demand if the cooling fan ON threshold has been exceeded for N successive cycles.
 * Sets the ecu status word to reflect cooling fan status
 *
 */
void coolingFanControl(float engineTemp){
	static unsigned int coolingFanOnCount = 0;
	if (engineTemp > cfPage1.p1.coolingFanOnTemp) {
//		if (++coolingFanOnCount > 4) {
//			HAL_GPIO_WritePin(Fan_Control_GPIO_Port, Fan_Control_Pin, GPIO_PIN_SET);
//			SET_COOLING_FAN_ON;
//		}
		__HAL_TIM_SetCompare (&htim12,TIM_CHANNEL_1,4000);
		SET_COOLING_FAN_ON;
	}
	else {
//		HAL_GPIO_WritePin(Fan_Control_GPIO_Port, Fan_Control_Pin, GPIO_PIN_RESET);
//		coolingFanOnCount = 0;
		CLEAR_COOLING_FAN_ON;
		__HAL_TIM_SetCompare (&htim12,TIM_CHANNEL_1,0);
	}
}


/*+++REVISION_HISTORY+++
1) 03 May 2021 Sync message flag no longer set by cyclicProcessingVLFTasks()
+++REVISION_HISTORY_ENDS+++*/

