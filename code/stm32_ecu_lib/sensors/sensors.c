

/*

Reads & conditions the analog sensors.


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

#include "sensors.h"
#include "math.h"
#include "global.h"
#include "utility_functions.h"
#include "cfg_data.h"
#include "ecu_services.h"

/*
 *
 *
 */

// prototypes
float applyFilter(float x, int lpfIndex);
float temperatureFromThermistorVoltage(uint16_t ADCOutput);

// Engine coolant thermistor settings
#define NTC_PULLUP_RESISTOR 3300.0F
#define NTC_SUPPLY_VOLTAGE 5.0F

// ADCs must be set to 10 bit resolution
// this factor converts the raw Analog to Digital Converter (ADC) output to milli-volts = 3.3 / 1.023

//#define CONVERT_ADC_TO_MILLIVOLTS 3.2258064516129F
#define CONVERT_ADC_TO_MILLIVOLTS 0.8058608058608F

// this factor converts the raw Analog to Digital Converter (ADC) output to volts = 3.3 / 1023
//#define CONVERT_ADC_TO_VOLTS 0.0032258064516129F
#define CONVERT_ADC_TO_VOLTS 0.0008058608058608F
// low pass filter co-efficients
lpfParameterStruct lpf[MAX_ANALOG_INPUTS];

// sensors disabled flag
int sensorsDisabled;

// thermistor conversion coefficients & estimated thermistor resistance
static float NTCa, NTCb;
float thermistorRt;

// Multiplier & Offset to transform TPS voltage to 0% - 100% representing fully closed & fully open pedal positions
static float TPSOffset = 0;
static float TPSMultiplier = 1;


// define the array indices for processed output data and associated filters
#define MAP_INDEX 		0
#define LAMBDA_INDEX 	1
#define ENG_TEMP_INDEX 	2
#define AIR_TEMP_INDEX	3
#define TPS_V_INDEX		4
#define VOLTS_INDEX		5


/*

readAnalog() initiates an ADC conversion, waits for completion then filters & converts the data stored in adcRawData[] into real world units.
Data is output to the supplied array in the following order:

Index	Function		Units
  0 	MAP				KPa
  1 	Lambda V		mV
  2		Engine Temp		Deg C
  3		Air Temp		Deg C
  4		TPS Voltage		mV
  5		Voltage			V

TPS% in keyData is also calculated.

*/

void readAnalog(float sensorDataArray[]){

	// process analog data if sensor not disabled
	if (sensorsDisabled == 0) {

		// start an ADC conversion & wait for completion
		startADCConversion();
		waitForADCCompletion();

		// process each signal using the pin number / raw data slot defined for the EEPROM version of the adapter board in ecu_services.h
		// the conversion factors for the following sensors are defined in the separate spreadsheet "conversions.ods"
		sensorDataArray[MAP_INDEX] 		= applyFilter(0.1075258065F * ((float)adcRawData[ADC_MAP]) + 9.4444F, MAP_INDEX);
		sensorDataArray[LAMBDA_INDEX] 	= applyFilter(3.2258064516F * ((float)adcRawData[ADC_LAMBDA]), LAMBDA_INDEX);
		sensorDataArray[AIR_TEMP_INDEX] = applyFilter(0.3225806452F * ((float)adcRawData[ADC_AIR_TEMP]), AIR_TEMP_INDEX);
		sensorDataArray[TPS_V_INDEX] 	= applyFilter(4.8387096774F * ((float)adcRawData[ADC_TPSV]), TPS_V_INDEX);
		sensorDataArray[VOLTS_INDEX] 	= applyFilter(0.0354838710F * ((float)adcRawData[ADC_VOLTAGE]), VOLTS_INDEX);

		// engine temp from the NTC thermistor needs additional processing
		sensorDataArray[ENG_TEMP_INDEX] = applyFilter(temperatureFromThermistorVoltage(adcRawData[ADC_ENG_TEMP]), ENG_TEMP_INDEX);

		// calculate TPS input as a percentage - 0% = throttle closed, 100% = fully open
		keyData.v.TPS = (sensorDataArray[TPS_V_INDEX] - TPSOffset) * TPSMultiplier;

	}

}


// calculate NTC co-efficients from calibration data
void initNTC(float T1, float Rt1, float T2, float Rt2) {
	NTCa = (T2 - T1) / logf(Rt2 / Rt1); // use "logf"??
	NTCb = T1 - NTCa * logf(Rt1);
}

void seInitialise(int disable){
	// clear the low pass filter N-1 values
	for (int i=0; i < MAX_ANALOG_INPUTS; i++){
		lpf[i].xN_1 = 0;
	}
	// set the filter co-efficients
	lpf[MAP_INDEX].alpha = cfPage1.filters.mapFilter;
	lpf[LAMBDA_INDEX].alpha = cfPage1.filters.lambdaSensorFilter;
	lpf[ENG_TEMP_INDEX].alpha = cfPage1.filters.coolantTempFilter;
	lpf[AIR_TEMP_INDEX].alpha = cfPage1.filters.airTempFilter;
	lpf[TPS_V_INDEX].alpha = cfPage1.filters.tpsFilter;
	lpf[VOLTS_INDEX].alpha = cfPage1.filters.voltageFilter;


	// set the NTC conversion factors
	initNTC(cfPage1.p2.thermistorT1, cfPage1.p2.thermistorR1, cfPage1.p2.thermistorT2, cfPage1.p2.thermistorR2);

	// take an initial reading to set the engine coolant sensor filter
	// this is to avoid the fan being switched on while the filter catches up
	readAnalog(keyData.dataArray);
	lpf[2].xN_1 = keyData.dataArray[2];		// index 2 is engine coolant temp

	// calculate TPS coefficients
	TPSOffset = cfPage1.p2.tpsFullyClosedVoltage;
	TPSMultiplier = 100.0F / (cfPage1.p2.tpsFullyOpenVoltage - TPSOffset);

	// save the disable switch
	sensorsDisabled = disable;
}

// ADC thermistor voltage to Deg C.
float temperatureFromThermistorVoltage(uint16_t ADCOutput){

	// get the thermistor voltage
	float Vt = CONVERT_ADC_TO_VOLTS * (float) ADCOutput;

	// calculate resistance of NTC thermistor
	thermistorRt = limitF(Vt * NTC_PULLUP_RESISTOR / (NTC_SUPPLY_VOLTAGE - Vt), 0.0001F, 99999.0F);

	// return temperature
	return NTCa * logf(thermistorRt) + NTCb;
}

// applies a low pass filter to the selected input
float applyFilter(float x, int lpfIndex) {
	float y = (x - lpf[lpfIndex].xN_1) * lpf[lpfIndex].alpha + lpf[lpfIndex].xN_1;
	lpf[lpfIndex].xN_1 = y;
	return y;
}
