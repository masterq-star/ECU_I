#ifndef _sensorClass
#define _sensorClass

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




#define MAX_ANALOG_INPUTS 6


// low pass filter (lpf) parameters
typedef struct {
	float xN_1; // last value
	float alpha;
} lpfParameterStruct;


// indicates if the sensors are disabled.
// if set to non-zero, sensors data is not updated, allowing a test harness
// to inject test values for disagnostic purposes.
extern int sensorsDisabled;

// read & filter analog inputs
// reads the specified number of inputs and returns the filtered result in sensorDataArray
// the raw unfiltered input is stored in rawAnalog
// rawAnalog[0] => uses lpf[0] => filtered data returned at index 0
// rawAnalog[1] => uses lpf[1] => filtered data returned at index 1
// etc...
extern void readAnalog(float sensorDataArray[]);

// calculated resistance of the NTC thermistor
extern float thermistorRt;

// initialises the NTC thermistor co-efficients
extern void initNTC(float T1, float Rt1, float T2, float Rt2);

// initialises the filters, starts an ADC conversion and sets the disabled flag
extern void seInitialise(int disable);


#endif
