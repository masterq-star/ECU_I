#ifndef _triggerWheelHandler
#define _triggerWheelHandler

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



#define NUM_INJECTORS 4

// teeth half, set at initialisation
extern int triggerWheelTeethHalf;

// conversion factors, set at initialisation
extern float rpmToTeethPerMillisecond;
extern float rpmFromPeriod;

// information on trigger wheel performance
extern volatile int crankPulsePeriodF;
extern volatile unsigned int triggerWheelInSync;

// handles the crankshaft trigger wheel pulse
extern void crankshaftPulseHandler(int crankPulsePeriod);

// switches off the injectors & coils
extern void injectorPowerReset(void);

// initialise tw software
extern void twInitialise(void);

// this flag is set by the camshaft handler to request an injector sequence reset. Set to non-zero resets the sequence.
extern volatile int twResetFlag;

#endif
