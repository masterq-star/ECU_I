#ifndef _autoIdleClass
#define _autoIdleClass

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



extern void aiInitialise(float cyclicPeriod);
extern float aiGetTargetTPS(float engineTemp);
extern float aiSetIdleActuator(float TPS, float targetTPS);
extern float aiGetDemand(float TPS, float targetTPS);

extern float aiType1ActuatorSetIdle(float TPS, float targetTPS);
extern void aiType1ActuatorInit(void);
extern float aiType2ActuatorSetIdle(float TPS, float targetTPS);
extern void aiType2ActuatorInit(void);

extern float aiTargetTPSAdjust;
extern float aiKi;
extern float aiKd;
extern float aiErrorSum;



#endif


/*+++REVISION_HISTORY+++
1) 12 Feb 2021 Added: extern aiGetDemand(float TPS, float targetTPS). aiType1ActuatorSetIdle() & aiType2ActuatorSetIdle() arguments changed.
+++REVISION_HISTORY_ENDS+++*/
