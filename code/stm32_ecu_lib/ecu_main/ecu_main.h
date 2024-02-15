#ifndef _ecu_main
#define _ecu_main

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



// task numbers for high-frequency, low frequency and very low frequency tasks
#define CYCLIC_PROCESSING_HF_TASKS 0
#define CYCLIC_PROCESSING_LF_TASKS 1
#define CYCLIC_PROCESSING_VLF_TASKS 2

// define the task periods in milli-seconds
#define CYCLIC_PROCESSING_HF_PERIOD 5
#define CYCLIC_PROCESSING_LF_PERIOD 40
#define CYCLIC_PROCESSING_VLF_PERIOD 1000

#define VE_MAP_SIZE_RPM 8
#define VE_MAP_SIZE_LOAD 8

extern int sendAuxMessageFlag;
extern int saveAFRFlag;
extern void ecuInitialisation(void);

#endif

/*+++REVISION_HISTORY+++
1) 03 May 2021 sendSyncMessageFlag removed from global space.
+++REVISION_HISTORY_ENDS+++*/
