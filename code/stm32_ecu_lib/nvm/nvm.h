#ifndef _nvmObject
#define _nvmObject

/*
 * Provides Non Volatile Memory (NVM) storage functions for ECU configuration and other data.
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

#include "main.h"

extern int nvTestEEPROMReady(void);
extern HAL_StatusTypeDef nvEEPROMBlockWrite(uint8_t * data, uint16_t eepromAddress, int nBytes);
extern HAL_StatusTypeDef nvEEPROMBlockRead(uint8_t * destPtr, uint16_t eepromAddress, int nBytes);

#endif


/*+++REVISION_HISTORY+++
1) 09 Jan 2021 Flash read/write operations removed.
+++REVISION_HISTORY_ENDS+++*/
