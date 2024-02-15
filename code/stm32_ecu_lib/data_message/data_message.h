#ifndef _dataMessageClass
#define _dataMessageClass

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


#include "cfg_data.h"


// The NVM transmit buffer needs to be sized as follows:
// 64 values each with format -9999.9 (7 chars x 64 = 448 chars) + preamble ($XXXX:YYYY.ZZZ,NN,B) (19 chars) + 64 separators + CRLF + NULL
// Total size of rx buffer should be 448 + 19 + 64 + 3 ~ 500
#define NVM_TX_BUFFER_SIZE 500

// similary for the data message buffer: 7 chars per item x 29 = 203 + preamble (1) + 29 separators + CRLF + NULL ~ 250
#define DATA_TX_BUFFER_SIZE 250

extern int formatCfgDataMessage(cfBlockID blockID);
extern int formatDataMessage(float dataArray[], int nItems);
extern char dataTxBuffer[DATA_TX_BUFFER_SIZE];
extern char nvmTxBuffer[NVM_TX_BUFFER_SIZE];
extern uint8_t dmDADP[KEY_DATA_STRUCT_SIZE];

#endif
