#include "global.h"


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


// Key data holds the principle ECU operating data. This is transmitted to the host
// computer on receipt of the sd# command.
keyDataUnion keyData;

// holds the engine management status, transmitted to the host on receipt
// of the sd# command just prior to the keyData array. Use
uint32_t ecuStatus;



/*+++REVISION_HISTORY+++
3) 11 Jan 2021 ecuStatus type changed to uint32_t from unsigned int.
+++REVISION_HISTORY_ENDS+++*/
