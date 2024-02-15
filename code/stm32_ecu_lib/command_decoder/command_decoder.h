#ifndef _commandDecoder
#define _commandDecoder

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


extern char SYNC_MSG[];
extern void cdExecuteCommand(char cmdLine[], int length);

#endif
