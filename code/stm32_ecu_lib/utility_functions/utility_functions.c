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


#include "utility_functions.h"
#include "math.h"

float limitF(float x, float lower, float upper) {
	float r = x > upper ? upper : x;
	r = r < lower ? lower : r;
	return r;
}

int limitI(int x, int lower, int upper) {
	int r = x > upper ? upper : x; // r = min(x,upper)
	r = r < lower ? lower : r; // r = max(lower,r )
	return r;
}

// Limits the input value to the range a, b. Where a & b are the extremes, not necessarily upper or lower limits.
float rangeF(float x, float a, float b){
	return limitF(x, fminf(a, b), fmaxf(a, b));
}

/*+++REVISION_HISTORY+++
1) 12 Feb 2021 Changed fmin() & fmax() to fminf() & fmaxf() to suit float types being used.
+++REVISION_HISTORY_ENDS+++*/
