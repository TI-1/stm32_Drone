/*
 * helperfns.h
 *
 *  Created on: Nov 21, 2023
 *      Author: tobii
 */

#ifndef INC_HELPERFNS_H_
#define INC_HELPERFNS_H_

#include <math.h>


#define FLT_EPSILON 1.19209290E-07F

bool floatComparison(float x,float y){
	if (fabs(x-y) < FLT_EPSILON){
		return true;
	}
	else {
		return false;
	}
}

template <typename T> T map(T value, T in_min, T in_max, T out_min, T out_max) {

		  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		}




#endif /* INC_HELPERFNS_H_ */
