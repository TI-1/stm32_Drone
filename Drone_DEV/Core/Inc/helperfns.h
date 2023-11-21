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




#endif /* INC_HELPERFNS_H_ */
