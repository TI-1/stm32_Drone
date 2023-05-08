/*
 * debug.h
 *
 *  Created on: Apr 19, 2023
 *      Author: tobii
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "usart.h"

#ifdef MYDEBUG
#define Debug(x) x
#else
#define Debug(x)
#endif



#endif /* INC_DEBUG_H_ */
