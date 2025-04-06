/*
 * int.h
 *
 *  Created on: Mar 24, 2025
 *      Author: maddie <3
 */

#ifndef INC_INT_H_
#define INC_INT_H_

#include "main.h"

// CONFIG:

#define SHUTDOWN_THRESHOLD 3.0 // V
#define MAX_DTC 0.35 // %
#define MAX_OT 20.0 // ms
#define MAX_BPS 20.0 // bps
#define MAX_V 340.0 // V
#define MAX_OCD 200.0 // A

#define SS 0 // singleshot
#define CN 1 // continuous

#define OFF 0
#define ON 1

void SendPulse(float OT, float Vmax, float OCD);
void IntInit();

#endif /* INC_INT_H_ */
