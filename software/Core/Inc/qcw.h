/*
 * qcw.h
 *
 *  Created on: Mar 17, 2025
 *      Author: maddie <3
 */

#ifndef INC_QCW_H_
#define INC_QCW_H_

#include "main.h"
#include "math.h"

void QCW_Init();
void QCW_Loop();

#define UART_SIZE 12

// CONFIG

#define START_CYCLES 25
#define START_FREQ 350000
#define PHASE_LEAD 10
#define MIN_PHASE 0
#define MAX_TEMP 60.0
#define FAN_START 30.0
#define FAN_END 45.0
#define CHARGE_THRESHOLD 1.0
#define RAMP_STEPS 200
#define FB_DR_TH 0.1
#define FB_AV_TH 0.4

void StartPulse(float length, float end_v1, float OCD);


#endif /* INC_QCW_H_ */
