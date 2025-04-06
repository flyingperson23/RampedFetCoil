/*
 * int.c
 *
 *  Created on: Mar 24, 2025
 *      Author: maddie <3
 */
#include "int.h"

uint16_t TS_CAL1 = 0;
uint16_t VREFINT = 0;
float VREF = 0;
float temp = 0;
float batt_v = 0;
uint16_t adc_buf[8];
uint32_t lasttick = 0;

/*
 *  0 - batt
 *  1 - ot
 *  2 - bps
 *  3 - vmax
 *  4 - ocd
 *  5 - dtc
 *  6 - temp
 *  7 - vref
*/

void IntInit() {
	TS_CAL1 = *((uint16_t *) 0x1FFF7568);
	VREFINT = *((uint16_t *) 0x1FFF756A);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buf, 8);

	HAL_Delay(100);

	HAL_TIM_Base_Start_IT(&htim17); // start main loop timer
	HAL_TIM_Base_Start_IT(&htim16); // start output timer

	HAL_HalfDuplex_EnableTransmitter(&huart1);
}

uint8_t mode = SS;
uint8_t state = OFF;
uint8_t button = OFF;
float v_max = 0;
float ocd = 0;
float bps = 0;
float dtc = 0;
float ontime = 0;
float ontime_limited = 0;

void MainLoop() {

	if (HAL_GetTick() - lasttick >= 100) {
		HAL_GPIO_WritePin(BLED_GPIO_Port, BLED_Pin, GPIO_PIN_RESET);
	}

	float VREF_NEW = 3.0f * (float) VREFINT / (float) adc_buf[7];
	if (VREF_NEW < 5.0f && VREF_NEW > 2.0f) {
		VREF = (VREF + VREF_NEW) / 2.0f;
	}

	float sense_data = (float) adc_buf[6] * VREF / 3.0f;
	float avg_slope_code = 2.53f * 4096.0f / 3000.0f;
	temp = (temp + ((sense_data - (float) TS_CAL1) / avg_slope_code - 30.0f)) / 2.0f;

	batt_v = (batt_v + 2.0f * (float) adc_buf[0] * VREF / 4096.0f) / 2.0f;
	if (batt_v < SHUTDOWN_THRESHOLD) {
		HAL_GPIO_WritePin(BATT_LED_GPIO_Port, BATT_LED_Pin, GPIO_PIN_RESET);

		//HAL_PWREx_EnterSHUTDOWNMode(); // TODO: test
	} else {
		HAL_GPIO_WritePin(BATT_LED_GPIO_Port, BATT_LED_Pin, GPIO_PIN_SET);
	}

	ontime = (float) adc_buf[1] / 4095.0f * MAX_OT; // ms
	bps = (float) adc_buf[2] / 4095.0f * MAX_BPS; // bps
	v_max = (float) adc_buf[3] / 4095.0f * MAX_V; // V
	ocd = (float) adc_buf[4] / 4095.0f * MAX_OCD; // A
	dtc = (float) adc_buf[5] / 4095.0f * MAX_DTC; // %

	if ((ontime / 1000.0f) * bps > dtc) { // ontime * bps = dtc
		ontime_limited = (dtc / bps) * 1000.0f;
	} else {
		ontime_limited = ontime;
	}

	float timer_freq = 24000000.0 / 65536.0;
	if (bps > 0) {
		TIM16->ARR = (int) (timer_freq / bps); // timer_freq / arr = triggering frequency
	} else {
		TIM16->ARR = 65535;
	}


	if (HAL_GPIO_ReadPin(SW_SINGLESHOT_GPIO_Port, SW_SINGLESHOT_Pin) == GPIO_PIN_SET) {
		if (mode == SS) {
			mode = CN;
			state = OFF;
		} else {
			mode = SS;
		}
	}

	if (HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_SET) {
		if (button == OFF) { // pressed
			if (mode == SS) {
				SendPulse(ontime, v_max, ocd);
			} else {
				if (state == OFF) {
					state = ON;
				} else {
					state = OFF;
				}
			}
		}
		button = ON;
	} else {
		button = OFF;
	}
}

uint32_t float_to_u32(float input) {
	return  *((uint32_t *) &input);
}

void FillBuffer(uint8_t * buffer, uint32_t value, uint8_t position) {
	buffer[position * 4 + 0] = (value >> 0) & 0xFF;
	buffer[position * 4 + 1] = (value >> 8) & 0xFF;
	buffer[position * 4 + 2] = (value >> 16) & 0xFF;
	buffer[position * 4 + 3] = (value >> 24) & 0xFF;
}

void SendPulse(float OT, float Vmax, float OCD) {
	uint8_t buffer[12]; // 3 values, 4 bytes per value
	FillBuffer(buffer, float_to_u32(OT), 0);
	FillBuffer(buffer, float_to_u32(Vmax), 1);
	FillBuffer(buffer, float_to_u32(OCD), 2);
	HAL_UART_Transmit(&huart1, buffer, 12, 1000);

	HAL_GPIO_WritePin(BLED_GPIO_Port, BLED_Pin, GPIO_PIN_SET);
	lasttick = HAL_GetTick();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim17) {
		MainLoop();
	}
	if (htim == &htim16) {
		if (mode == CN && state == ON) {
			SendPulse(ontime_limited, v_max, ocd);
		}
	}
}


