/*
 * qcw.c
 *
 *  Created on: Mar 17, 2025
 *      Author: maddie <3
 */

#include "qcw.h"

uint16_t vbus_buf[1];
uint16_t aux_buf[3]; // ext temp, int temp, vrefint

uint8_t uart_buffer[UART_SIZE];

uint16_t TS_CAL1;
uint16_t TS_CAL2;
uint16_t VREFINT;

float VREF = 3.3f;
float temp_int = 0;
float temp_ext = 0;
float vbus = 0;

float transfer_function[RAMP_STEPS];

uint32_t fb_dr_upper;
uint32_t fb_av_upper;
uint32_t fb_dr_lower;
uint32_t fb_av_lower;

void QCW_Init() {
	GD_DIS_GPIO_Port->BRR = GD_DIS_Pin; // disable GD

	HAL_Delay(1000);

	TS_CAL1 = *((uint16_t *) 0x1FFF75A8); // get calibration data from memory
	TS_CAL2 = *((uint16_t *) 0x1FFF75CA);
	VREFINT = *((uint16_t *) 0x1FFF75AA);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) aux_buf, 3);

	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) vbus_buf, 1);

	HAL_TIM_Base_Start(&htim15); // ADC trigger comparator

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // OCD dac - Configured to be the - input of OCD comparator

    HAL_COMP_Start(&hcomp1); // OCD and ZCD comparators
    HAL_COMP_Start(&hcomp2);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // input capture

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // LED1 - Vbus
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // LED2 - Ready
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // LED3 - Pulse
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // LED4 - OCD
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // FAN

    HAL_TIM_Base_Start_IT(&htim7);

    HAL_HalfDuplex_EnableReceiver(&huart1);
    HAL_UART_Receive_IT(&huart1, uart_buffer, UART_SIZE);

    for (int i = 0; i < RAMP_STEPS; i++) {
    	transfer_function[i] = 1.0f - (1.0f / M_PI) * acos(2.0f*(float) i / (float) RAMP_STEPS - 1.0f);
    	// voltage(percent) = 0.5 - 0.5cos(pi*phase(percent))
    	// phase(percent) = 1 - 1/pi arccos(2*voltage(percent) - 1)
    	// correct for phase shift to effective voltage nonlinearity
    }
}

float vbus_last = 0;
uint8_t rdy = 0;
void QCW_Loop() { // 10Hz
	if (rdy && (temp_ext < MAX_TEMP)) {
		TIM4->CCR2 = TIM4->ARR / 4; // Set ready light
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
	} else {
		TIM4->CCR2 = 0; // Clear ready light
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
	}

	float difference = vbus - vbus_last;
	if (difference < CHARGE_THRESHOLD && vbus > 50) rdy = 1;
	vbus_last = vbus; // precharge

	float fan = (temp_ext - (float) FAN_START) / ((float) FAN_END - (float) FAN_START);

	if (fan < 0) fan = 0;
	if (fan > 1) fan = 1;

	//fan = 1.0f;

	TIM4->CCR3 = (int) (fan * (float) TIM4->ARR); // fan
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1) {
		if (aux_buf[2] != 0) {
			float vref_new = 3.0f * (float) VREFINT / (float) aux_buf[2];
			if (vref_new < 5.0f && vref_new > 1.5f) {
				VREF = (VREF + vref_new) / 2.0f; // stm's have an internal bandgap reference! so you can figure out exactly what vdda is!
			}
		}
		temp_int = (100.0f)/((float) TS_CAL2 - (float) TS_CAL1) * ((float) aux_buf[1] - (float) TS_CAL1) + 30.0f;
		float volts_therm = (float) aux_buf[0] * VREF / 4095.0f;
		temp_ext = (volts_therm - 0.5f) * 100.0f;
	}
	if (hadc == &hadc2) {
		vbus = (float) vbus_buf[0] * VREF / 4095.0f * 201.0f;// read bus voltage to correct for sag
		if (vbus < 350) {
			TIM4->CCR1 = (int) (TIM4->ARR * (vbus / 350.0)); // Set VBUS LED
		} else {
			TIM4->CCR1 = TIM4->ARR;
		}

		//TODO: REMOVE
		//vbus = 200;
		//rdy = 1;
	}

}

uint8_t start_counter = 0;
uint8_t ocd = 0;
uint16_t ramp_cnt = 0;
float end_v = 0;
int ccr3 = 0;

// length in ms
void StartPulse(float length, float end_v1, float OCD) {
	if (vbus > 0 && rdy) {
		uint32_t counts = (uint32_t) (4095.0 / VREF * OCD / 200.0f * 2.0f); // 200:1 CT, 2R burden
	    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, counts);

		TIM3->CCR1 = TIM3->ARR; // Start pulse led
		TIM3->CCR2 = 0; // Clear OCD led

		ocd = 0;
		ramp_cnt = 0;
		TIM1->ARR = (((int) (170000000.0 / START_FREQ)) >> 1) - 1;
		TIM1->CNT = 0;
		start_counter = 0;
		end_v = end_v1;

		TIM1->CCR1 = TIM1->ARR - PHASE_LEAD;
		TIM1->CCR2 = PHASE_LEAD;

		TIM1->CCR3 = MIN_PHASE;
		TIM1->CCR4 = TIM1->ARR - MIN_PHASE;

		TIM6->ARR = (uint32_t) (170000.0 * length / (float) RAMP_STEPS) - 1; // ramp adjust timer
		TIM6->CNT = 0;

		TIM8->ARR = TIM1->ARR;
		TIM8->CNT = 0;
		TIM8->CCR1 = TIM8->ARR - PHASE_LEAD;

		HAL_TIM_Base_Start_IT(&htim6);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

		HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);

		GD_DIS_GPIO_Port->BSRR = GD_DIS_Pin; // enable

	}
}

void EndPulse() {
    GD_DIS_GPIO_Port->BRR = GD_DIS_Pin; // disable

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Base_Stop(&htim6);

    TIM3->CCR1 = 0; // Clear pulse led
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		if (start_counter >= START_CYCLES) {

		    //TIM1->ARR = TIM2->CCR1 >> 1; // asymmetric has up/down counting
		    //TIM8->ARR = TIM1->ARR;


			/*
		    uint32_t new_arr = TIM2->CCR1 >> 1;

		    if (new_arr > TIM1->ARR) {
		    	if (new_arr < fb_dr_upper) {
		    		TIM1->ARR = new_arr;
		    	} else if (new_arr < fb_av_upper) {
		    		TIM1->ARR = (TIM1->ARR + new_arr) >> 1;
		    	}
		    } else {
		    	if (new_arr > fb_dr_lower) {
		    		TIM1->ARR = new_arr;
		    	} else if (new_arr > fb_av_lower) {
		    		TIM1->ARR = (TIM1->ARR + new_arr) >> 1;
		    	}
		    }
		    TIM8->ARR = TIM1->ARR;


		    fb_dr_upper = (uint32_t) ((float) TIM1->ARR * (1.0f + FB_DR_TH));
		    fb_dr_lower = (uint32_t) ((float) TIM1->ARR * (1.0f - FB_DR_TH));
		    fb_av_upper = (uint32_t) ((float) TIM1->ARR * (1.0f + FB_AV_TH));
		    fb_av_lower = (uint32_t) ((float) TIM1->ARR * (1.0f - FB_AV_TH));
*/


		    // this uses tim2 to capture the period of the zcd signal that comes from the zcd comparator
		    // unless doing phase shift modulation this is the only code you *need*, to sync the output timer's period
		    // w/ the incoming period
		}
	    TIM2->CNT = 0;
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
	if (hcomp == &hcomp1) {
		ocd = 1; // detected OCD event
	}
}

uint8_t hardsw_side = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim8) {

		if (ocd) { // if ocd, turn off on timer reset
			EndPulse();
		    TIM3->CCR2 = TIM3->ARR; // Set OCD Led
		}

		if (start_counter < START_CYCLES) {
			start_counter++;
		} // TODO: make a wick ??

		//hardsw_side = !hardsw_side; // alternate hardswitch side

		if (hardsw_side) {
			TIM1->CCR1 = TIM1->ARR - PHASE_LEAD;
			TIM1->CCR2 = PHASE_LEAD;

			TIM1->CCR3 = ccr3;
			TIM1->CCR4 = TIM1->ARR - ccr3;

		} else {
			TIM1->CCR1 = ccr3;
			TIM1->CCR2 = TIM1->ARR - ccr3;

			TIM1->CCR3 = TIM1->ARR - PHASE_LEAD;
			TIM1->CCR4 = PHASE_LEAD;

		}
		TIM8->CCR1 = TIM8->ARR - PHASE_LEAD;
	}
}

// Asymmetric pwm: up down counting; for output channel 1 ccr1 controls compare value for up counting,
// ccr2 for down counting. for output channel 3, ccr3 for up and ccr4 for down. ccr2-ccr1 = ccr4-ccr3 = arr for 50% dtc
// but shift ccr1 relative to ccr3 for phase shift modulation, 100% power when ccr1=ccr3
// if the channel not being shifted is offset a little from the end i.e. ccr1 = arr - pl, ccr2 = pl this gives pl as phase lead
// bc the comparator is configured to reset the counter of the main timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) { // increment ramp
		ramp_cnt++;

		if (ramp_cnt >= RAMP_STEPS) {
			EndPulse();
		} else if (vbus > 0) {
			uint16_t tf_index = (uint16_t) ((float) ramp_cnt * end_v / vbus);
			if (tf_index >= RAMP_STEPS) tf_index = RAMP_STEPS - 1;
			float phase_percent = transfer_function[tf_index];
			ccr3 = (int) ((float) TIM1->ARR * phase_percent) - PHASE_LEAD;
			if (ccr3 < MIN_PHASE) ccr3 = MIN_PHASE;
			if (ccr3 > TIM1->ARR) ccr3 = TIM1->ARR;
		}

	}
	if (htim == &htim7) {
		QCW_Loop();
	}
}

float u32_to_float(uint32_t input) {
  return *((float *) &input);
}

uint32_t GetValue(uint8_t * buffer, uint8_t position) {
  return  (buffer[position * 4 + 0] << 0)  |
		  (buffer[position * 4 + 1] << 8)  |
		  (buffer[position * 4 + 2] << 16) |
		  (buffer[position * 4 + 3] << 24);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	float OT = u32_to_float(GetValue(uart_buffer, 0));
	float Vmax = u32_to_float(GetValue(uart_buffer, 1));
	float OCD = u32_to_float(GetValue(uart_buffer, 2));

	if (Vmax < 500 && Vmax > 0 && OT < 200 && OT > 0 && OCD < 600 && OCD > 0 && temp_ext < MAX_TEMP && rdy) {
		StartPulse(OT, Vmax, OCD);
	}

	HAL_UART_Receive_IT(&huart1, uart_buffer, UART_SIZE);
}
