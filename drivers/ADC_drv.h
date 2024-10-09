/*
 * ADC_drv.h
 *
 *  Created on: Oct 8, 2024
 *      Author: silvere
 */

#ifndef DRIVERS_ADC_DRV_H_
#define DRIVERS_ADC_DRV_H_

#include "global.h"

#define TS_CAL1_REG 0x1FF800FAU
#define TS_CAL2_REG 0x1FF800FEU
#define CALIBRATION_SIZE 1U

#define V_REF	3U


void adc_config(void);
void adc_enable_temp(void);
uint16_t get_adc_value(void);
uint16_t get_temperature(void);



#endif /* DRIVERS_ADC_DRV_H_ */
