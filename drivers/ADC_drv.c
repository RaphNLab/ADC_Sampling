/*
 * ADC_drv.c
 *
 *  Created on: Oct 8, 2024
 *      Author: silvere
 */
#include "ADC_drv.h"
#include <libopencm3/stm32/adc.h>


void adc_config(void)
{
	adc_set_sample_time(ADC1, ADC_CHANNEL16, ADC_SMPR_SMP_24CYC);
	adc_set_right_aligned(ADC1);
	adc_set_continuous_conversion_mode(ADC1);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
}


void adc_enable_temp(void)
{
	if(adc_get_flag(ADC1, ADC_SR_RCNR) != TRUE && adc_get_flag(ADC1, ADC_SR_ADONS))
	{
		adc_power_on(ADC1);
		adc_start_conversion_regular(ADC1);
		adc_enable_temperature_sensor();
	}
}

uint16_t get_adc_value(void)
{
	uint16_t ret_val = 0;

	if(adc_get_flag(ADC1, ADC_SR_EOC))
	{
		ret_val = adc_read_regular(ADC1);
	}

	return ret_val;
}


uint16_t get_temperature(void)
{
	uint16_t ret_val = 0;

	float tmp = 0.0;
	uint32_t *ts_cal1 = (uint32_t*)TS_CAL1_REG;
	uint32_t *ts_cal2 = (uint32_t*)TS_CAL2_REG;

	uint16_t temp_raw = get_adc_value();

	tmp = (float)((110.0 - 30.0) / ((float)*ts_cal1 - (float)*ts_cal1));

	ret_val = (uint16_t)((uint16_t)(tmp) * (temp_raw - *ts_cal1)) + 30;

	return ret_val;
}
