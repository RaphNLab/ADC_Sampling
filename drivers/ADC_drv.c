/*
 * ADC_drv.c
 *
 *  Created on: Oct 8, 2024
 *      Author: silvere
 */
#include "ADC_drv.h"
#include "drivers/timer_drv.h"
#include <libopencm3/stm32/adc.h>


void adc_config(void)
{
	rcc_periph_clock_enable(RCC_ADC1);
	adc_power_off(ADC1);
	//rcc_periph_reset_pulse(RST_ADC1);

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

	adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_384CYC);
	uint8_t channels[1] = {ADC_CHANNEL_TEMP};
	adc_set_regular_sequence(ADC1, 1, channels);
	adc_set_right_aligned(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
}


void adc_enable_temp(void)
{
	if(!adc_get_flag(ADC1, ADC_SR_ADONS))
	{
		adc_power_on(ADC1);
		sleep_us(50);
		adc_enable_temperature_sensor();
	}
}

uint16_t get_adc_value(void)
{
	uint16_t ret_val = 0;


	adc_start_conversion_regular(ADC1);
	while(!adc_get_flag(ADC1, ADC_SR_EOC));
	ret_val = adc_read_regular(ADC1);

	return ret_val;
}


int16_t get_temperature(void)
{
	int16_t ret_val = 0;

	volatile uint16_t temp_raw = get_adc_value();
	ret_val = COMPUTATION_TEMPERATURE_TEMP30_TEMP110(temp_raw);

	return ret_val;
}
