/*
 * ADC_drv.c
 *
 *  Created on: Oct 8, 2024
 *      Author: silvere
 */
#include "ADC_drv.h"
#include "drivers/timer_drv.h"
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>


uint16_t adc_raw[100];
bool_t dma_xfer_cmpl = FALSE;

void adc_config(void)
{
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_DMA1);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

	/*Set PA0 as analog pin*/
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

	adc_power_off(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP , ADC_SMPR_SMP_384CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_384CYC);
	uint8_t channels[] = {ADC_CHANNEL_TEMP};
	adc_set_regular_sequence(ADC1, 1, channels);
	adc_set_right_aligned(ADC1);
	adc_enable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);

	/* Configure DMA for ADC transfer */
	dma_channel_reset(DMA1, DMA_CHANNEL1);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)ADC_DATA_REG);
	dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_raw);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_LOW);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, (uint16_t)1);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);


	ADC_CR2(ADC1) |= ADC_CR2_DDS; // Important To issue DMA request after each conversion
	adc_enable_dma(ADC1);
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
	//volatile uint16_t temp_raw = get_adc_value();

	dma_enable_channel(DMA1, DMA_CHANNEL1);
	adc_start_conversion_regular(ADC1);

	if(dma_xfer_cmpl)
	{
		ret_val = COMPUTATION_TEMPERATURE_TEMP30_TEMP110(adc_raw[0]);
		dma_xfer_cmpl = FALSE;
	}

	return ret_val;
}


void dma1_channel1_isr(void)
{
	if(dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
	{
		//Clear flag and Disable DMA
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
		dma_disable_channel(DMA1, DMA_CHANNEL1);
		dma_xfer_cmpl = TRUE;
	}
}
