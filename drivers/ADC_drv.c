/*
 * ADC_drv.c
 *
 *  Created on: Oct 8, 2024
 *      Author: silvere
 */
#include "ADC_drv.h"
#include "drivers/timer_drv.h"


volatile uint16_t adc_value;
volatile bool adc_eoc_flag = false;


void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_clock_config[RCC_CLOCK_VRANGE1_HSI_PLL_32MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA1);
}

void adc_setup(void)
{
	nvic_enable_irq(NVIC_ADC1_IRQ);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_384CYC);

	adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL_TEMP});
	adc_enable_temperature_sensor();
	adc_enable_eoc_interrupt(ADC1);
	ADC_CR2(ADC1) |= ADC_CR2_DDS;
	adc_power_on(ADC1);
}

void dma_setup(void)
{
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

	dma_channel_reset(DMA1, DMA_CHANNEL);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL, (uint32_t)ADC_DATA_REG);
	dma_set_memory_address(DMA1, DMA_CHANNEL, (uint32_t)&adc_value);
	dma_set_number_of_data(DMA1, DMA_CHANNEL, 1);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL, DMA_CCR_PSIZE_16BIT );
	dma_set_memory_size(DMA1, DMA_CHANNEL, DMA_CCR_MSIZE_16BIT);
	dma_set_priority(DMA1, DMA_CHANNEL, DMA_CCR_PL_LOW);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL);
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

	adc_start_conversion_regular(ADC1);

	sleep_ms(10);
	while(!dma_xfer_cmpl) {}

	ret_val = COMPUTATION_TEMPERATURE_TEMP30_TEMP110(adc_value);
	dma_xfer_cmpl = FALSE;

	return ret_val;
}


void adc1_isr(void)
{
    if (adc_get_flag(ADC1, ADC_SR_EOC)) {
        adc_clear_flag(ADC1, ADC_SR_EOC);
        if (!adc_eoc_flag) {
            dma_enable_channel(DMA1, DMA_CHANNEL);
            adc_eoc_flag = true;
        }
    }
}

void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL, DMA_TCIF);
        dma_disable_channel(DMA1, DMA_CHANNEL);

        // Print the raw ADC value
        printf("Raw ADC value: %d\n", adc_value);

        // Reset the flag
        adc_eoc_flag = false;

        // Trigger next conversion
        adc_start_conversion_regular(ADC1);
    }
}

/*void dma1_channel1_isr(void)
{
	if(dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
	{
		//Clear flag and Disable DMA
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
		dma_disable_channel(DMA1, DMA_CHANNEL1);
		printf("adc_raw %d\n", adc_raw[0]);
		dma_xfer_cmpl = TRUE;
	}
}

void adc1_isr(void)
{
	if(adc_get_flag(ADC1, ADC_SR_EOC))
	{
		adc_clear_flag(ADC1, ADC_SR_EOC);
		dma_enable_channel(DMA1, DMA_CHANNEL1);
		//uint16_t *adc_dr = ADC_DATA_REG;
		//printf("adc_dr %d\n", *adc_dr);
	}
}*/
