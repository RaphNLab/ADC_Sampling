/*
 * ADC_drv.c
 *
 *  Created on: Oct 8, 2024
 *      Author: silvere
 */
#include "ADC_drv.h"
#include "drivers/timer_drv.h"


volatile uint16_t adc_dma_buf[ADC_CHANNELS];
volatile bool adc_eoc_flag = false;
volatile bool dma_xfer_cmpl = false;

volatile uint16_t adc_buf[ADC_CHANNELS][ADC_SAMPLES];
uint16_t sample_idx = 0;

volatile bool sample_cmpl = false;

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

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	//adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_eoc_after_each(ADC1);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_sample_time(ADC1, ADC_CHANNEL0 , ADC_SMPR_SMP_48CYC );
	adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_384CYC);
	//adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_384CYC);

	uint8_t channels[] = {ADC_CHANNEL_TEMP, ADC_CHANNEL0};
	adc_set_regular_sequence(ADC1, ADC_CHANNELS, channels);
	adc_enable_scan_mode(ADC1);
	adc_enable_temperature_sensor();
	adc_enable_eoc_interrupt(ADC1);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM4_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

	adc_set_dma_continue(ADC1);
	adc_power_on(ADC1);
}

void dma_setup(void)
{
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

	dma_channel_reset(DMA1, DMA_CHANNEL);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL, (uint32_t)ADC_DATA_REG);
	dma_set_memory_address(DMA1, DMA_CHANNEL, (uint32_t)adc_dma_buf);
	dma_set_number_of_data(DMA1, DMA_CHANNEL, ADC_CHANNELS);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL, DMA_CCR_PSIZE_16BIT);
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

	while(!dma_xfer_cmpl) {}

	ret_val = COMPUTATION_TEMPERATURE_TEMP30_TEMP110(adc_dma_buf[0]);
	dma_xfer_cmpl = false;

	return ret_val;
}


void adc_task(void)
{
	if(!sample_cmpl)
	{
		adc_buf[0][sample_idx] = adc_dma_buf[0];
		adc_buf[1][sample_idx] = adc_dma_buf[1];
		sample_idx++;
		if(sample_idx == 100)
		{
			sample_cmpl = true;
		}
	}
	else
	{
		printf("Sample done\n");
	}
}

/* ADC interrupt handler */
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

/* DMA isr handler */
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL, DMA_TCIF);
        dma_disable_channel(DMA1, DMA_CHANNEL);

        // Reset the flag
        adc_eoc_flag = false;

        //Set transfercomplete flag
        dma_xfer_cmpl = true;
        // Trigger next conversion
        //adc_start_conversion_regular(ADC1);
    }
}








