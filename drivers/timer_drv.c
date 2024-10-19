#include "timer_drv.h"

#include <libopencm3/stm32/timer.h>


/**
 * Data type, Constant and macro definitions
 *
*/

/**
 * Static data declaration
 *
*/

/**
 * Private function prototypes
 *
*/

/**
 * Public functions
 *
*/

void sleep_us(uint16_t delay_us)
{
	uint16_t start = timer_get_counter(TIM2);
	while((timer_get_counter(TIM2) - start) < delay_us);
}

void sleep_ms(uint16_t delay_ms)
{
	uint16_t i;
	for(i = 0; i < delay_ms; i++)
	{
		sleep_us(1000);
	}
}

/* Sleep timer: 1us tick */
void timer_sleep_setup(void)
{
	rcc_clock_setup_pll(&rcc_clock_config[RCC_CLOCK_VRANGE1_HSI_PLL_24MHZ]);
	rcc_periph_clock_enable(RCC_TIM2);

	timer_set_period(TIM2, 5000); // Generate timer overflow event every 2ms
	timer_set_prescaler(TIM2, 24); // Counter increment every 1us
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_enable_counter(TIM2);
}

/*Configure timer 3 for debouncing. Debounce time = 10ms*/
void timer_debounce_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM3);
	nvic_enable_irq(NVIC_TIM3_IRQ);

	timer_set_prescaler(TIM3, 24000);
	timer_set_period(TIM3, 10);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_enable_irq(TIM3, TIM_DIER_UIE);
}

/* Configured to generate timer event every 100ms */
void timer_adc_external_trigger_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM4);
	nvic_enable_irq(NVIC_TIM4_IRQ);

	timer_set_prescaler(TIM4, 24000);
	timer_set_period(TIM4, 25);
	timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_set_master_mode(TIM4, TIM_CR2_MMS_UPDATE);
	timer_enable_counter(TIM4);
}


/**
 * Private functions
 *
*/


