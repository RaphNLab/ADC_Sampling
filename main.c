#include "drivers/timer_drv.h"
#include "drivers/uart_drv.h"
#include "drivers/led_driver.h"
#include "ADC_drv.h"

int main(void)
{
	timer_sleep_config();
	led_config();

	serial_debug_config();
	clock_setup();
	adc_setup();
	dma_setup();
	adc_enable_temp();


	adc_enable_dma(ADC1);
	adc_start_conversion_regular(ADC1);

	while (1)
	{
		/*int16_t temperature = 0;

		temperature = get_temperature();
		printf("%d°C\n", temperature);*/
		__asm__("wfi");
	}
	return 0;
}
