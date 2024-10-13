#include "drivers/timer_drv.h"
#include "drivers/uart_drv.h"
#include "drivers/led_driver.h"
#include "ADC_drv.h"

int main(void)
{
	timer_sleep_config();
	led_config();

	serial_debug_config();
	adc_config();
	adc_enable_temp();

	while (1)
	{
		int16_t temperature = 0;

		temperature = get_temperature();
		printf("%d°C\n", temperature);
	}
	return 0;
}
