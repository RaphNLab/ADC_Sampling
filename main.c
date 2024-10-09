#include "drivers/timer_drv.h"
#include "drivers/uart_drv.h"
#include "drivers/led_driver.h"

int main(void)
{
	timer_sleep_config();
	led_config();

	while (1)
	{

	}
	return 0;
}
