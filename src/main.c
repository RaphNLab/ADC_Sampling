#include "timer_drv.h"
#include "uart_drv.h"
#include "led_driver.h"
#include "ADC_drv.h"

int main(void)
{
	timer_sleep_setup();
	led_setup();

	uartDevConfig(&myUartDev, MY_USART_DEVICE, uartRxBuffer, uartTxBuffer, USART_DATA_LEN);
	clock_setup();
	adc_setup();
	dma_setup();
	adc_enable_temp();
	timer_adc_external_trigger_setup();
	adc_enable_dma(ADC1);
	//adc_start_conversion_regular(ADC1);

	while (1)
	{
		int16_t temperature = 0;

		temperature = get_temperature();
		serial_trace("%d°C\n", temperature);
		adc_task();
		//__asm__("wfi");
	}
	return 0;
}
