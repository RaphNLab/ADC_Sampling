#include "uart_drv.h"

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>

/**
 * Data type, Constant and macro definitions
 *
*/


uint8_t uartRxBuffer[USART_DATA_LEN];
uint8_t uartTxBuffer[USART_DATA_LEN];

UartDev_T myUartDev;
uint8_t isrCnt = 0;

char *uartCmdList[] =
{
	"RUN_T0",
	"HELP",
	"SET_LB"
};


/**
 * Static data declaration
 *
*/

/**
 * Private function prototypes
 *
*/
static void uartGpioSetup(void);
static void serial_debug_setup(void);
static void ascii_uart_puts_n(char* buf, uint16_t len);
static void printMenu(void);



/**
 * @brief Enable and configure GPIO pins used as alternate function for RX and TX pin
 * @param None
 * */
static void uartGpioSetup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, (GPIO2 | GPIO3));
	gpio_set_af(GPIOA, GPIO_AF7, (GPIO2 | GPIO3));
}


/**
 * @brief Configure the serial console by setting the baudrate, databits, RX/TX mode
 * 		  Enable interupt and activate the USART device
 * @param None
 * */
static void serial_debug_setup(void)
{
	rcc_periph_clock_enable(RCC_USART2);

	usart_set_parity(MY_USART_DEVICE, USART_PARITY_NONE);
	usart_set_baudrate(MY_USART_DEVICE, CONSOLE_BAUDRATE);
	usart_set_stopbits(MY_USART_DEVICE, USART_STOPBITS_1);
	usart_set_databits(MY_USART_DEVICE, CONSOLE_DATABIT);
	usart_set_flow_control(MY_USART_DEVICE, USART_FLOWCONTROL_NONE);
	usart_set_mode(MY_USART_DEVICE, USART_MODE_TX_RX);

	/*Enable Interrupt on RX pin*/
	usart_enable_rx_interrupt(MY_USART_DEVICE);
	nvic_enable_irq(NVIC_USART2_IRQ);

	uartGpioSetup();
	usart_enable(MY_USART_DEVICE);
}


/**
 * @brief This function configures the UART device as well as its transmit and receive buffers
 * @param uartDev uartDev UartDev_T* pointer to USART device
 * @param uartBase uint32_t USART base address
 * @param rxBuffer uint8_t* pointer to the receive buffer
 * @param txBuffer uint8_t* pointer to the transmit buffer
 * @param size uint8_t data size
 */

void uartDevConfig(UartDev_T *uartDev, uint32_t uartBase, uint8_t *rxBuffer, uint8_t *txBuffer, uint8_t size)
{
	if(uartDev == NULL)
	{
		/* Report invalid device pointer*/
	}
	else if (txBuffer == NULL)
	{
		/* Report invalid buffer pointer*/
	}
	else if (rxBuffer == NULL)
	{
		/* Report invalid buffer pointer*/
	}
	else
	{
		uartDev->uartBase = uartBase;
		uartDev->uartRxBuffer = rxBuffer;
		uartDev->uartTxBuffer = txBuffer;
		uartDev->uartRxFlag = UART_NO_RX;
		uartDev->uartTxFlag = UART_NO_TX;
		uartDev->size = size;

		serial_debug_setup();
	}
}


/**
 * @brief Task to handle every incomming commands from the console
 * @param uartDev UartDev_T* pointer to uart device
 * */
void UartHandleCmd_Task(UartDev_T *uartDev)
{
	int8_t arg = 0;
	if(uartDev->uartRxFlag == UART_RX_CMP)
	{
		char *args[16];
		char *token;
		uint8_t i;
		for(i = 0; i < 16; i++)
		{
			args[i] = "";
		}
		token = strtok((char*)uartDev->uartRxBuffer, " ");
		i = 0;
		while(token != NULL)
		{
			args[i] = token;
			i++;
			token = strtok(NULL, " ");
		}

		if((strcasecmp(args[0], (const char*)uartCmdList[HELP])) == 0)
		{
			printMenu();
		}
		else if((strcasecmp(args[0], (const char*)uartCmdList[SET_LB])) == 0)
		{
			arg = atoi(args[1]);
			if(arg < 0 && arg > 31)
			{
				serial_trace("Invalid argument\n");
			}
			else 
			{
				serial_trace("Long Term Drift Sample set to: %d\n", arg);
			}
		}
		else
		{
			/* Report invalid command*/
			serial_trace("Invalid CMD %s\n", uartDev->uartRxBuffer);
			printMenu();
		}
		/* Reset the flag*/
		uartDev->uartRxFlag = UART_NO_RX;
	}
}


/**
 * @brief USART2 interrupt service routine handling every incoming byte from the console
 * 	      Once a command is fully received a flag is set to notify.
 * */
void usart2_isr(void)
{
	volatile uint8_t rcv_char = '\0';
	if(usart_get_flag(myUartDev.uartBase, USART_SR_RXNE))
	{
		rcv_char = usart_recv(myUartDev.uartBase);

		if(rcv_char != '\r')
		{
			myUartDev.uartRxBuffer[isrCnt] = rcv_char;
			isrCnt++;
		}
		else /* If the enter character is received*/
		{
			myUartDev.uartRxFlag = UART_RX_CMP;
			isrCnt = 0;
		}
	}
}


/**
 * Private functions
 *
*/


void ascii_uart_puts_n(char* buf, uint16_t len)
{
	uint16_t i;
	for (i = 0; i < len; i++) {
		if (buf[i] == '\n') {
			usart_send_blocking(myUartDev.uartBase, '\r');
		}
		usart_send_blocking(myUartDev.uartBase, buf[i]);
	}
}


void serial_trace(const char *format, ... )
{
	char str_buffer[100];
	
	va_list args;
	va_start(args, format);
	uint16_t len = vsprintf(str_buffer, format, args);
	ascii_uart_puts_n(str_buffer, len);
	
	va_end(args);
}


void printMenu(void)
{
    serial_trace(
		  "############################################################\t\n"
          "*\tWelcome to the test program *   *\t\n"
          "*\t- Type *HELP* to print this menu                   *\t\n"
          "############################################################\t\n"
          "\n"
          "*\t- This is just a test			               *\t\n"
          "\n############################################################\t\n"
    );
}