
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "r2c2.h"
#include "usb.h"
#include "serial_fifo.h"
#include "uart.h"

#define usbBUFFER_LEN			( 20 )

#define DBG uart_writestr

static xQueueHandle RxQueue = NULL;
static xQueueHandle TxQueue = NULL;

void USBShellTask( void *pvParameters )
{
	uint8_t c;
	( void ) pvParameters; /* Just to prevent compiler warnings about the unused parameter. */

    // TASK INIT

	DBG("Initialising USB shell\n");
//    buzzer_play(2500, 200); /* high beep */

#if 0
	RxQueue = xQueueCreate( usbBUFFER_LEN, sizeof( uint8_t ) );
	TxQueue = xQueueCreate( usbBUFFER_LEN, sizeof( uint8_t ) );

	if( ( RxQueue == NULL ) || ( TxQueue == NULL ) )
	{
		/* Not enough heap available to create the buffer queues, can't do
		anything so just delete ourselves. */
		vTaskDelete( NULL );
	}
#endif
	
	DBG("Starting USB shell\n");
    USBSerial_Init();

    // TASK BODY

	// process received data (USB stuff is done inside interrupt)
	for( ;; )
	{
        fifo_get(&rxfifo, &c);

		if (c != 0) 
		{
			// Echo character
			fifo_put (&txfifo, c);
		}
	}
}

