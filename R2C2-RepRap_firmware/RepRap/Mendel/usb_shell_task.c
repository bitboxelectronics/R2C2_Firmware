
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "r2c2.h"

#include "gcode_parse.h"
#include "gcode_task.h"
#include "serial_fifo.h"
#include "uart.h"
#include "usb.h"

#include "usb_shell_task.h"

#define DBG uart_writestr

static volatile tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;

void USBShellTask( void *pvParameters )
{
    (void) pvParameters; /* Just to prevent compiler warnings about the unused parameter. */
    uint8_t c;
    eParseResult parse_result;

    // TASK INIT
    USBSerial_Init();

    GcodeInputMsg.pLineBuf = &LineBuf;

    // say hi to host
    serial_writestr("Start\r\nOK\r\n");

    // TASK BODY

    // process received data (USB stuff is done inside interrupt)
    for( ;; )
    {
        // process characters from the serial port
        while (!LineBuf.seen_lf && (serial_rxchars() != 0) )
        {
          c = serial_popchar();
      
          if (LineBuf.len < MAX_LINE)
            LineBuf.data [LineBuf.len++] = c;

          if ((c==10) || (c==13))
          {
            if (LineBuf.len > 1)
            {
              LineBuf.seen_lf = 1;
              xQueueSend (GcodeRxQueue, &GcodeInputMsg, portMAX_DELAY);
            }
            else
              LineBuf.len = 0;
          }      
        }

    }
}

