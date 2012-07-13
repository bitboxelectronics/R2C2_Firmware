
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "r2c2.h"

#include "gcode_parse.h"
#include "gcode_task.h"
#include "uart.h"
#include "eth_shell_task.h"

#include "enc28j60.h"

#define DBG uart_writestr

static volatile tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;



void EthShellTask( void *pvParameters )
{
    (void) pvParameters; /* Just to prevent compiler warnings about the unused parameter. */
    uint8_t c;
    eParseResult parse_result;

    // TASK INIT
    //NetInit();

    GcodeInputMsg.pLineBuf = &LineBuf;

    // say hi to host
    //serial_writestr("Start\r\nOK\r\n");

    // TASK BODY

    // process received data
    for( ;; )
    {
      digital_write (1, (1<<15), 0);
      delay (250);
      digital_write (1, (1<<15), 1);
      delay (250);
    }
}

