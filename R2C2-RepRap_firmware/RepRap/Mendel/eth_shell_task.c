
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

static uint8_t mac_addr [6] = {0x00, 0x1f, 0x00, 0x01, 0x02, 0x03};

static void NetInit(void)
{
  enc28j60SpiInit();

  enc28j60Init(mac_addr);

  enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
  delay_ms(10);

  // flash leds on
   // 0x880 is PHLCON LEDB=on, LEDA=on
   // enc28j60PhyWrite(PHLCON,0b0011 1000 1000 00 00);
   enc28j60PhyWrite(PHLCON,0x3880);
   delay_ms(100);

   // 0x990 is PHLCON LEDB=off, LEDA=off
   // enc28j60PhyWrite(PHLCON,0b0011 1001 1001 00 00);
   enc28j60PhyWrite(PHLCON,0x3990);

  // 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
  // enc28j60PhyWrite(PHLCON,0b0011 0100 0111 01 10);
  enc28j60PhyWrite(PHLCON,0x3476);
}


void EthShellTask( void *pvParameters )
{
    (void) pvParameters; /* Just to prevent compiler warnings about the unused parameter. */
    uint8_t c;
    eParseResult parse_result;

    // TASK INIT
    NetInit();

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

