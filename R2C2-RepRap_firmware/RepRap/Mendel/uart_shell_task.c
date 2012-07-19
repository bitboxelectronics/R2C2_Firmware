/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "r2c2.h"

#include "gcode_parse.h"
#include "gcode_task.h"

#include "lw_io.h"
#include "uart.h"

#include "uart_shell_task.h"


static volatile tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;

void uart_shell_task ( tShellParams *pParameters )
{
    uint8_t c;
    int uart_num = pParameters->uart_num;
    eParseResult parse_result;

    // TASK INIT

    GcodeInputMsg.pLineBuf = &LineBuf;

    uart_init (uart_num);

    // say hi to host
    lw_printf("Start\r\nOK\r\n");

    // TASK BODY

    // process received data
    for( ;; )
    {
        // process characters from the serial port
        while (!LineBuf.seen_lf && (uart_data_available(uart_num) != 0) )
        {
          c = uart_receive(uart_num);
      
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

