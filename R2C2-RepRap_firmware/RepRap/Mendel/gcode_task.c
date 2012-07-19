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
#include "stepper.h"
#include "gcode_parse.h"
#include "gcode_process.h"
#include "gcode_task.h"

#define QUEUE_LEN 4

xQueueHandle  GcodeRxQueue = NULL;

tLineBuffer   sd_line_buf;

// static xQueueHandle TxQueue = NULL;

static tGcodeInputMsg GcodeInputMsg;

void GcodeTask( void *pvParameters )
{
    (void) pvParameters; /* Just to prevent compiler warnings about the unused parameter. */
    eParseResult parse_result;

    // TASK INIT
    /* Initialize Gcode parse variables */
    gcode_parse_init();

    //TODO: set up default feedrate?

    // grbl init
    plan_init();
    st_init();


    GcodeRxQueue = xQueueCreate( QUEUE_LEN, sizeof( tGcodeInputMsg ) );
    if( GcodeRxQueue == NULL )
    {
        /* Not enough heap available to create the buffer queues, can't do
           anything so just delete ourselves. */
        vTaskDelete( NULL );
    }

    // TASK BODY

    // process received data
    for( ;; )
    {
        if (uxQueueMessagesWaiting(GcodeRxQueue) > 0)
        {
            if (xQueueReceive (GcodeRxQueue, &GcodeInputMsg, portMAX_DELAY))
            {
                // if queue is full, we wait
                while (plan_queue_full())
                    /* wait */ ;

                parse_result = gcode_parse_line (GcodeInputMsg.pLineBuf);

                GcodeInputMsg.pLineBuf->len = 0;
                GcodeInputMsg.pLineBuf->seen_lf = 0;
            }
        }

        // process SD file if no serial command pending
        if (!sd_line_buf.seen_lf && sd_printing)
        {
          if (sd_read_file (&sd_line_buf))
          {
              sd_line_buf.seen_lf = 1;

              parse_result = gcode_parse_line (&sd_line_buf);

              sd_line_buf.len = 0;
              sd_line_buf.seen_lf = 0;

          } 
          else
          {
            sd_printing = false;
            lw_puts ("Done printing file\r\n");
          }
        }

    } // for ()
}

