
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
            serial_writestr ("Done printing file\r\n");
          }
        }

    } // for ()
}

