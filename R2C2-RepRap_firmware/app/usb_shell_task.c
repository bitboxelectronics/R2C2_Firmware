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

#include "rtos_api.h"

#include "r2c2.h"
#include "gcode_parse.h"
#include "gcode_task.h"
#include "lw_io.h"
#include "lw_ioctl.h"
#include "usb_shell_task.h"
#include "uart_shell_task.h"


static tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;
static tShellParams task_params;

static void _task_init (tShellParams *pParameters)
{
    pParameters->in_file = lw_fopen ("usbser", "rw");
    pParameters->out_file = pParameters->in_file;

    GcodeInputMsg.pLineBuf = &LineBuf;
    GcodeInputMsg.out_file = pParameters->out_file;
    GcodeInputMsg.result = PR_OK;
    GcodeInputMsg.in_use = 0;

    // say hi to host
    lw_fprintf(pParameters->out_file, "Start\r\nOK\r\n");
}

static void _task_poll (tShellParams *pParameters)
{
    int num;
    uint8_t c;
    eParseResult parse_result;

    if (!GcodeInputMsg.in_use)
    {
      if (GcodeInputMsg.result == PR_BUSY)
      {
        // try again
        GcodeInputMsg.in_use = 1;
        tGcodeInputMsg *p_message = &GcodeInputMsg; 
        xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);
      }
      else
      {
        lw_ioctl (pParameters->in_file, LW_FIONREAD, &num);

        if (num > 0 )
        {
          c = lw_fgetc (pParameters->in_file);
  
          if (LineBuf.len < MAX_LINE)
            LineBuf.data [LineBuf.len++] = c;

            if ((c==10) || (c==13))
            {
              if (LineBuf.len > 1)
              {
                GcodeInputMsg.in_use = 1;
                tGcodeInputMsg *p_message = &GcodeInputMsg; 
                xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);
              }
              else
                LineBuf.len = 0;
            }
        }      
      }
    }
}

void USBShellTask( void *pvParameters )
{
  // TASK INIT
  
  if (pvParameters != NULL)
    task_params = *(tShellParams *)pvParameters;

  _task_init(&task_params);

  // TASK BODY

  // process received data (USB stuff is done inside interrupt)
  for( ;; )
  {
    _task_poll(&task_params);
  }
}

