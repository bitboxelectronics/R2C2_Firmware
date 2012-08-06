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

#include <string.h>

#include "rtos_api.h"

#include "r2c2.h"
#include "gcode_parse.h"
#include "gcode_task.h"
#include "lw_io.h"
#include "lw_ioctl.h"
#include "ui_task.h"

#include "LiquidCrystal.h"


#define PIN_LCD_ENABLE  ENCODE_PORT_BIT(1,14)
#define PIN_LCD_RW      0xff
#define PIN_LCD_RS      ENCODE_PORT_BIT(0,25)

#define PIN_LCD_D0      ENCODE_PORT_BIT(1,9)
#define PIN_LCD_D1      ENCODE_PORT_BIT(0,7)
#define PIN_LCD_D2      ENCODE_PORT_BIT(1,0)
#define PIN_LCD_D3      ENCODE_PORT_BIT(0,6)


#define NUM_LEDS 3


static uint8_t led_pins [NUM_LEDS] = {
  ENCODE_PORT_BIT(0,8),
  ENCODE_PORT_BIT(1,31),
  ENCODE_PORT_BIT(1,15)
};


static tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;

static  uint32_t currentSecond, lastSecond;


static void _task_init (void *pvParameters)
{
    GcodeInputMsg.pLineBuf = &LineBuf;
    GcodeInputMsg.out_file = NULL;
    GcodeInputMsg.result = PR_OK;
    GcodeInputMsg.in_use = 0;

    LineBuf.len = 0;

    //
    // Set LED pins as outputs and turn LEDs off
    for (int j=0; j < NUM_LEDS; j++)
    {
      pinMode(led_pins[j], OUTPUT);
      digitalWrite (led_pins[j], 0);
    }

    
    LiquidCrystal_4bit (PIN_LCD_RS, PIN_LCD_ENABLE, 
      PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3); 

    /*LiquidCrystal_8bit_rw (PIN_LCD_RS, PIN_LCD_RW, PIN_LCD_ENABLE, 
      PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3,
      PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7); 
    */

    begin (20, 4);
    print ("RepRap Control Panel");
    setCursor (0,2); print ("Ready");

}

static void _task_poll (void *pvParameters)
{
    int num;
    uint8_t c;
    eParseResult parse_result;

#if 0
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
        // get position
        strcpy (LineBuf.data, "M114");
        LineBuf.len = strlen (LineBuf.data); 

        GcodeInputMsg.in_use = 1;
        tGcodeInputMsg *p_message = &GcodeInputMsg; 
        xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);
      }
    }
#endif

    // Toggle LED once per second
    currentSecond = millis() / 1000;

    if (currentSecond != lastSecond)
    {
      int val = lastSecond % 3;

      digitalWrite (led_pins[val], 0);

      lastSecond = currentSecond;

      val = lastSecond % 3;
      digitalWrite (led_pins[val], 1);

    }

}

void ui_task ( void *pvParameters )
{
  // TASK INIT
  
  _task_init(NULL);

  // TASK BODY

  // process received data (USB stuff is done inside interrupt)
  for( ;; )
  {
    _task_poll (NULL);
  }
}

