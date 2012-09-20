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
#include "app_config.h"
#include "ui_task.h"
#include "keypad_gen4_mb.h"
#include "ui_menu.h"

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// leds
#define LED_OK        0   // green
#define LED_ATTENTION 1   // yellow
#define LED_FAULT     2   // red

// menu refresh rate
#define UPDATE_TIME   200   // ms

// keys
#define KEY_POLL_TIME 20   // ms

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------


static int32_t start_time;
static int32_t led_ticks;

static eLedState led_state [NUM_LEDS];

// keys
static uint16_t key_last_state;
static uint16_t key_state;

static uint8_t key_timer [MAX_BUTTONS];
static uint8_t key_event [MAX_BUTTONS];


// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

static void _task_init (void *pvParameters)
{

    //
    // Set LED pins as outputs
    for (int j=0; j < NUM_LEDS; j++)
    {
      set_pin_mode (config.interface_cp_led_pin[j], OUTPUT);
    }

    led_ticks = 0;

    attention_status = oa_system_requested_hold; // todo
    host_state = hs_ready;

    // all leds on
    for (int j=0; j < NUM_LEDS; j++)
    {
      led_state [j] = led_on;
    }

    start_time = millis();

    keypad_init();

    menu_init();
}

static void _task_poll (void *pvParameters)
{
    uint8_t j;

#if 0
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
        // get position
        strcpy (LineBuf.data, "M114");
        LineBuf.len = strlen (LineBuf.data); 

        GcodeInputMsg.in_use = 1;
        tGcodeInputMsg *p_message = &GcodeInputMsg; 
        xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);
      }
    }
#endif


    int now = millis();

    //
    // keypad scan
    //
    if (now - start_time > KEY_POLL_TIME)
    {
      // poll keys
  
      // get current states
      key_state = keypad_get_key_state ();

      // compare to previous
      for (int j=0; j < MAX_BUTTONS; j++)
      {
        if ( (key_last_state & _BV(j)) == 0)
        {
          if ( key_state & _BV(j) )
          {
            // key down, start timer
            key_timer [j] = 100 / KEY_POLL_TIME;
          }
        }
        else // was down
        {
          if ( key_state & _BV(j) )
          { 
             // still down
             if (key_timer [j] > 0)
             {
                key_timer [j] --;
                if (key_timer [j] == 0)
                {
                  key_event[j] = 1;

                  menu_keyhandler (j);
                }
             }
             else
             {
                // already signalled
             }
          }
          else
          {
             // key released
          }
        }
      }

      key_last_state = key_state;
    }

    //
    // timer update polling
    // 
    if (now - start_time > UPDATE_TIME)
    {
// --------------------------------------------------------------------------
      // update display
      menu_poll();

// --------------------------------------------------------------------------
      // update status leds
      led_ticks++;
      if (led_ticks == 10)
        led_ticks = 0;

      //
      if (menu != menu_splash)
      {
        switch (host_state)
        {
          case hs_ready:
            led_state [LED_OK] = led_on;
            led_state [LED_FAULT] = led_off;
            break;

          case hs_running:
            led_state [LED_OK] = led_on;
            led_state [LED_FAULT] = led_off;
            break;

          case hs_error:
            led_state [LED_OK] = led_off;
            led_state [LED_FAULT] = led_on;
            break;
        }

        switch (attention_status)
        {
          case oa_ok:
            led_state [LED_ATTENTION] = led_off;
          break;

          case oa_user_requested_hold:
            led_state [LED_ATTENTION] = led_on;
          break;

          case oa_system_requested_hold:
            led_state [LED_ATTENTION] = led_flash;
          break;
        }
      }


      // ---------------
      // 
      for (int j=0; j < NUM_LEDS; j++)
      {
        switch (led_state[j])
        {
          case led_off:
            write_pin (config.interface_cp_led_pin[j], 0);
            break;
          case led_on:
            write_pin (config.interface_cp_led_pin[j], 1);
            break;
          case led_flash:
            if (led_ticks < 5)
              write_pin (config.interface_cp_led_pin[j], 0);
            else
              write_pin (config.interface_cp_led_pin[j], 1);
            break;
        }
      }

// --------------------------------------------------------------------------

      start_time = now;
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

