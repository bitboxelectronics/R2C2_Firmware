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

#include "app_config.h"
#include "lw_io.h"
#include "gcode_parse.h"
#include "gcode_task.h"
#include "sermsg.h"
#include "temp.h"
#include "LiquidCrystal.h"

#include "keypad_gen4_mb.h"

#include "ui_menu.h"

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

eMenu menu;
eHostState host_state;
eOperatorAttention attention_status;

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef enum {
  DISTANCE_TINY,
  DISTANCE_SHORT,
  DISTANCE_MEDIUM,
  DISTANCE_LONG
  } eJogDistance;


// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

static tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;

static int32_t timer_ticks;
static LW_FILE *lcdf;

// monitor menu

// jog menu
static eJogDistance jog_distance = DISTANCE_SHORT;

static int menu_index;
static int menu_top;

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

static void jog (uint8_t key_pressed) 
{
  float dist;
  uint8_t axis;
  tTarget target;

  if (GcodeInputMsg.in_use)
    return;

  GcodeInputMsg.type = GC_PACKED;

  target.feed_rate = 1200;
  target.invert_feed_rate = false;

  switch(jog_distance) 
  {
    case DISTANCE_TINY:
      dist = 0.1;
    break;
    case DISTANCE_SHORT:
      dist = 1.0;
    break;
    case DISTANCE_MEDIUM:
      dist = 10.0;
    break;
    case DISTANCE_LONG:
      dist = 50.0;
    break;
  }

  switch(key_pressed) 
  {
    case KEY_X_MINUS:
      axis = 'X';
      dist = -dist;
    break;

    case KEY_X_PLUS:
      axis = 'X';
    break;

    case KEY_Y_MINUS:
      axis = 'Y';
      dist = -dist;
    break;

    case KEY_Y_PLUS:
      axis = 'Y';
    break;

    case KEY_Z_MINUS:
      axis = 'Z';
      dist = -dist;
    break;

    case KEY_Z_PLUS:
      axis = 'Z';
    break;
  }

  plan_set_feed_rate (&target);

  LineBuf.len = 0;
  gcode_add_packed_command_int (&LineBuf, 'G', 91);
  gcode_add_packed_command (&LineBuf, CODE_END_COMMAND);

  gcode_add_packed_command_int (&LineBuf, 'G', 1);
  gcode_add_packed_command_float (&LineBuf, axis, dist);
  gcode_add_packed_command (&LineBuf, CODE_END_COMMAND);

  gcode_add_packed_command_int (&LineBuf, 'G', 90);
  gcode_add_packed_command (&LineBuf, CODE_END_COMMAND);

  GcodeInputMsg.in_use = 1;
  tGcodeInputMsg *p_message = &GcodeInputMsg; 
  xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);

}


static void display_sd_entries (void)
{

}


// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief enter a new menu/screen
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------
void menu_enter (eMenu new_menu)
{
  menu = new_menu;

  switch (menu)
  {
    case menu_splash:
      // splash screen
      lcd_begin (20, 4);
      
      lcd_print ("RepRap Control Panel");
      lcd_setCursor (0,2); lcd_print ("Initialising");

      // all leds on
      for (int j=0; j < NUM_LEDS; j++)
      {
        write_pin (config.interface_cp_led_pin[j], 1);
      }
      timer_ticks = 0;

    break;

    case menu_monitor:
      lcd_clear();
      switch (host_state)
      {
        case hs_ready:
          lcd_print ("Ready");
          break;
        case hs_running:
          lcd_print ("Printing");
          // lcd_print ("<file>");
          break;
        case hs_error:
          lcd_print ("Fault");
          break;
      }

      lcd_setCursor (0,2);
      lcd_print ("Tool: ---/---\xdf");
      lcd_setCursor (0,3);
      lcd_print ("Bed : ---/---\xdf");
      
    break;


    case menu_jog:
      lcd_clear();
      lcd_print ("Jog mode: ");

      switch (jog_distance)
      {
        case DISTANCE_TINY:
          lcd_print ("TINY ");
          break;
        case DISTANCE_SHORT:
          lcd_print ("SHORT");
          break;
        case DISTANCE_MEDIUM:
          lcd_print ("MED. ");
          break;
        case DISTANCE_LONG:
          lcd_print ("LONG ");
          break;
      }

      lcd_setCursor (0,1);
      lcd_print ("  Y+          Z+");

      lcd_setCursor (0,2);
      lcd_print ("X-  X+    (mode)");

      lcd_setCursor (0,3);
      lcd_print ("  Y-          Z-");

    break;

    case menu_sd_select:
      menu_index = 0;
      menu_top = 0;
      // display entries, pointer
      lcd_clear();
      lcd_print ("Select file: ");

      display_sd_entries ();
    break;

  }
}

// --------------------------------------------------------------------------
//! @brief update current menu/display
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------

void menu_poll (void)
{
  timer_ticks++;
  
  switch (menu)
  {
    case menu_splash:

      if (timer_ticks == 10)
      {
        // all leds off
        for (int j=0; j < NUM_LEDS; j++)
        {
          write_pin (config.interface_cp_led_pin[j], 0);
        }

        menu_enter (menu_monitor);
      }
    break;

    case menu_monitor:
    {
        uint16_t val;

        val = temp_get (EXTRUDER_0);
        lcd_setCursor (6,2);
        fserwrite_int32_wz (lcdf, val, 3, 0);

        val = temp_get_target (EXTRUDER_0);
        lcd_setCursor (10,2);
        fserwrite_int32_wz (lcdf, val, 3, 0);

        val = temp_get (HEATED_BED_0);
        lcd_setCursor (6,3);
        fserwrite_int32_wz (lcdf, val, 3, 0);

        val = temp_get_target (HEATED_BED_0);
        lcd_setCursor (10,3);
        fserwrite_int32_wz (lcdf, val, 3, 0);
    }
    break;
  

    case menu_jog:
      lcd_setCursor (10, 0);

      switch (jog_distance)
      {
        case DISTANCE_TINY:
          lcd_print ("TINY ");
          break;
        case DISTANCE_SHORT:
          lcd_print ("SHORT");
          break;
        case DISTANCE_MEDIUM:
          lcd_print ("MED. ");
          break;
        case DISTANCE_LONG:
          lcd_print ("LONG ");
          break;
      }
    break;

    case menu_sd_select:
    break;
  }
}

void menu_keyhandler (uint8_t key_pressed)
{
  
  if (key_pressed == KEY_CANCEL)
    reboot();

  switch (menu)
  {
    case menu_splash:
      // no-op
      break;

    case menu_monitor:
      if (key_pressed == KEY_ZERO)
        menu_enter (menu_jog);
      break;

    case menu_jog:
      switch (key_pressed)
      {
        case KEY_Y_MINUS:
        case KEY_Z_MINUS:
        case KEY_Y_PLUS:
        case KEY_Z_PLUS:
        case KEY_X_MINUS:
        case KEY_X_PLUS:
          jog(key_pressed);
        break;

        case KEY_OK:
          jog_distance++;
          if (jog_distance > DISTANCE_LONG)
            jog_distance = 0;
        break;

        case KEY_ZERO:
//        case KEY_CANCEL:
          // prev screen
          menu_enter (menu_monitor);
          break;
      }
    break;

    case menu_sd_select:
    break;
  }
}

void menu_init()
{
    LiquidCrystal_4bit (PACKED_PORT_BIT(config.interface_cp_lcd_pin_rs), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_en), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[4]), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[5]), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[6]), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[7]) ); 

    GcodeInputMsg.pLineBuf = &LineBuf;
    GcodeInputMsg.out_file = NULL;
    GcodeInputMsg.result = PR_OK;
    GcodeInputMsg.in_use = 0;

    LineBuf.len = 0;

    timer_ticks = 0;
    lcdf = lw_fopen ("lcd", "w");

    menu_enter (menu_splash);
}

