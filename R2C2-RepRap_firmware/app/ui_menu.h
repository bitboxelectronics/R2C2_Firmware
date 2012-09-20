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


#ifndef _UI_MENU_H
#define _UI_MENU_H

#define NUM_LEDS 3


// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef enum {
  menu_splash,
  menu_monitor,
  menu_jog,
  menu_homing,
  menu_sd_select
  } eMenu;

// 
typedef enum {
// idle/standby states
  hs_ready,
   
// running states
  hs_wait_user,   // paused/feed hold
                  // tool change/gcode
  hs_wait_temp,   // running - wait
  hs_wait_time,
  hs_running,
//  hs_run_from_sd, ??

// stopped states
  hs_error
  } eHostState;

typedef enum {
  led_off, 
  led_flash,
  led_on 
  } eLedState;

typedef enum {
  oa_ok,
  oa_user_requested_hold,
  oa_system_requested_hold
  } eOperatorAttention;

// home required
// tool change
// feed hold
// gcode pause?

extern eMenu menu;

extern eHostState host_state;
extern eOperatorAttention attention_status;


void menu_init (void);
void menu_enter (eMenu new_menu);
void menu_poll (void);
void menu_keyhandler (uint8_t key_pressed);

#endif
