/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
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

#include <stdint.h>
#include "lpc17xx_timer.h"
#include "machine.h"
#include "serial.h"
#include "dda_queue.h"
#include "dda.h"
#include "gcode_parse.h"
#include "gcode_process.h"
#include "timer.h"
#include "temp.h"
#include "sermsg.h"
#include "debug.h"
#include "sersendf.h"
#include "ios.h"
#include "adc.h"
#include "pinout.h"
#include "stdlib.h"
#include "debug.h"
#include "config.h"
#include "buzzer.h"

extern volatile uint8_t step_requested;

uint8_t leds_enabled;

uint8_t  led_on;
uint16_t led_on_time;
uint16_t led_off_time;

tTimer blinkTimer;


void io_init(void)
{
  /* Extruder 0 Heater pin */
  pin_mode(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, OUTPUT);
  extruder_heater_off();

  /* Heated Bed 0 Heater pin */
  pin_mode(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, OUTPUT);
  heated_bed_off();

  /* setup I/O pins */
  pin_mode(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, OUTPUT);
  digital_write(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, 1); /* Disable reset for all stepper motors */

  pin_mode(X_STEP_PORT, X_STEP_PIN, OUTPUT);
  pin_mode(X_DIR_PORT, X_DIR_PIN, OUTPUT);
  pin_mode(X_ENABLE_PORT, X_ENABLE_PIN, OUTPUT);
  x_enable();
  pin_mode(X_MIN_PORT, X_MIN_PIN, INPUT);

  pin_mode(Y_STEP_PORT, Y_STEP_PIN, OUTPUT);
  pin_mode(Y_DIR_PORT, Y_DIR_PIN, OUTPUT);
  pin_mode(Y_ENABLE_PORT, Y_ENABLE_PIN, OUTPUT);
  y_enable();
  pin_mode(Y_MIN_PORT, Y_MIN_PIN, INPUT);

  pin_mode(Z_STEP_PORT, Z_STEP_PIN, OUTPUT);
  pin_mode(Z_DIR_PORT, Z_DIR_PIN, OUTPUT);
  pin_mode(Z_ENABLE_PORT, Z_ENABLE_PIN, OUTPUT);
  z_enable();
  pin_mode(Z_MIN_PORT, Z_MIN_PIN, INPUT);

  pin_mode(E_STEP_PORT, E_STEP_PIN, OUTPUT);
  pin_mode(E_DIR_PORT, E_DIR_PIN, OUTPUT);
  pin_mode(E_ENABLE_PORT, E_ENABLE_PIN, OUTPUT);
  e_enable();

  adc_init();
}


void blinkTimerCallback (tTimer *pTimer)
{
  if (leds_enabled)
  {
    led_on = led_on ^ 1;

    if (led_on)
      StartSlowTimer (&blinkTimer, led_on_time, blinkTimerCallback);
    else
      StartSlowTimer (&blinkTimer, led_off_time, blinkTimerCallback);
  }
  else
  {
    led_on = 0;
  }

}

void startBlink(void)
{
  leds_enabled = 1;
  led_on = 0x00;
//  StartSlowTimer (&blinkTimer, led_on_time, blinkTimerCallback);
}

void stopBlink (void)
{
  leds_enabled = 0;
  led_on = 0;
  StopSlowTimer (&blinkTimer);
  unstep();
}

void timerCallback (tHwTimer *pTimer, uint32_t int_mask)
{
  (void)pTimer;

  if (int_mask & _BIT(TIM_MR0_INT))
  {
    // decide which outputs need stepping
    queue_step();
  }

  if (int_mask & _BIT(TIM_MR1_INT))
  { 
    // step the required channels
    if (step_requested & 1)
      x_step();
    if (step_requested & 2)
      y_step();
    if (step_requested & 4)
      z_step();
    if (step_requested & 8)
      e_step();
  }

  if (int_mask & _BIT(TIM_MR2_INT))
  { 
    // turn off step outputs
    if ((led_on & 1) == 0)
    {
      x_unstep();
    }
    if ((led_on & 2) == 0)
    {
      y_unstep();
    }
    if ((led_on & 4) == 0)
    {
      z_unstep();
    }
    if ((led_on & 8) == 0)
    {
      e_unstep();
    }
    // else leave as is (important!)
  }

}

void init(void)
{
  // set up inputs and outputs
  io_init();

  /* Initialize DDA variables */
  dda_init();

  /* Initialize Gcode parse variables */
  gcode_parse_init();

  // set up default feedrate
  current_position.F = startpoint.F = next_target.target.F = \
      config.search_feedrate_z;

  // set up timers
  // we use hardware timer 0
  setupHwTimer(0, timerCallback);
  // Set the Match 1 and Match 2 interrupts
  // The time from Match0 to Match 1 defines the low pulse period of the step output
  // and the time from Match1 to Match 2 defines the minimum high pulse period of the step output-
  // if the LED is in a blink ON period the step output will be left high until next required step
  setHwTimerMatch(0, 1, 500); // Match1 about Match0 + 5 us
  setHwTimerMatch(0, 2, 1000); // Match2, about Match0 + 10 us

  // set the LED blink times, 50 ms on/off = 10 flashes per second
  led_on_time = 50;
  led_off_time = 50;

  AddSlowTimer (&blinkTimer);

  // say hi to host
  serial_writestr("Start\r\nOK\r\n");
}

int main_reprap (void)
{
  long timer1 = 0;

  buzzer_init();
  buzzer_play(1500, 100); /* low beep */
	buzzer_wait();
  buzzer_play(2500, 200); /* high beep */

  init();

  read_config();

  // main loop
  for (;;)
  {
    // if queue is full, no point in reading chars- host will just have to wait
    if ((serial_rxchars() != 0) && (queue_full() == 0))
    {
      unsigned char c = serial_popchar();

      /* Read each char and at end of each line, put the "GCode" on movebuffer.
       * If there are movemnet to do, Timer will start and execute code which
       * will take data from movebuffer and generate the required step pulses
       * for stepper motors.
       */
      gcode_parse_char(c);
    }

    /* Do every 100ms */
    #define DELAY1 100
    if (timer1 < millis())
    {
      timer1 = millis() + DELAY1;

      /* Manage the extruder temperature */
      temp_tick();

      /* If there are no activity during 30 seconds, power off the machine */
      if (steptimeout > (30 * 4))
      {
        power_off();
      }
      else
      {
        steptimeout++;
      }
    }

  }
}
