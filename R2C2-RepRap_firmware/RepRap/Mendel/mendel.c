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
#include <stdlib.h>

#include "lpc17xx_timer.h"
#include "lpc17xx_wdt.h"
#include "lpc17xx_adc.h"
#include "r2c2.h"

#include "machine.h"
#include "dda_queue.h"
#include "dda.h"
#include "gcode_parse.h"
#include "gcode_process.h"
#include "pinout.h"
#include "debug.h"
#include "config.h"
#include "temp.h"

#ifdef USE_GRBL
#include "planner.h"
#include "stepper.h"
#endif

extern volatile uint8_t step_requested;

uint8_t leds_enabled;

uint8_t  led_on;
uint16_t led_on_time;
uint16_t led_off_time;

tTimer blinkTimer;
tTimer temperatureTimer;

tLineBuffer serial_line_buf;
tLineBuffer sd_line_buf;

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
  PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = HEATED_BED_0_ADC_PORT;
  PinCfg.Pinnum = HEATED_BED_0_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate = 200Khz */
}

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

  pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);

  adc_init();
}

void temperatureTimerCallback (tTimer *pTimer)
{
  temp_tick();
}

void blinkTimerCallback (tTimer *pTimer)
{
  if (leds_enabled)
  {
    led_on = led_on ^ 0x0F;

    if (led_on)
      StartSlowTimer (&blinkTimer, led_on_time, blinkTimerCallback);
    else
      StartSlowTimer (&blinkTimer, led_off_time, blinkTimerCallback);
  }
  else
  {
    led_on = 0x00;
  }

}

void startBlink(void)
{
  leds_enabled = 1;
#ifdef STEP_LED_FLASH_FIXED  
  StartSlowTimer (&blinkTimer, led_on_time, blinkTimerCallback);
  led_on = 0x0F;
#else
  led_on = 0x00;
#endif
}

void stopBlink (void)
{
  leds_enabled = 0;
  led_on = 0x00;
  StopSlowTimer (&blinkTimer);
  unstep();
}

void timerCallback (tHwTimer *pTimer, uint32_t int_mask)
{
  (void)pTimer;

  if (int_mask & _BIT(TIM_MR0_INT))
  {
    // decide which outputs need stepping
//!    queue_step();
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
#ifdef STEP_LED_NONE
    // turn off all step outputs
    unstep();
#elif !defined(STEP_LED_ON_WHEN_ACTIVE)
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
#endif
  }

}

void check_boot_request (void)
{
  if (digital_read (4, (1<<29)) == 0)
  {
    WDT_Init (WDT_CLKSRC_PCLK, WDT_MODE_RESET);
    WDT_Start (10);
    while (1);
  }
}

void init(void)
{
  // set up inputs and outputs
  io_init();

#ifndef USE_GRBL
  /* Initialize DDA variables */
  dda_init();
#endif

  /* Initialize Gcode parse variables */
  gcode_parse_init();

#ifndef USE_GRBL
  // set up default feedrate
  current_position.F = startpoint.F = next_target.target.F = \
      config.search_feedrate_z;
#endif

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

  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

  // say hi to host
  serial_writestr("Start\r\nOK\r\n");
}

int app_main (void)
{
  long timer1 = 0;
  eParseResult parse_result;

  buzzer_init();
  buzzer_play(1500, 100); /* low beep */
	buzzer_wait();
  buzzer_play(2500, 200); /* high beep */

  init();

  read_config();

#ifdef USE_GRBL
  // grbl init
  plan_init();      
  st_init();    
#endif
  
  // main loop
  for (;;)
  {

    // process characters from the serial port
    while (!serial_line_buf.seen_lf && (serial_rxchars() != 0) )
    {
      unsigned char c = serial_popchar();
      
      if (serial_line_buf.len < MAX_LINE)
        serial_line_buf.data [serial_line_buf.len++] = c;

      if ((c==10) || (c==13))
      {
        if (serial_line_buf.len > 1)
          serial_line_buf.seen_lf = 1;
        else
          serial_line_buf.len = 0;
      }      
    }

    // process SD file if no serial command pending
    if (!sd_line_buf.seen_lf && sd_printing)
    {
      if (sd_read_file (&sd_line_buf))
      {
          sd_line_buf.seen_lf = 1;
      } 
      else
      {
        sd_printing = false;
        serial_writestr ("Done printing file\r\n");
      }
    }

    // if queue is full, we wait
    if (!plan_queue_full())
    {
  
      /* At end of each line, put the "GCode" on movebuffer.
       * If there are movement to do, Timer will start and execute code which
       * will take data from movebuffer and generate the required step pulses
       * for stepper motors.
       */
  
      // give priority to user commands
      if (serial_line_buf.seen_lf)
      {
        parse_result = gcode_parse_line (&serial_line_buf);
        serial_line_buf.len = 0;
        serial_line_buf.seen_lf = 0;
      }
      else if (sd_line_buf.seen_lf)
      {
        parse_result = gcode_parse_line (&sd_line_buf);
        sd_line_buf.len = 0;
        sd_line_buf.seen_lf = 0;
      }

    }

    /* Do every 100ms */
    #define DELAY1 100
    if (timer1 < millis())
    {
      timer1 = millis() + DELAY1;

      /* Manage the extruder temperature */
//      temp_tick();

      /* If there are no activity during 30 seconds, power off the machine */
      if (steptimeout > (30 * 1000/DELAY1))
      {
        power_off();
      }
      else
      {
        steptimeout++;
      }
    }

#ifdef USE_BOOT_BUTTON
    // OPTION: enter bootloader on "Boot" button
    check_boot_request();
#endif

  }
}
