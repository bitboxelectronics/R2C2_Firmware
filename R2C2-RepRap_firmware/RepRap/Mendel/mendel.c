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

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "lpc17xx_timer.h"
#include "lpc17xx_wdt.h"
#include "lpc17xx_adc.h"

#include "r2c2.h"
#include "machine.h"
#include "gcode_parse.h"
#include "pinout.h"
#include "debug.h"
#include "config.h"
#include "temp.h"
#include "planner.h"
#include "stepper.h"
#include "usb_shell_task.h"
#include "gcode_task.h"


tTimer temperatureTimer;

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
  /* Manage the temperatures */
  temp_tick();
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

static void PrinterInit (void)
{
  buzzer_init();

  // set up inputs and outputs
  io_init();

  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

}

void PrinterTask( void *pvParameters )
{
  long timer1 = 0;

  read_config();

  buzzer_play(1500, 100); /* low beep */
	buzzer_wait();
  buzzer_play(2500, 200); /* high beep */

  // loop
  for (;;)
  {
    /* Do every 100ms */
    #define DELAY1 100
    if (timer1 < millis())
    {
      timer1 = millis() + DELAY1;

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


void app_main (void)
{
  PrinterInit();

  /* Create the tasks */
  xTaskCreate( PrinterTask,  (signed char *)"Print", 512, ( void * ) NULL, tskIDLE_PRIORITY, NULL );
  xTaskCreate( GcodeTask,    (signed char *)"Gcode", 512, ( void * ) NULL, tskIDLE_PRIORITY, NULL );
  xTaskCreate( USBShellTask, (signed char *)"USBsh", 128, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* should not get here */
}


