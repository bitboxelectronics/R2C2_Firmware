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
//#include "pinout.h"
#include "pin_control.h"
#include "debug.h"
#include "app_config.h"
#include "temp.h"
#include "planner.h"
#include "stepper.h"

#include "eth_shell_task.h"
#include "usb_shell_task.h"
#include "gcode_task.h"


tTimer temperatureTimer;

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate = 200Khz */
}

void ctc_init (tCtcSettings *ctc_config)
{
  PINSEL_CFG_Type PinCfg;

  // heater output pin
  set_pin_mode (ctc_config->pin_heater, OUTPUT);
  write_pin (ctc_config->pin_heater, DISABLE);

  // fan output pin
  set_pin_mode(ctc_config->pin_cooler, OUTPUT);
  write_pin (ctc_config->pin_cooler, DISABLE);


  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = ctc_config->pin_temp_sensor.port;
  PinCfg.Pinnum = ctc_config->pin_temp_sensor.pin_number;
  PINSEL_ConfigPin(&PinCfg);

}

void io_init(void)
{
  int axis;

  /* Extruder 0 */
  ctc_init (&config.extruder_ctc[0]);

  //TODO: extruder 1

  /* Heated Bed */
  ctc_init (&config.heated_bed_ctc);

  /* setup stepper axes */
  set_pin_mode (config.pin_all_steppers_reset, OUTPUT);
  write_pin (config.pin_all_steppers_reset, DISABLE); /* Disable reset for all stepper motors */

  for (axis = 0; axis < MAX_AXES; axis++)
  {
    if (config.axis [axis].is_configured)
    {
      set_pin_mode (config.axis [axis].pin_step, OUTPUT);
      set_pin_mode (config.axis [axis].pin_dir, OUTPUT);
      set_pin_mode (config.axis [axis].pin_enable, OUTPUT);
      
      set_pin_mode (config.axis [axis].pin_min_limit, INPUT);
    
      axis_enable(axis);
    }
  }
  

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

  // NB Anything before read_config call must not rely on anything in config!
  // read_config must not use any peripherals apart from SPI?
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
  xTaskCreate( USBShellTask, (signed char *)"USBSh", 128, ( void * ) NULL, tskIDLE_PRIORITY, NULL );
  xTaskCreate( EthShellTask, (signed char *)"EthSh", 128, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* should not get here */
}


