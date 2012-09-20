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

#include "rtos_api.h"

#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_wdt.h"
#include "lpc17xx_adc.h"

#include "r2c2.h"
#include "gcode_parse.h"
//#include "pinout.h"
#include "pin_control.h"
#include "debug.h"
#include "app_config.h"
#include "temp.h"
#include "planner.h"
#include "stepper.h"
#include "usb_serial.h"
#include "uart.h"
#include "ff.h"
#include "eth_shell_task.h"
#include "usb_shell_task.h"
#include "uart_shell_task.h"
#include "gcode_task.h"
#include "ui_task.h"

FATFS   fs;       /* Work area (file system object) for logical drive */

tTimer  temperatureTimer;



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
  write_pin (config.pin_all_steppers_reset, DISABLE); /* Disable reset state for all stepper motors */

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

void reboot (void)
{
  NVIC_SystemReset();
}

void check_boot_request (void)
{
  if (digital_read (BOOT_SW_PORT, _BV(BOOT_SW_PIN_NUMBER)) == 0)
  {
    reboot();
  }
}

static void PrinterInit (void)
{
  FRESULT res;

  app_config_set_defaults();

  // initialise some drivers useful for debugging
  buzzer_init();

  // initialize low-level USB serial and UART drivers
  _sys_init_devices();

  // open standard files
  lw_initialise();

  dbg_init();

  /* initialize SPI for SDCard */
  spi_init();

  /* Register a work area for logical drive 0 */
  res = f_mount(0, &fs);
  if (res)
    debug("Err mount fs\n");
  else  
  {
    // read_config will use SPI and a message output (control interface or debug)
    app_config_read();
  }

  // init devices?

  // set up inputs and outputs
  io_init();

  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

}

void PrinterTask( void *pvParameters )
{
  tShellParams shell_params;
  long timer1 = 0;

  PrinterInit();

  // -- init complete, can now start other tasks --

  // GCode engine
  xTaskCreate( GcodeTask,    (signed char *)"Gcode", 512, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

  // GCode control interfaces
  xTaskCreate( USBShellTask, (signed char *)"USBSh", 128, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

  // option
  xTaskCreate( EthShellTask, (signed char *)"EthSh", 128, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

  // option
  // Start a Gcode shell on UART
  shell_params.in_file = lw_fopen ("uart1", "rw");
  shell_params.out_file = shell_params.in_file;
  xTaskCreate( uart_shell_task, (signed char *)"UartSh", 128, ( void * ) &shell_params, tskIDLE_PRIORITY, NULL );

  // start up user interface
  xTaskCreate( ui_task, (signed char *)"UiTask", 256, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

  // now to do GCode startup
  exec_gcode_file ("autoexec.g");

  // -- all startup done, signal readiness

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

      //TODO: there are two types of timeout, 
      // 1: disable steppers when idle to avoid nuisance noise
      // 2: safe power off (steppers, heaters) after being idle for a while, in case machine is unattended and
      //    host has stopped without clean up, or user has just forgot to turn things off.
       
      /* If there are no activity during 30 seconds, power off the machine */
      if (steptimeout > (30 * 1000/DELAY1))
      {
        atx_power_off();
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

/*
  required start sequence:
  //TODO: where to read autoexec.g, set config defaults

  1.  create PrinterTask

  2.  start scheduler 
       [ system tick is now running]

  3.  init buzzer [no config, no debug IO, requires timer]

  4.  init USB CDC-serial [ no config]

  5.  read_config files from SD [requires timer, debug IO]

  6.  can now initialize peripherals: steppers, CTC, ADC

  7.  start temperature monitoring [requires timer, adc]

  8.  start the other tasks [requires above stuff]

  9.  system is now be ready to accept general GCode commands
*/

void app_main (void)
{
  portBASE_TYPE res;
  /* Create the main system task. 
  *  NB: our system timer tick is called from FreeRTOS timer tick, which only runs after scheduler has started.
  *  Therefore, we start only PrinterTask to do initialisation which requires timer, namely the FatFs/SD code.
  */
  //TODO: Check stack usage
  res = xTaskCreate( PrinterTask,  (signed char *)"Print", 512, ( void * ) NULL, tskIDLE_PRIORITY, NULL );
  if (res != pdPASS)
    debug ("error starting PrinterTask\n");

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* should not get here */
}


