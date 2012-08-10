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

#include "rtos_api.h"

#include "lpc17xx_nvic.h"

/* Application includes */
#include "lw_io.h"
#include "r2c2.h"
#include "uart.h"
#include "soundplay.h"

//TODO:
#ifdef debug
#define DBG_INIT()   uart_init()
#define DBGF(s)   uart3_writestr(s)
#else
#define DBG_INIT()
#define DBGF(s)
#endif

#define USER_FLASH_START 0x10000 /* For USB bootloader */
//#define USER_FLASH_START 0x0 /* No USB bootloader */

extern int app_main (void);

/**********************************************************************/
void startup_delay(void)
{
  for (volatile unsigned long i = 0; i < 500000; i++) { ; }
}

void fatal_error (void)
{
  for( ;; )
  {
    buzzer_play_sync (FREQ_B4, 1000);
    buzzer_play_sync (FREQ_A4, 1000);
  }
}

#ifdef USE_FREERTOS
/**********************************************************************/
/* Called from every tick interrupt */
void vApplicationTickHook( void )
{
  static unsigned long ulTicksSinceLastDisplay = 0;

  ulTicksSinceLastDisplay++;

  r2c2_SysTick();
}

/**********************************************************************/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
  /* This function will get called if a task overflows its stack. */

  ( void ) pxTask;
  ( void ) pcTaskName;

  //DBGF ("stkov\n");

  fatal_error();
}

#endif

/**********************************************************************
 * @brief	Main sub-routine
 **********************************************************************/
int main(void)
{
  // DeInit NVIC and SCBNVIC
  NVIC_DeInit();
  NVIC_SCBDeInit();

  /* Configure the NVIC Preemption Priority Bits:
   * two (2) bits of preemption priority, six (6) bits of sub-priority.
   * Since the Number of Bits used for Priority Levels is five (5), so the
   * actual bit number of sub-priority is three (3)
   */
   // FreeRTOS requires 0 it seems
  NVIC_SetPriorityGrouping(0x0);

  /* Change the Vector Table to the USER_FLASH_START
  in case the user application uses interrupts */
  SCB->VTOR = (USER_FLASH_START & 0x1FFFFF80);

  DBG_INIT();
  DBGF ("init\n");

#if !defined(USE_FREERTOS)
  SysTickTimer_Init(); // Initialize the timer for millis()
#endif


  // enter the application
  app_main ();

  /* should never get here */
  DBGF ("main:err\n");
  fatal_error();
  while(1) ;
}
