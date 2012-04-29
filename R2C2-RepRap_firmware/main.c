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

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Application includes */
#include "r2c2.h"
#include "uart.h"


#define DBG uart_writestr

#define USER_FLASH_START 0x10000 /* For USB bootloader */
//#define USER_FLASH_START 0x0 /* No USB bootloader */

extern int app_main (void);

/**********************************************************************/
void startup_delay(void)
{
  for (volatile unsigned long i = 0; i < 500000; i++) { ; }
}

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

    DBG ("stkov\n");

	for( ;; );
}

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

#ifdef UARTDEBUG
  uart_init();
  DBG ("init\n");
#endif 

  app_main ();

  /* should never get here */
  DBG ("main:err\n");
  while(1) ;
}