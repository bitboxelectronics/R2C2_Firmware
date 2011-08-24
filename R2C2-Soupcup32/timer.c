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

#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"
#include "lpc_types.h"
// #include "ios.h"
#include "sdcard.h"
// #include "buzzer.h"

unsigned char clock_counter_250ms = 0;
unsigned char clock_counter_1s = 0;
volatile unsigned char clock_flag = 0;
long millis_ticks;
volatile uint8_t buzzer_state = 0;
uint16_t duration;
void (*tick_handler)(void) = 0;

void TIMER0_IRQHandler(void)
{
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
	if (tick_handler)
		tick_handler();

}

/* Ticks are in 1/100MHz = 10ns ticks, minimum of 200 ticks = 2us
 * Maximum time of 2^32*10ns = 42.94967296 seconds. */
void setTimer(uint32_t ticks)
{
  LPC_TIM0->MR0 = ticks;
}

void setupTimerInterrupt(void)
{
  TIM_TIMERCFG_Type TIM_ConfigStruct;
  TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

  // Prescale in absolute value
  TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_TICKVAL;
  TIM_ConfigStruct.PrescaleValue  = 1;
  TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);

  /* Configure Timer0 to have the same clock as CPU: 100MHz */
  CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_1);

  // use channel 0, MR0
  TIM_MatchConfigStruct.MatchChannel = 0;
  // Enable interrupt when MR0 matches the value in TC register
  TIM_MatchConfigStruct.IntOnMatch   = 1;
  //Enable reset on MR0: TIMER will reset if MR0 matches it
  TIM_MatchConfigStruct.ResetOnMatch = 1;
  //Do not stop on MR0 if MR0 matches it
  TIM_MatchConfigStruct.StopOnMatch  = 0;
  //Do not toggle MR0.0 pin if MR0 matches it
  TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
  // Set Match value, count value of 100000000 (100000000 * 10ns = 100000000ns = 1s --> 1 Hz)
  TIM_MatchConfigStruct.MatchValue   = 1000;
  TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

  /* Set to have highest priority = 0 */
  NVIC_SetPriority(TIMER0_IRQn, 1);
}

void enableTimerInterrupt(void)
{
  /* Enable interrupt for timer 0 */
  NVIC_EnableIRQ(TIMER0_IRQn);

  /* Start timer 0 */
  TIM_Cmd(LPC_TIM0, ENABLE);
}

void disableTimerInterrupt(void)
{
  /* Stop timer 0 */
  TIM_Cmd(LPC_TIM0, DISABLE);

  /* Clear timer 0 pending interrupt */
  TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

  /* Disable interrupt for timer 0 */
  NVIC_DisableIRQ(TIMER0_IRQn);
}

uint8_t timerInterruptIsEnabled(void)
{
  return NVIC->ISER[((uint32_t)(TIMER0_IRQn) >> 5)] & (1 << ((uint32_t)(TIMER0_IRQn) & 0x1F));
}

void SysTickTimer_Init(void)
{
  // Setup SysTick Timer to interrupt at 1 msec intervals
  // Lowest priority = 31
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    while (1);  // Capture error
  }
}

//  SysTick_Handler happens every 1/1000 second
void SysTick_Handler(void)
{
  millis_ticks++;

	#ifdef	_SDCARD_H
		static uint8_t counter = 0;

		/* 100ms tick for SDCard ***********************************************/
		counter++;
		if (counter > 99)
		{
			MMC_disk_timerproc();
			counter = 0;
		}
		/***********************************************************************/
	#endif

	#ifdef	_BUZZER_H
	/* Buzzer soft timer ***************************************************/
		if (buzzer_state) /* if buzzer in on... */
		{
			if (!--duration) /* decrement and test the duration time that buzzer should be on */
			{
				buzzer_pwm_stop();
				buzzer_state = 0;
			}
		}
	/***********************************************************************/
	#endif
}

long millis(void)
{
  return millis_ticks;
}

void delay_ms(int ms)
{
  int start = millis();
  while (millis() - start <= ms);
}

void delayMicrosecondsInterruptible(int us)
{
  (void) us;
  /* todo: implement this delay for LPC17xx */
}

// delay( microseconds )
void delay(int d){
  while (d > 65535) {
          delayMicrosecondsInterruptible(65534);
          d -= 65535;
  }
  delayMicrosecondsInterruptible(d & 0xFFFF);
}

#ifdef	_BUZZER_H
void buzzer_start (uint16_t duration_ms)
{
  duration = duration_ms;
  buzzer_state = 1;
}
#endif