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
#include "lpc17xx_gpio.h"
#include "lpc_types.h"
#include "ios.h"
#include "sdcard.h"
#include "timer.h"


//unsigned char clock_counter_250ms = 0;
//unsigned char clock_counter_1s = 0;
//volatile unsigned char clock_flag = 0;


static volatile long millis_ticks;

static tTimer *SlowTimerHead;
static tTimer *SlowTimerTail;


static tHwTimer HwTimer [NUM_HARDWARE_TIMERS];

static LPC_TIM_TypeDef *pTimerRegs [NUM_HARDWARE_TIMERS] = 
  {LPC_TIM0, LPC_TIM1, LPC_TIM2, LPC_TIM3 };

struct tTimerConfig 
  {
    uint32_t      ClkPwr_PClkSel;  
    uint32_t      TimerIrq;  
  };

static const struct tTimerConfig 
  TimerConfig [NUM_HARDWARE_TIMERS] =
  {
    { CLKPWR_PCLKSEL_TIMER0, TIMER0_IRQn},
    { CLKPWR_PCLKSEL_TIMER1, TIMER1_IRQn},
    { CLKPWR_PCLKSEL_TIMER2, TIMER2_IRQn},
    { CLKPWR_PCLKSEL_TIMER3, TIMER3_IRQn},
  };

static inline void TIMER_IRQHandlerGeneric (uint16_t timerNum)
{
  uint32_t int_mask;

  int_mask = pTimerRegs[timerNum]->IR;

  // clear ALL pending timer interrupts 
  pTimerRegs[timerNum]->IR = int_mask;

  if (HwTimer[timerNum].timerCallback)
      HwTimer[timerNum].timerCallback (&HwTimer[timerNum], int_mask);
}

// IRQ handlers referenced in startup.S

void TIMER0_IRQHandler(void)
{

  TIMER_IRQHandlerGeneric (0);
}

void TIMER1_IRQHandler(void)
{
  TIMER_IRQHandlerGeneric (1);
}

void TIMER2_IRQHandler(void)
{
  TIMER_IRQHandlerGeneric (2);
}

void TIMER3_IRQHandler(void)
{
  TIMER_IRQHandlerGeneric (3);
}

/* Set the timer interval. With default setup,
 * ticks are in 1/100MHz = 10ns ticks, minimum of 200 ticks = 2us
 * Maximum time of 2^32*10ns = 42.94967296 seconds. */
void setHwTimerInterval (uint16_t timerNum, uint32_t ticks)
{
  pTimerRegs[timerNum]->MR0 = ticks;
}

// setup hardware timer timerNum
// timerCallback: if not-NULL, will be called on any timer interrupt. Default is Match0
// For convenience various default values are setup: a default reload/Match0 period of 
// 10us is set, this can be changed by setHwTimerInterval.
void setupHwTimer (uint16_t timerNum, tHwTimerCallback timerCallback)
{
  TIM_TIMERCFG_Type TIM_ConfigStruct;
  TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

  // Prescale in absolute value
  TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_TICKVAL;
  TIM_ConfigStruct.PrescaleValue  = 1;
  TIM_Init (pTimerRegs[timerNum], TIM_TIMER_MODE, &TIM_ConfigStruct);

  /* Configure Timer to have the same clock as CPU: 100MHz */
  CLKPWR_SetPCLKDiv (TimerConfig[timerNum].ClkPwr_PClkSel, CLKPWR_PCLKSEL_CCLK_DIV_1);

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
  TIM_ConfigMatch (pTimerRegs[timerNum], &TIM_MatchConfigStruct);

  /* Set to have highest priority = 0 */
  NVIC_SetPriority(TimerConfig[timerNum].TimerIrq, 0);

  // store the callback address for later use
  HwTimer[timerNum].timerCallback = timerCallback;

}



// start the timer and enable interrupts
void enableHwTimer (uint16_t timerNum)
{
  /* Enable interrupt for timer */
  NVIC_EnableIRQ (TimerConfig[timerNum].TimerIrq);

  /* Start timer*/
  TIM_Cmd (pTimerRegs[timerNum], ENABLE);
}

// stop the timer and disable interrupts
void disableHwTimer (uint16_t timerNum)
{
  /* Stop timer*/
  TIM_Cmd(pTimerRegs[timerNum], DISABLE);

  /* Clear timer pending interrupt */
  TIM_ClearIntPending(pTimerRegs[timerNum], TIM_MR0_INT);

  /* Disable interrupt for timer*/
  NVIC_DisableIRQ(TimerConfig[timerNum].TimerIrq);
}

uint8_t isHwTimerEnabled(uint16_t timerNum)
{
  return NVIC->ISER[((uint32_t)(TimerConfig[timerNum].TimerIrq) >> 5)] & (1 << ((uint32_t)(TimerConfig[timerNum].TimerIrq) & 0x1F));
}

// set and enable a MatchX value and interrupt. If set, the timer callback will be called
// when timer is enabled
void setHwTimerMatch (uint16_t timerNum, uint16_t matchReg, uint32_t interval)
{
  TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

  TIM_MatchConfigStruct.MatchChannel = matchReg;
  // Enable interrupt when value matches the value in TC register
  TIM_MatchConfigStruct.IntOnMatch   = 1;
  // Do not toggle pin if timer matches it
  TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
  // Set Match value 
  TIM_MatchConfigStruct.MatchValue   = interval;
  TIM_ConfigMatch (pTimerRegs[timerNum], &TIM_MatchConfigStruct);
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
  static uint8_t counter = 0;
  tTimer *pTimer;

  millis_ticks++;

  /* 100ms tick for SDCard ***********************************************/
  counter++;
  if (counter > 99)
  {
    MMC_disk_timerproc();
    counter = 0;
  }
  /***********************************************************************/

  
  // process the slow timer list
  pTimer = SlowTimerHead;
  while (pTimer)
  {
    if (pTimer->Running)
    {
      if (pTimer->Current > 0)
        pTimer->Current--;

      if (pTimer->Current == 0)
      {
        if (pTimer->AutoReload)
          pTimer->Current = pTimer->Reload;
        else 
          pTimer->Running = 0;
        
        pTimer->Expired = 1;
        if (pTimer->timerCallback)
          pTimer->timerCallback(pTimer);
      }
    }
    pTimer = pTimer->pNext;
  }
  /***********************************************************************/
}

long millis(void)
{
  return millis_ticks;
}

void delay_ms(int ms)
{
  int start = millis();

  while (millis() - start <= ms)
    ;
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


// Slow timers
bool AddSlowTimer (tTimer *pTimer)
{
  pTimer->pNext = NULL;
  if (SlowTimerHead == NULL)
  {
    SlowTimerHead = pTimer;
    SlowTimerTail = pTimer;
  }
  else
  {
    SlowTimerTail->pNext = pTimer;
    SlowTimerTail = pTimer;
  }

  return true;
}

void StartSlowTimer (tTimer *pTimer, uint32_t intervalMillis, tTimerCallback timerCallback)
{
  pTimer->Reload = intervalMillis;
  pTimer->Current = pTimer->Reload;
  pTimer->timerCallback = timerCallback;
  pTimer->Expired = 0;
  pTimer->Running = 1;
}

void StopSlowTimer (tTimer *pTimer)
{
  pTimer->Running = false;
}
// END

