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

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_pwm.h"

#include "timer.h"

static tTimer buzzerTimer;

//
// Internal functions
//

static void buzzer_pwm_set_frequency (uint16_t frequency)
{
  uint32_t match_value = (((float) 1/frequency)) * 1000000;

  PWM_TIMERCFG_Type PWMCfgDat;
  PWM_MATCHCFG_Type PWMMatchCfgDat;

  /* PWM block section -------------------------------------------- */
  /* Initialize PWM peripheral, timer mode
   * PWM prescale value = 1 (absolute value - tick value) */
  PWMCfgDat.PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
  PWMCfgDat.PrescaleValue = 1;
  PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat);

  /* Set match value for PWM match channel 0 = match_value, update immediately */
  PWM_MatchUpdate(LPC_PWM1, 0, match_value, PWM_MATCH_UPDATE_NOW);
  /* PWM Timer/Counter will be reset when channel 0 matching
   * no interrupt when match
   * no stop when match */
  PWMMatchCfgDat.IntOnMatch = DISABLE;
  PWMMatchCfgDat.MatchChannel = 0;
  PWMMatchCfgDat.ResetOnMatch = ENABLE;
  PWMMatchCfgDat.StopOnMatch = DISABLE;
  PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

  /* Configure PWM channel: --------------------------------------------- */
  /* - Single edge
   * - PWM Duty on each PWM channel determined by
   * the match on channel 0 to the match of that match channel.
   * Example: PWM Duty on PWM channel 1 determined by
   * the match on channel 0 to the match of match channel 1.
   */

  /* Configure PWM channel edge option
   * Note: PWM Channel 1 is in single mode as default state and
   * can not be changed to double edge mode */
  PWM_ChannelConfig(LPC_PWM1, 3, PWM_CHANNEL_SINGLE_EDGE);

  /* Set up match value */
  PWM_MatchUpdate(LPC_PWM1, 3, match_value/2, PWM_MATCH_UPDATE_NOW);
  /* Configure match option */
  PWMMatchCfgDat.IntOnMatch = DISABLE;
  PWMMatchCfgDat.MatchChannel = 3;
  PWMMatchCfgDat.ResetOnMatch = DISABLE;
  PWMMatchCfgDat.StopOnMatch = DISABLE;
  PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
  /* Enable PWM Channel Output */
  PWM_ChannelCmd(LPC_PWM1, 3, ENABLE);
}

static void buzzer_pwm_start (void)
{
  /* Reset and Start counter */
  PWM_ResetCounter(LPC_PWM1);
  PWM_CounterCmd(LPC_PWM1, ENABLE);

  /* Start PWM now */
  PWM_Cmd(LPC_PWM1, ENABLE);
}

static void buzzer_pwm_stop (void)
{
  /* Stop counter */
  PWM_CounterCmd(LPC_PWM1, DISABLE);

  /* Stop PWM now */
  PWM_Cmd(LPC_PWM1, DISABLE);
}

static void buzzerTimerCallback (tTimer *pTimer)
{
  (void)pTimer; // not used

  buzzer_pwm_stop();
}

//
// Public functions
//

void buzzer_play (uint16_t frequency, uint16_t duration)
{
  buzzer_pwm_set_frequency(frequency);
  buzzer_pwm_start();
  
  StartSlowTimer (&buzzerTimer, duration, buzzerTimerCallback); 
}

void buzzer_wait(void)
{
  while (buzzerTimer.Running)
      {};
  
}

void buzzer_init (void)
{
  PINSEL_CFG_Type PinCfg;

  /*
   * Initialize P2.2 PWM1[3] pin
   */
  PinCfg.Funcnum = PINSEL_FUNC_1;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
  PinCfg.Portnum = 2;
  PinCfg.Pinnum = 2;
  PINSEL_ConfigPin(&PinCfg);
  
  AddSlowTimer (&buzzerTimer);
}

