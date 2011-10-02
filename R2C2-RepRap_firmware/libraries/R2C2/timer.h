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

#ifndef	_TIMER_H
#define	_TIMER_H

#include <stdbool.h>

// time-related constants
//#define	US	(F_CPU / 1000000)
#define	MS	(F_CPU / 1000)

// #define	DEFAULT_TICK	(100 US)
#define	WAITING_DELAY  (10 * MS)

#define NUM_HARDWARE_TIMERS 4

// Slow timers
typedef struct tTimer tTimer; // incomplete type

typedef void (*tTimerCallback)(tTimer *);

struct tTimer
{
  tTimer            *pNext;
  tTimerCallback    timerCallback;
  uint32_t          Current;
  uint32_t          Reload;
  volatile uint8_t  Running:1;
  volatile uint8_t  Expired:1;
  volatile uint8_t  AutoReload:1;
};

// Hardware timers
typedef struct tHwTimer tHwTimer;

// timer callback gets pointer to tHwTimer struct and copy of timer IR register
typedef void (*tHwTimerCallback)(tHwTimer *, uint32_t);

struct tHwTimer
{
  tHwTimerCallback timerCallback;
} ;

void setupHwTimer (uint16_t timerNum, tHwTimerCallback timerCallback);
void setHwTimerInterval (uint16_t timerNum, uint32_t ticks);
void enableHwTimer (uint16_t timerNum);
void disableHwTimer (uint16_t timerNum);
uint8_t isHwTimerEnabled(uint16_t timerNum);
void setHwTimerMatch (uint16_t timerNum, uint16_t matchReg, uint32_t interval);

void SysTickTimer_Init(void);
void delay(int delay);
void delay_ms(int delay);
void delayMicrosecondsInterruptible(int us);
#define	delay_us(d) delayMicrosecondsInterruptible(d)
long millis(void);

// Slow timer (i.e. +/-1ms resolution)
bool AddSlowTimer (tTimer *pTimer);
void StartSlowTimer (tTimer *pTimer, uint32_t intervalMillis, tTimerCallback timerCallback);
void StopSlowTimer (tTimer *pTimer);
#define IsSlowTimerExpired (pTimer)  ((pTimer)->Expired)

#endif	/* _TIMER_H */

