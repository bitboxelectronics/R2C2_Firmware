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

#include <stdlib.h>

#include	"dda_queue.h"
#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"

extern void startBlink();
extern void stopBlink();

/* movebuffer works as a ring buffer */
uint8_t mb_head = 0;
uint8_t mb_tail = 0;
DDA movebuffer[MOVEBUFFER_SIZE];

// return 255 if full, else 0
uint8_t queue_full() 
{
  uint8_t a;
  a = (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) == 0) ? 255:0;
  return a;
}

// return 255 if empty, else 0
uint8_t queue_empty() 
{
  return ((mb_tail == mb_head) && (movebuffer[mb_tail].live == 0)) ? 255:0;
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
void queue_step()
{
  // do our next step
  if (movebuffer[mb_tail].live)
  {
    if (movebuffer[mb_tail].waitfor_temp)
    {
      temp_tick();
      if (temp_achieved(EXTRUDER_0))
      {
        movebuffer[mb_tail].live = movebuffer[mb_tail].waitfor_temp = 0;
        serial_writestr("Temp achieved\r\n");
      }
    }
    else
    {
      // NOTE: dda_step makes this interrupt interruptible after steps have been sent but before new speed is calculated.
      dda_step(&(movebuffer[mb_tail]));
    }
  }

  // fall directly into dda_start instead of waiting for another step
  // the dda dies not directly after its last step, but when the timer fires and there's no steps to do
  if (movebuffer[mb_tail].live == 0)
    next_move();
}

void enqueue(TARGET *t)
{
  // don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
  while (queue_full())
    delay(WAITING_DELAY);

  uint8_t h = mb_head + 1;
  h &= (MOVEBUFFER_SIZE - 1);

  if (t != NULL)
  {
    dda_create(&movebuffer[h], t);
  }
  else
  {
    // it's a wait for temp
    movebuffer[h].waitfor_temp = 1;
    movebuffer[h].nullmove = 0;
  }

  mb_head = h;

  // fire up in case we're not running yet
  if (isHwTimerEnabled(0) == 0)
    next_move();
}

void next_move()
{
  if (queue_empty() == 0)
  {
    // next item
    uint8_t t = mb_tail + 1;
    t &= (MOVEBUFFER_SIZE - 1);
    
    DDA *current = &movebuffer[t];
    
    mb_tail = t;
    
    if (current->waitfor_temp)
    {
      serial_writestr ("Waiting for target temp\n");
      current->live = 1;
      setHwTimerInterval (0, HEATER_WAIT_TIMEOUT);
      enableHwTimer(0);
    }
    else
    {
      startBlink();
      dda_start(current);
    }
  }
  else
  {
    disableHwTimer(0);
    stopBlink();
  }
}

void queue_flush()
{
  disableHwTimer(0);
  stopBlink();

  // flush queue
  mb_tail = mb_head;
  movebuffer[mb_head].live = 0;
}
