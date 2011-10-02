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
#include <string.h>         // memcpy
#include <math.h>

#include "r2c2.h"

#include "dda.h"
#include "dda_queue.h"
#include "debug.h"
#include "pinout.h"
#include "machine.h"
#include "config.h"

#ifndef ABS
#define ABS(v)          (((v) >= 0)?(v):(-(v)))
#endif

#ifndef ABSDELTA
#define ABSDELTA(a, b)  (((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

extern uint8_t led_on;

// inverse, used in distance calculation during DDA setup
double um_per_step_x;
double um_per_step_y;
double um_per_step_z;
double um_per_step_e;

void dda_init(void)
{
  um_per_step_x = ((double) (1000.0 / config.steps_per_mm_x));
  um_per_step_y = ((double) (1000.0 / config.steps_per_mm_y));
  um_per_step_z = ((double) (1000.0 / config.steps_per_mm_z));
  um_per_step_e = ((double) (1000.0 / config.steps_per_mm_e));

  // initialise with a safe slow feedrate
  // magic number: accel does not work with lower value
  startpoint.F = 240;
}

/*
        step timeout
*/

volatile uint16_t steptimeout = 0;

/*
        position tracking
*/

TARGET startpoint;
TARGET current_position;

//
volatile uint8_t step_requested;
uint8_t led_count [4];

/*
        utility functions
*/

double calc_distance( double dx, double dy )
{
  return (sqrt(dx*dx + dy*dy));
}

double calc_distance_3( double dx, double dy, double dz )
{
  return (sqrt(dx*dx + dy*dy + dz*dz));
}

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
__attribute__((const)) uint8_t   msbloc (uint32_t v) {
        uint8_t i;
        uint32_t c;
        for (i = 31, c = 0x80000000; i; i--) {
                if (v & c)
                        return i;
                c >>= 1;
        }
        return 0;
}

/*
        CREATE a dda given current_position and a target, save to passed location so we can write directly into the queue
*/

void dda_create(DDA *dda, TARGET *target) {
        double distance;
        uint64_t c_limit, c_limit_calc;

        // initialise DDA to a known state
        dda->allflags = 0;

        // we end at the passed target
        memcpy(&(dda->endpoint), target, sizeof(TARGET));

        dda->x_delta = ABS(target->X - startpoint.X);
        dda->y_delta = ABS(target->Y - startpoint.Y);
        dda->z_delta = ABS(target->Z - startpoint.Z);

        if (config.enable_extruder_1)
          dda->e_delta = ABS(target->E - startpoint.E);
        else
          dda->e_delta = 0;

        dda->x_direction = (target->X >= startpoint.X)?1:0;
        dda->y_direction = (target->Y >= startpoint.Y)?1:0;
        dda->z_direction = (target->Z >= startpoint.Z)?1:0;
        dda->e_direction = (target->E >= startpoint.E)?1:0;

        dda->total_steps = dda->x_delta;
        if (dda->y_delta > dda->total_steps)
                dda->total_steps = dda->y_delta;
        if (dda->z_delta > dda->total_steps)
                dda->total_steps = dda->z_delta;
        if (dda->e_delta > dda->total_steps)
                dda->total_steps = dda->e_delta;

        if (dda->total_steps == 0)
        //if ((dda->total_steps == 0) && (startpoint.F == target->F))
        {
          dda->nullmove = 1;
        }
        else {
                // get steppers ready to go
                steptimeout = 0;
                power_on();

                dda->x_counter = dda->y_counter = dda->z_counter = dda->e_counter = -(dda->total_steps >> 1);

                // since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
                if (dda->z_delta == 0)
                        distance = calc_distance(dda->x_delta * um_per_step_x, dda->y_delta * um_per_step_y);
                else if (dda->x_delta == 0 && dda->y_delta == 0)
                        distance = dda->z_delta * um_per_step_z;
                else
                        distance = calc_distance_3(dda->x_delta * um_per_step_x, dda->y_delta * um_per_step_y, dda->z_delta * um_per_step_z);

                if (distance < 2)
                        distance = dda->e_delta * um_per_step_e;

                /*
                 * distance --> distância (em micrometros - um) total
                 * dda->total_steps --> nr de steps (máximo de qualquer um dos eixos)
                 * (distance / dda->total_steps) --> distância de cada step
                 * move_duration --> nr the CPU ticks que demora cada step - um*10ns ticks
                 */
                double move_duration = (((double) distance/1000) / dda->total_steps) * F_CPU * 60;

                // similarly, find out how fast we can run our axes.
                // do this for each axis individually, as the combined speed of two or more axes can be higher than the capabilities of a single one.
                c_limit = 0;
                c_limit_calc = ((((((double) dda->x_delta*um_per_step_x)/1000) / dda->total_steps) * F_CPU * 60) / config.maximum_feedrate_x) * 256;
                if (c_limit_calc > c_limit)
                        c_limit = c_limit_calc;
                c_limit_calc = ((((((double) dda->y_delta*um_per_step_y)/1000) / dda->total_steps) * F_CPU * 60) / config.maximum_feedrate_y) * 256;
                if (c_limit_calc > c_limit)
                        c_limit = c_limit_calc;
                c_limit_calc = ((((((double) dda->z_delta*um_per_step_z)/1000) / dda->total_steps) * F_CPU * 60) / config.maximum_feedrate_z) * 256;
                if (c_limit_calc > c_limit)
                        c_limit = c_limit_calc;
                c_limit_calc = ((((((double) dda->e_delta*um_per_step_e)/1000) / dda->total_steps) * F_CPU * 60) / config.maximum_feedrate_e) * 256;
                if (c_limit_calc > c_limit)
                        c_limit = c_limit_calc;

                #ifdef ACCELERATION_REPRAP
                // c is initial step time in IOclk ticks
                dda->c = (move_duration / startpoint.F) * 256;
                if (dda->c < c_limit)
                        dda->c = c_limit;
                dda->end_c = (move_duration / target->F) * 256;
                if (dda->end_c < c_limit)
                        dda->end_c = c_limit;

                if (dda->c != dda->end_c) {
                        uint32_t stF = startpoint.F / 4;
                        uint32_t enF = target->F / 4;
                        // now some constant acceleration stuff, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
                        uint32_t ssq = (stF * stF);
                        uint32_t esq = (enF * enF);
                        int32_t dsq = (int32_t) (esq - ssq) / 4;

                        uint8_t msb_ssq = msbloc(ssq);
                        uint8_t msb_tot = msbloc(dda->total_steps);

                        // the raw equation WILL overflow at high step rates, but 64 bit math routines take waay too much space
                        // at 65536 mm/min (1092mm/s), ssq/esq overflows, and dsq is also close to overflowing if esq/ssq is small
                        // but if ssq-esq is small, ssq/dsq is only a few bits
                        // we'll have to do it a few different ways depending on the msb locations of each
                        if ((msb_tot + msb_ssq) <= 30) {
                                // we have room to do all the multiplies first
                                dda->n = ((int32_t) (dda->total_steps * ssq) / dsq) + 1;
                        }
                        else if (msb_tot >= msb_ssq) {
                                // total steps has more precision
                                dda->n = (((int32_t) dda->total_steps / dsq) * (int32_t) ssq) + 1;
                        }
                        else {
                                // otherwise
                                dda->n = (((int32_t) ssq / dsq) * (int32_t) dda->total_steps) + 1;
                        }

                        dda->accel = 1;
                }
                else
                        dda->accel = 0;

                #elif defined ACCELERATION_RAMPING
                        // add the last bit of dda->total_steps to always round up
                        dda->ramp_steps = dda->total_steps / 2 + (dda->total_steps & 1);
                        dda->step_no = 0;
                        // c is initial step time in IOclk ticks
                        dda->c = ACCELERATION_STEEPNESS * 256;
                        dda->c_min = (move_duration / target->F) * 256;
                        if (dda->c_min < c_limit)
                                dda->c_min = c_limit;
                        dda->n = 1;
                        dda->ramp_state = RAMP_UP;
                #else
                        dda->c = (move_duration / target->F) * 256;
                        if (dda->c < c_limit)
                                dda->c = c_limit;
                #endif
        }

        // next dda starts where we finish
        memcpy(&startpoint, target, sizeof(TARGET));
}

/*
        Start a prepared DDA
*/

void dda_start(DDA *dda) {
        // called from interrupt context: keep it simple!
        if (dda->nullmove) {
                // just change speed?
                current_position.F = dda->endpoint.F;
                // keep dda->live = 0
        }
        else {
                if (dda->waitfor_temp) {
                        //serial_writestr("Waiting for target temp\r\n");
                }
                else {
                        // ensure steppers are ready to go
                        steptimeout = 0;
                        power_on();

                        // set direction outputs
                        x_direction(dda->x_direction);
                        y_direction(dda->y_direction);
                        z_direction(dda->z_direction);
                        e_direction(dda->e_direction);
                }

                // ensure this dda starts
                dda->live = 1;

// debug: sersendf ("%lx\n", (uint32_t)(dda->c>>8));

                // set timeout for first step
                setHwTimerInterval (0, dda->c >> 8);
                enableHwTimer(0);
        }
}

/*
        STEP
*/

static inline void inc_led_count (uint8_t *pCount, uint8_t led_mask)
{
#ifdef STEP_LED_FLASH_VARIABLE
  (*pCount) ++;
  if (*pCount == 128)
  {
    led_on = led_on ^ led_mask;
    *pCount = 0;
  }
#endif
}

bool hit_home_stop_y (unsigned dir)
{
  if (config.home_direction_y < 0)
  {
    return y_min() && (dir == 0);
  }
  else
  {
    return y_min() && (dir != 0);
  }
}

void dda_step(DDA *dda) {
  // called from interrupt context! keep it as simple as possible
  uint8_t did_step = 0;

  step_requested = 0;
  
  if ( (current_position.X != dda->endpoint.X) && 
        (  (dda->endpoint.options.g28 == 0)
        || ! (x_min() && (dda->x_direction == 0) )
        )
     )
  {
    dda->x_counter -= dda->x_delta;
    if (dda->x_counter < 0)
    {
      x_unstep();
      step_requested |= 1;
      did_step = 1;
      if (dda->x_direction)
        current_position.X++;
      else
        current_position.X--;

      dda->x_counter += dda->total_steps;
      inc_led_count (&led_count[0], 1);
    }
  }

  if (dda->endpoint.options.g28 == 1)
  {
    if ((current_position.Y != dda->endpoint.Y) && (!hit_home_stop_y (dda->y_direction) ) )
    {
      dda->y_counter -= dda->y_delta;
      if (dda->y_counter < 0)
      {
        y_unstep();
        step_requested |= 2;
        did_step = 1;
        if (dda->y_direction)
          current_position.Y++;
        else
          current_position.Y--;

        dda->y_counter += dda->total_steps;
        inc_led_count (&led_count[1], 2);
      }
    }
  }
  else
  {
    if (current_position.Y != dda->endpoint.Y)
    {
      dda->y_counter -= dda->y_delta;
      if (dda->y_counter < 0)
      {
        y_unstep();
        step_requested |= 2;
        did_step = 1;
        if (dda->y_direction)
          current_position.Y++;
        else
          current_position.Y--;

        dda->y_counter += dda->total_steps;
        inc_led_count (&led_count[1], 2);
      }
    }
  }

  if (dda->endpoint.options.g28 == 1)
  {
    if ((current_position.Z != dda->endpoint.Z) && (!(z_min() && (dda->z_direction == 0))))
    {
      dda->z_counter -= dda->z_delta;
      if (dda->z_counter < 0)
      {
        z_unstep();
        step_requested |= 4;
        did_step = 1;
        if (dda->z_direction)
          current_position.Z++;
        else
          current_position.Z--;

        dda->z_counter += dda->total_steps;
        inc_led_count (&led_count[2], 4);
      }
    }
  }
  else
  {
    if (current_position.Z != dda->endpoint.Z)
    {
      dda->z_counter -= dda->z_delta;
      if (dda->z_counter < 0)
      {
        z_unstep();
        step_requested |= 4;
        did_step = 1;
        if (dda->z_direction)
          current_position.Z++;
        else
          current_position.Z--;

        dda->z_counter += dda->total_steps;
        inc_led_count (&led_count[2], 4);
      }
    }
  }

  if (current_position.E != dda->endpoint.E)
  {
    dda->e_counter -= dda->e_delta;
    if (dda->e_counter < 0)
    {
      e_unstep();
      step_requested |= 8;
      did_step = 1;
      if (dda->e_direction)
        current_position.E++;
      else
        current_position.E--;

      dda->e_counter += dda->total_steps;
      inc_led_count (&led_count[3], 8);
    }
  }

  #ifdef ACCELERATION_REPRAP
        // linear acceleration magic, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
        if (dda->accel) 
        {
          if ((dda->c > dda->end_c) && (dda->n > 0)) 
          {
            uint32_t new_c = dda->c - (dda->c * 2) / dda->n;
            if (new_c <= dda->c && new_c > dda->end_c) 
            {
              dda->c = new_c;
              dda->n += 4;
            }
            else
              dda->c = dda->end_c;
          }
          else if ((dda->c < dda->end_c) && (dda->n < 0)) 
          {
            uint32_t new_c = dda->c + ((dda->c * 2) / -dda->n);
            if (new_c >= dda->c && new_c < dda->end_c) 
            {
              dda->c = new_c;
              dda->n += 4;
            }
            else
              dda->c = dda->end_c;
          }
          else if (dda->c != dda->end_c) 
          {
            dda->c = dda->end_c;
          }
          // else we are already at target speed
          setHwTimerInterval (0, dda->c >> 8);
        }
  #endif

        #ifdef ACCELERATION_RAMPING
                // - algorithm courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
                // - for simplicity, taking even/uneven number of steps into account dropped
                // - number of steps moved is always accurate, speed might be one step off
                switch (dda->ramp_state) {
                        case RAMP_UP:
                        case RAMP_MAX:
                                if (dda->step_no >= dda->ramp_steps) {
                                        // RAMP_UP: time to decelerate before reaching maximum speed
                                        // RAMP_MAX: time to decelerate
                                        dda->ramp_state = RAMP_DOWN;
                                        dda->n = -((int32_t)2) - dda->n;
                                }
                                if (dda->ramp_state == RAMP_MAX)
                                        break;
                        case RAMP_DOWN:
                                dda->n += 4;
                                // be careful of signedness!
                                dda->c = (int32_t)dda->c - ((int32_t)(dda->c * 2) / dda->n);
                                if (dda->c <= dda->c_min) {
                                        // maximum speed reached
                                        dda->c = dda->c_min;
                                        dda->ramp_state = RAMP_MAX;
                                        dda->ramp_steps = dda->total_steps - dda->step_no;
                                }
                                setHwTimerInterval (0, dda->c >> 8);
                                break;
                }
                dda->step_no++;
        #endif

        if (did_step) {
                // we stepped, reset timeout
                steptimeout = 0;

        // if we could do anything at all, we're still running
        // otherwise, must have finished
        }
        else {
                dda->live = 0;
                // linear acceleration code doesn't alter F during a move, so we must update it here
                // in theory, we *could* update F every step, but that would require a divide in interrupt context which should be avoided if at all possible
                current_position.F = dda->endpoint.F;
        }

  // Match 1 and Match 2 interrupts will create step pulse
}
