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

#ifndef DDA_H
#define DDA_H

#include        <stdint.h>
//#include        "config.h"
#include        "machine.h"

#ifndef ABS
#define ABS(v)          (((v) >= 0)?(v):(-(v)))
#endif

/*
        enums
*/
// wether we accelerate, run at full speed, break down, etc.
typedef enum {
        RAMP_UP,
        RAMP_MAX,
        RAMP_DOWN
} ramp_state_t;

/*
        types
*/

// target is simply a point in space/time
typedef struct {
        int64_t                                         X;
        int64_t                                         Y;
        int64_t                                         Z;
        int64_t                                         E;
        uint64_t                                        F;
        struct
        {
          uint8_t g28;
        } options;
} TARGET;

// this is a digital differential analyser data struct
typedef struct {
        // this is where we should finish
        TARGET                                          endpoint;

        union {
                struct {
                        // status fields
                        uint8_t                                         nullmove                        :1;
                        uint8_t                                         live                                    :1;
                        #ifdef ACCELERATION_REPRAP
                        uint8_t                                         accel                                   :1;
                        #endif

                        // wait for temperature to stabilise flag
                        uint8_t                                         waitfor_temp    :1;

                        // directions
                        uint8_t                                         x_direction             :1;
                        uint8_t                                         y_direction             :1;
                        uint8_t                                         z_direction             :1;
                        uint8_t                                         e_direction             :1;
                };
                uint8_t                                                 allflags;       // used for clearing all flags
        };

        // distances
        uint64_t                                        x_delta;
        uint64_t                                        y_delta;
        uint64_t                                        z_delta;
        uint64_t                                        e_delta;

        // bresenham counters
        int64_t                                         x_counter;
        int64_t                                         y_counter;
        int64_t                                         z_counter;
        int64_t                                         e_counter;

        // total number of steps: set to max(x_delta, y_delta, z_delta, e_delta)
        uint64_t                                        total_steps;

        // linear acceleration variables: c and end_c are 24.8 fixed point timer values, n is the tracking variable
        uint64_t                                        c;
        #ifdef ACCELERATION_REPRAP
        uint64_t                                        end_c;
        int64_t                                         n;
        #endif
        #ifdef ACCELERATION_RAMPING
        // start of down-ramp, intitalized with total_steps / 2
        uint64_t                                        ramp_steps;
        // counts actual steps done
        uint64_t                                        step_no;
        // 24.8 fixed point timer value, maximum speed
        uint64_t                                        c_min;
        // tracking variable
        int64_t                                         n;
        ramp_state_t                    ramp_state;
        #endif
} DDA;

/*
        variables
*/

// steptimeout is set to zero when we step, and increases over time so we can turn the motors off when they've been idle for a while
extern volatile uint16_t steptimeout;

// startpoint holds the endpoint of the most recently created DDA, so we know where the next one created starts
// could also be called last_endpoint
extern TARGET startpoint;

// current_position holds the machine's current position. this is only updated when we step, or when G92 (set home) is received.
extern TARGET current_position;

/*
        methods
*/

//double approx_distance( double dx, double dy );
//double approx_distance_3( double dx, double dy, double dz );

double calc_distance( double dx, double dy );

// const because return value is always the same given the same v
__attribute__((const)) uint8_t   msbloc (uint32_t v);

/* Initialize DDA variables */
void dda_init(void);

// create a DDA
void dda_create(DDA *dda, TARGET *target);

// start a created DDA (called from timer interrupt or when there is a GCode like G1)
void dda_start(DDA *dda);

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda);

#endif  /* DDA_H */
