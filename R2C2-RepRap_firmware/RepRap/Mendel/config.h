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

#ifndef CONFIG_H
#define CONFIG_H

#include "stdint.h"
#include "stdbool.h"

/*
  // pin config
  input
  output
  pullup/down/etc
  special function: pwm etc
*/

typedef struct
{
  bool    configured;
  char    letter_code;      // X,Y,Z,A,B,C
  
  double  steps_per_mm;
  int32_t maximum_feedrate;
  double  acceleration;
  
  // step, dir, enable, reset
  // polarity: active low/high
  // pulse len low,high
  uint8_t pin_step;
  uint8_t pin_dir;
  uint8_t pin_enable;
  uint8_t pin_reset;
  
  int32_t step_invert;
  int32_t dir_invert;

  int32_t search_feedrate;
  int32_t homing_feedrate;
  int32_t home_pos;
  int32_t home_direction;
  
  int32_t printing_vol;
  
} tAxisSettings;

// axis configs: 
// X Y Z E
// X Y Z A
// X Y A B
#define NUM_AXES 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

#define MM_REPRAP_MENDEL  0
#define MM_RAPMAN         1

struct configuration
{
  int32_t  machine_model;
  
  int32_t num_axes;
  
  tAxisSettings axis[NUM_AXES];

  double steps_per_mm_y;
  double steps_per_mm_z;
  double steps_per_mm_e;

  int32_t maximum_feedrate_y;
  int32_t maximum_feedrate_z;
  int32_t maximum_feedrate_e;

  double  acceleration;
  double  junction_deviation;

  int32_t invert_dir_y;
  int32_t invert_dir_z;
  
  int32_t search_feedrate_y;
  int32_t search_feedrate_z;
  int32_t search_feedrate_e;
  
  // rate when homing (fast)
  int32_t homing_feedrate_y;
  int32_t homing_feedrate_z;
  
  // direction to move when homing (depends on endstop locations)
  int32_t home_direction_y;
  int32_t home_direction_z;
  
  // position at home
  int32_t home_pos_y;
  int32_t home_pos_z;

  
  // printable volume size
  // TODO: Need to define origin?
  int32_t printing_vol_y;
  int32_t printing_vol_z;
  
  // The following are specific to printers

  // dump pos
  int32_t have_dump_pos;
  int32_t dump_pos_x;
  int32_t dump_pos_y;
  
  // rest pos
  int32_t have_rest_pos;
  int32_t rest_pos_x;
  int32_t rest_pos_y;

  // wipe pos
  int32_t have_wipe_pos;
  int32_t wipe_entry_pos_x;
  int32_t wipe_entry_pos_y;
  int32_t wipe_exit_pos_x;
  int32_t wipe_exit_pos_y;
  
  //
  int32_t steps_per_revolution_e;
  
  // options
  int32_t wait_on_temp;
  int32_t enable_extruder_1;
};

extern struct configuration config;

void read_config (void);
void print_config (void);

#endif /* CONFIG_H */
