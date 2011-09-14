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

struct configuration
{
  uint16_t steps_per_mm_x;
  uint16_t steps_per_mm_y;
  uint16_t steps_per_mm_z;
  uint16_t steps_per_mm_e;

  uint32_t maximum_feedrate_x;
  uint32_t maximum_feedrate_y;
  uint32_t maximum_feedrate_z;
  uint32_t maximum_feedrate_e;

  uint32_t search_feedrate_x;
  uint32_t search_feedrate_y;
  uint32_t search_feedrate_z;
  uint32_t search_feedrate_e;
  
  // rate when homing (fast)
  uint32_t homing_feedrate_x;
  uint32_t homing_feedrate_y;
  uint32_t homing_feedrate_z;
  
  // direction to move when homing (depends on endstop locations)
  int16_t home_direction_x;
  int16_t home_direction_y;
  int16_t home_direction_z;
  
  // position at home
  int16_t home_pos_x;
  int16_t home_pos_y;
  int16_t home_pos_z;

  // printable volume size
  // TODO: Need to define origin?
  int16_t printing_vol_x;
  int16_t printing_vol_y;
  int16_t printing_vol_z;
  
  // dump pos
  int16_t dump_pos_x;
  int16_t dump_pos_y;
  
  // rest pos
  int16_t rest_pos_x;
  int16_t rest_pos_y;

};

extern struct configuration config;

void read_config (void);

#endif /* CONFIG_H */
