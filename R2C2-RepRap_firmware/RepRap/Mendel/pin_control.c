/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
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

#include "pin_control.h"
#include "ios.h"
#include "config.h"

void axis_enable    (unsigned axis)
{
  write_pin (config.axis [axis].pin_enable, ENABLE); 
}

void axis_disable   (unsigned axis)
{
  write_pin (config.axis [axis].pin_enable, DISABLE); 
}

void axis_step      (unsigned axis)
{
  write_pin (config.axis [axis].pin_step, ACTIVE); 
}

void axis_unstep    (unsigned axis)
{
  write_pin (config.axis [axis].pin_step, INACTIVE); 
}

void axis_set_direction (unsigned axis, unsigned dir)
{
  write_pin (config.axis [axis].pin_dir, dir);  // may be inverted by pin definition
}

bool axis_min (unsigned axis)
{
  // NB The pin definition will handle active low/high
  if (read_pin (config.axis [axis].pin_min_limit))
    return true;
  else
    return false;  
}

// enable all axes
void enable_all_axes (void)
{
  int axis;

  for (axis = 0; axis < config.num_axes; axis++)
  {
    if (config.axis [axis].is_configured)
      axis_enable (axis);
  }
}

// disable all axes
void disable_all_axes (void)
{
  int axis;

  for (axis = 0; axis < config.num_axes; axis++)
  {
    if (config.axis [axis].is_configured)
      axis_disable (axis);
  }
}



void atx_power_on()
{
  //TODO
}

void atx_power_off()
{
  //TODO
}


