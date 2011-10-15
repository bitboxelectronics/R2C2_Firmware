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

#ifndef	_TEMP_H
#define	_TEMP_H

#define NUMBER_OF_SENSORS               2
#define EXTRUDER_0                      0
#define HEATED_BED_0                    1

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"

// set target temperature
void temp_set(uint16_t t, uint8_t sensor_number);

// return last read temperature
uint16_t temp_get(uint8_t sensor_number);

// return target temperature
uint16_t temp_get_target(uint8_t sensor_number);

// true if last read temp is close to target temp, false otherwise
uint8_t temp_achieved(uint8_t sensor_number);

// send current temperature to host
void temp_print(void);

// periodically read temperature and update heater with PID
void temp_tick(void);

#define NUMTEMPS 31
extern uint16_t temptable[NUMTEMPS][3];

bool      temp_set_table_entry (uint8_t sensor_number, uint16_t temp, uint16_t adc_val);
uint16_t  temp_get_table_entry (uint8_t sensor_number, uint16_t temp);

#endif	/* _TIMER_H */
