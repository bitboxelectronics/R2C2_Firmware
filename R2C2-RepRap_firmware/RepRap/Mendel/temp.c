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

#include "temp.h"
#include "machine.h"
#include "pinout.h"
#include "sersendf.h"
#include "stepper.h"

/* {ADC value Extruder0, ADC value HeatedBed0, temperature} */
uint16_t temptable[NUMTEMPS][3] = {
  {860, 60, 300},
  {1849, 95, 248},
  {2208, 119, 226},
  {2711, 215, 198},
  {2960, 293, 183},
  {3332, 447, 163},
  {3568, 641, 145},
  {3711, 865, 131},
  {3870, 1408, 105},
  {3960, 1906, 86},
  {4032, 2732, 64},
  {4062, 3352, 42},
  {4070, 3755, 22},
  {4080, 4085, 0}
};

static uint16_t current_temp [NUMBER_OF_SENSORS] = {0};
static uint16_t target_temp  [NUMBER_OF_SENSORS] = {0};
static uint32_t adc_filtered [NUMBER_OF_SENSORS] = {4095, 4095}; // variable must have the higher value of ADC for filter start at the lowest temperature

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

static uint16_t read_temp(uint8_t sensor_number);

void temp_set(uint16_t t, uint8_t sensor_number)
{
  if (t)
  {
    steptimeout = 0;
//?    power_on();
  }

  target_temp[sensor_number] = t;
}

uint16_t temp_get(uint8_t sensor_number)
{
  return current_temp[sensor_number];
}

uint16_t temp_get_target(uint8_t sensor_number)
{
  return target_temp[sensor_number];
}

uint8_t	temp_achieved(uint8_t sensor_number)
{
  if (current_temp[sensor_number] >= (target_temp[sensor_number] - 2))
    return 255;

  return 0;
}

uint8_t temps_achieved (void)
{
  if ((current_temp[EXTRUDER_0] >= (target_temp[EXTRUDER_0] - 2)) && (current_temp[HEATED_BED_0] >= (target_temp[HEATED_BED_0] - 2)))
    return 255;

  return 0;
}

void temp_print()
{
  sersendf("ok T:%u.0 B:%u.0\r\n", current_temp[EXTRUDER_0], current_temp[HEATED_BED_0]); /* for RepRap software */
}

void temp_tick(void)
{
  /* Read and average temperatures */
  current_temp[EXTRUDER_0] = read_temp(EXTRUDER_0);
  current_temp[HEATED_BED_0] = read_temp(HEATED_BED_0);

  /* Manage heater using simple ON/OFF logic, no PID */
  if (current_temp[EXTRUDER_0] < target_temp[EXTRUDER_0])
  {
    extruder_heater_on();
  }
  else
  {
    extruder_heater_off();
  }

  /* Manage heater using simple ON/OFF logic, no PID */
  if (current_temp[HEATED_BED_0] < target_temp[HEATED_BED_0])
  {
    heated_bed_on();
  }
  else
  {
    heated_bed_off();
  }
}

/* Read and average the ADC input signal */
static uint16_t read_temp(uint8_t sensor_number)
{
  int32_t raw = 0;
  int16_t celsius = 0;
  uint8_t i;

  if (sensor_number == EXTRUDER_0)
  {
    raw = analog_read(EXTRUDER_0_SENSOR_ADC_CHANNEL);
  }
  else if (sensor_number == HEATED_BED_0)
  {
    raw = analog_read(HEATED_BED_0_SENSOR_ADC_CHANNEL);
  }
  
  // filter the ADC values with simple IIR
  adc_filtered[sensor_number] = ((adc_filtered[sensor_number] * 15) + raw) / 16;
  
  raw = adc_filtered[sensor_number];

  /* Go and use the temperature table to math the temperature value... */
  if (raw < temptable[0][sensor_number]) /* Limit the smaller value... */
  {
    celsius = temptable[0][2];
  }
  else if (raw >= temptable[NUMTEMPS-1][sensor_number]) /* Limit the higher value... */
  {
    celsius = temptable[NUMTEMPS-1][2];
  }
  else
  {
    for (i=1; i<NUMTEMPS; i++)
    {
      if (raw < temptable[i][sensor_number])
      {
        celsius = temptable[i-1][2] +
            (raw - temptable[i-1][sensor_number]) *
            (temptable[i][2] - temptable[i-1][2]) /
            (temptable[i][sensor_number] - temptable[i-1][sensor_number]);

        break;
      }
    }
  }

  return celsius;
}

bool temp_set_table_entry (uint8_t sensor_number, uint16_t temp, uint16_t adc_val)
{
  if (sensor_number < NUMBER_OF_SENSORS)
  {
    for (int entry=0; entry < NUMTEMPS; entry++)
    {
      if (temptable[entry][2] == temp)
      {
        temptable[entry][sensor_number] = adc_val;
        return true;
      }
    }
    return false;
  }
  else
    return false;
}

uint16_t temp_get_table_entry (uint8_t sensor_number, uint16_t temp)
{
  uint16_t result = 0xffff;
  
  if (sensor_number < NUMBER_OF_SENSORS)
  {
    for (int entry=0; entry < NUMTEMPS; entry++)
    {
      if (temptable[entry][2] == temp)
      {
        result = temptable[entry][sensor_number];
        break;
      }
    }
  }
  return result;
}
