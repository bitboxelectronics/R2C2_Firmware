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

#include "adc.h"
#include "app_config.h"
#include "temp.h"
//#include "machine.h"
#include "pin_control.h"
#include "debug.h"
#include "stepper.h"    // TODO: steptimeout


/* Table for NTC EPCOS B57560G104F and R1 = 330R for Extruder0
 * Table for NTC EPCOS B57560G104F and R1 = 12K for HeatedBed0 */
 // 274
 // 10k
uint16_t temptable[NUMTEMPS][3] = {
  {1009,   36, 300}, /* {ADC value Extruder0, ADC value HeatedBed0, temperature} */
  {1119,   42, 290},
  {1240,   48, 280},
  {1372,   56, 270},
  {1517,   65, 260},
  {1673,   76, 250},
  {1839,   90, 240},
  {2015,  106, 230},
  {2198,  126, 220},
  {2385,  151, 210},
  {2573,  182, 200},
  {2759,  220, 190},
  {2940,  268, 180},
  {3112,  328, 170},
  {3270,  402, 160},
  {3415,  496, 150},
  {3544,  614, 140},
  {3655,  761, 130},
  {3750,  941, 120},
  {3830, 1161, 110},
  {3894, 1420, 100},
  {3946, 1719,  90},
  {3986, 2048,  80},
  {4017, 2394,  70},
  {4041, 2737,  60},
  {4058, 3056,  50},
  {4070, 3335,  40},
  {4079, 3563,  30},
  {4085, 3738,  20},
  {4089, 3866,  10},
  {4092, 3954,   0}
};

static uint16_t current_temp [NUMBER_OF_SENSORS] = {0};
static uint16_t target_temp  [NUMBER_OF_SENSORS] = {0};
static uint32_t adc_filtered [NUMBER_OF_SENSORS] = {0};

static uint16_t ticks;

/* Define a value for sequencial number of reads of ADC, to average the readed
 * value and try filter high frequency noise.
 */
#define ADC_READ_TIMES 4

#define NUM_TICKS 20

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

static uint16_t read_temp(uint8_t sensor_number);

#if 0
static uint16_t temp_read(uint8_t sensor_number)
{
  return current_temp[sensor_number];
}
#endif


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

void temp_tick(void)
{

  /* Read and average temperatures */
  current_temp[EXTRUDER_0] = read_temp(EXTRUDER_0);
  current_temp[HEATED_BED_0] = read_temp(HEATED_BED_0);

  ticks ++;
  if (ticks == NUM_TICKS)
  {
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
    
    ticks = 0;
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
    raw = analog_read(config.extruder_ctc[0].sensor_adc_channel);
  }
  else if (sensor_number == HEATED_BED_0)
  {
    raw = analog_read(config.heated_bed_ctc.sensor_adc_channel);
  }
  
  // filter the ADC values with simple IIR
  adc_filtered[sensor_number] = ((adc_filtered[sensor_number] * 7) + raw) / 8;
  
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
