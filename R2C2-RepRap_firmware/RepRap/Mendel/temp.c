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


 // 274
 // 10k
 /* {ADC value Extruder0, ADC value HeatedBed0, temperature} */
/* uint16_t temptable[NUMTEMPS][3] = {
 { 1156,   44, 300 },
 { 1276,   50, 290 },
 { 1406,   58, 280 },
 { 1547,   67, 270 },
 { 1698,   78, 260 },
 { 1859,   91, 250 },
 { 2029,   107, 240 },
 { 2205,  127, 230 },
 { 2385,  151, 220 },
 { 2567,  180, 210 },
 { 2746,  216, 200 },
 { 2921,  261, 190 },
 { 3088,  317, 180 },
 { 3244,  387, 170 },
 { 3386,  473, 160 },
 { 3514,  582, 150 },
 { 3627,  716, 140 },
 { 3723,  880, 130 },
 { 3805, 1080, 120 },
 { 3872, 1318, 110 },
 { 3927, 1594, 100 },
 { 3971, 1903,  90 },
 { 4005, 2234,  80 },
 { 4031, 2572,  70 },
 { 4050, 2897,  60 },
 { 4064, 3191,  50 },
 { 4075, 3442,  40 },
 { 4082, 3642,  30 },
 { 4087, 3794,  20 },
 { 4090, 3902,  10 },
 { 4093, 3977,   0 }
}; */

uint16_t temptable[NUMTEMPS][3] = {
    {       1156    ,       44      ,       300     }       ,
    {       1214    ,       47      ,       295     }       ,
    {       1276    ,       50      ,       290     }       ,
    {       1340    ,       54      ,       285     }       ,
    {       1406    ,       58      ,       280     }       ,
    {       1475    ,       62      ,       275     }       ,
    {       1547    ,       67      ,       270     }       ,
    {       1621    ,       72      ,       265     }       ,
    {       1698    ,       78      ,       260     }       ,
    {       1777    ,       84      ,       255     }       ,
    {       1859    ,       91      ,       250     }       ,
    {       1943    ,       99      ,       245     }       ,
    {       2029    ,       107     ,       240     }       ,
    {       2116    ,       117     ,       235     }       ,
    {       2204    ,       127     ,       230     }       ,
    {       2294    ,       138     ,       225     }       ,
    {       2385    ,       151     ,       220     }       ,
    {       2475    ,       165     ,       215     }       ,
    {       2566    ,       180     ,       210     }       ,
    {       2656    ,       197     ,       205     }       ,
    {       2746    ,       216     ,       200     }       ,
    {       2834    ,       238     ,       195     }       ,
    {       2920    ,       261     ,       190     }       ,
    {       3005    ,       288     ,       185     }       ,
    {       3087    ,       317     ,       180     }       ,
    {       3167    ,       350     ,       175     }       ,
    {       3243    ,       387     ,       170     }       ,
    {       3316    ,       428     ,       165     }       ,
    {       3385    ,       473     ,       160     }       ,
    {       3451    ,       525     ,       155     }       ,
    {       3513    ,       581     ,       150     }       ,
    {       3571    ,       645     ,       145     }       ,
    {       3626    ,       715     ,       140     }       ,
    {       3676    ,       794     ,       135     }       ,
    {       3722    ,       880     ,       130     }       ,
    {       3765    ,       975     ,       125     }       ,
    {       3804    ,       1080    ,       120     }       ,
    {       3839    ,       1194    ,       115     }       ,
    {       3871    ,       1318    ,       110     }       ,
    {       3900    ,       1451    ,       105     }       ,
    {       3926    ,       1593    ,       100     }       ,
    {       3949    ,       1744    ,       95      }       ,
    {       3970    ,       1902    ,       90      }       ,
    {       3988    ,       2066    ,       85      }       ,
    {       4004    ,       2233    ,       80      }       ,
    {       4017    ,       2403    ,       75      }       ,
    {       4030    ,       2571    ,       70      }       ,
    {       4040    ,       2737    ,       65      }       ,
    {       4049    ,       2896    ,       60      }       ,
    {       4057    ,       3048    ,       55      }       ,
    {       4063    ,       3190    ,       50      }       ,
    {       4069    ,       3322    ,       45      }       ,
    {       4074    ,       3441    ,       40      }       ,
    {       4078    ,       3547    ,       35      }       ,
    {       4081    ,       3641    ,       30      }       ,
    {       4084    ,       3723    ,       25      }       ,
    {       4086    ,       3793    ,       20      }       ,
    {       4088    ,       3852    ,       15      }       ,
    {       4089    ,       3902    ,       10      }       ,
    {       4091    ,       3942    ,       5       }       ,
    {       4092    ,       3976    ,       0       }
};


static uint16_t current_temp [NUMBER_OF_SENSORS] = {0};
static uint16_t target_temp  [NUMBER_OF_SENSORS] = {0};
static uint32_t adc_filtered [NUMBER_OF_SENSORS] = {4096, 4096}; // variable must have the higher value of ADC for filter start at the lowest temperature

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
//?    power_on();E: 243 -- B: 0
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

  static uint8_t counter = 0;
  if (counter == 100)
  {
    counter = 0;
    sersendf("E: %u -- ", current_temp[EXTRUDER_0]);
    sersendf("B: %u\r\n", current_temp[HEATED_BED_0]);
  }
  counter++;

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
