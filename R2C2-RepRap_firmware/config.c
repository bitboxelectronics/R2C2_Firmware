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

#include <stdbool.h>
#include "stdlib.h"
#include "string.h"

#include "config.h"
#include "spi.h"
#include "ff.h"
#include "debug.h"
#include "gcode_parse.h"
#include "dda.h"
#include "uart.h"

/* values reflecting the gearing of your machine
 * all numbers are integers, so no decimals, please :-)
 */
struct configuration config;

typedef struct {
  char      *name;
  int32_t   *pValue;
  int32_t   default_value;
} tConfigItem;

/* calculate the default values appropriate for your machine */
tConfigItem config_lookup [] = 
{
  { "steps_per_mm_x", &config.steps_per_mm_x, 80},
  { "steps_per_mm_y", &config.steps_per_mm_y, 80},
  { "steps_per_mm_z", &config.steps_per_mm_z, 6400},
  { "steps_per_mm_e", &config.steps_per_mm_e, 36},    /* Wades extruder, NEMA 17 geared extruder (1/39 * 6.5mm) */

  /* used for G0 rapid moves and as a cap for all other feedrates */
  { "maximum_feedrate_x", &config.maximum_feedrate_x, 3000}, /* 50mm / second */
  { "maximum_feedrate_y", &config.maximum_feedrate_y, 3000},
  { "maximum_feedrate_z", &config.maximum_feedrate_z, 60},   /* 1mm / second */
  { "maximum_feedrate_e", &config.maximum_feedrate_e, 3000}, /* 50mm / second */

  /* used when searching endstops and similar */
  { "search_feedrate_x", &config.search_feedrate_x, 120},
  { "search_feedrate_y", &config.search_feedrate_y, 120},
  { "search_feedrate_z", &config.search_feedrate_z, 60},
  { "search_feedrate_e", &config.search_feedrate_e, 1600},
  
  { "homing_feedrate_x", &config.homing_feedrate_x, 3000},
  { "homing_feedrate_y", &config.homing_feedrate_y, 3000},
  { "homing_feedrate_z", &config.homing_feedrate_z, 60},
  
  // home pos is left front
  { "home_direction_x", &config.home_direction_x, -1}, 
  { "home_direction_y", &config.home_direction_y, -1},
  { "home_direction_z", &config.home_direction_z, -1},
  
  { "home_pos_x", &config.home_pos_x, 0},
  { "home_pos_y", &config.home_pos_y, 0},
  { "home_pos_z", &config.home_pos_z, 0},

  { "printing_vol_x", &config.printing_vol_x , 0},
  { "printing_vol_y", &config.printing_vol_y , 0},
  { "printing_vol_z", &config.printing_vol_z , 0},
  
  // dump pos
  { "have_dump_pos", &config.have_dump_pos , 0},
  { "dump_pos_x", &config.dump_pos_x , 0},
  { "dump_pos_y", &config.dump_pos_x , 0},
  
  // rest pos
  { "have_rest_pos", &config.have_rest_pos , 0},
  { "rest_pos_x", &config.rest_pos_x , 0},
  { "rest_pos_y", &config.rest_pos_y , 0},

  // wipe pos
  { "have_wipe_pos", &config.have_wipe_pos , 0},
  { "wipe_pos_x", &config.wipe_pos_x , 0},
  { "wipe_pos_y", &config.wipe_pos_y , 0},

  { "steps_per_revolution_e", &config.steps_per_revolution_e, 3200},  // 200 * 16
  
  { "wait_on_temp", &config.wipe_pos_y , 0},
    
};

#define NUM_TOKENS (sizeof(config_lookup)/sizeof(tConfigItem))

uint16_t read_u16 (FIL *file, char *line)
{
  f_gets(line, 80, file); /* read one line */
  char *p_pos = strchr(line, '='); /* find the '=' position */
  
  if (p_pos != NULL)
    return (atoi(p_pos+1));
  else
    return 0;
}

// return true if c matches any character in s
bool char_match (char c, char *s)
{
  while (*s)
  {
    if (*s == c)
      return true;
    s++;
  }
  return false;
}

// a version of strtok()
// NB: very unsafe; not re-entrant. Use with caution.
char *get_token (char *pLine, char *separators)
{
  static char *pNext;
  static char saved;
  char *pToken = NULL;
  
  if (pLine)
  {
    pNext = pLine;
  }
  else if (pNext)
  {
    *pNext = saved;
  }

  if (!pNext || !separators)
    return NULL;

  // skip separators    
  while (*pNext && char_match (*pNext, separators) )
  {
    pNext ++;
  }
      
  if (*pNext == 0)
    // reached end of string
    return NULL;
  else
  {  
    // find next separator
    pToken = pNext;
    while (*pNext && ! char_match (*pNext, separators) )
    {
      pNext ++;
    }
    
    saved = *pNext;
    *pNext = 0;
    return pToken;
  }
}

void read_config (void)
{
  char line[80];
  char *pToken;
  char *pLine;
  unsigned j;

  // first set defaults
  for (j=0; j < NUM_TOKENS; j++)
  {
    *config_lookup[j].pValue = config_lookup[j].default_value;
  }
    
  /* initialize SPI for SDCard */
  spi_init();

  /* access to "config.txt" file on SDCard */

  FATFS fs;       /* Work area (file system object) for logical drive */
  FIL file;       /* file object */
  FRESULT res;    /* FatFs function common result code */

  /* Register a work area for logical drive 0 */
  res = f_mount(0, &fs);
  if(res)
    debug("Err mount fs\n");
  
  /* Open config.txt file */
  res = f_open(&file, "config.txt", FA_OPEN_EXISTING | FA_READ);
  if (res)
    debug("File config.txt not found\n");
  else
  {
    bool    found;

    pLine = f_gets(line, sizeof(line), &file); /* read one line */
    while (pLine)
    {
      pToken = get_token (pLine, " =\t\n");
      if (pToken && *pToken != '#')
      {
        found = false;      
        for (j=0; (j < NUM_TOKENS) && !found; j++)
        {
          if (stricmp (pToken, config_lookup[j].name) == 0)
          {
            found = true;
            pToken = get_token (NULL, " \t\n");
            if (pToken && (*pToken == '='))
            {
              // get value
              pToken = get_token (NULL, " \t\n");
            
              if (pToken)
                *config_lookup[j].pValue = atoi (pToken);
              else
                sersendf ("Missing value for %s\r\n", config_lookup[j].name);
              
              // debug  
              //sersendf ("Found: %s = %d\r\n", config_lookup[j].name, *config_lookup[j].pValue);
            }
            else
              sersendf ("Expected '='%s\r\n", line);              
          }
        }
        
        if (!found)
          sersendf ("Unknown config: %s\r\n", pToken);
      }
      
      pLine = f_gets(line, sizeof(line), &file); /* read next line */
    }

    /* Close config.txt file */
    res = f_close(&file);
    if (res)
      debug("Error closing config.txt\n");
  }

  //
  //read_gcode_file ("autoexec.g");
  
  res = f_open(&file, "autoexec.g", FA_OPEN_EXISTING | FA_READ);
  if (res == FR_OK)
  {
    tLineBuffer line_buf;
    
    pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file); /* read one line */
    while (pLine)
    {
      line_buf.len = strlen(pLine);
      gcode_parse_line (&line_buf);
      pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file); /* read next line */
    }

    /* Close file */
    res = f_close(&file);
    if (res)
      debug("Error closing autoexec.g\n");
  }  
  
  // 
  
  /* Initialize using values readed from "config.txt" file */
  gcode_parse_init();
  dda_init();
}
