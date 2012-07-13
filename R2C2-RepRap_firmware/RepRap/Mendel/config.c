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
#include <ctype.h>
#include "stdlib.h"
#include "string.h"

#include "config.h"
#include "spi.h"
#include "ff.h"
#include "debug.h"
#include "gcode_parse.h"
#include "uart.h"

#include "pinout.h"
#include "stepper.h"


/* values reflecting the configuration of your system
 * numbers are integers or double
 */
struct configuration config;

#define TYPE_INT      0
#define TYPE_DOUBLE   1
#define TYPE_PIN_DEF  2
//#define TYPE_BYTE   2

#define MAX_LINE_LEN 80

typedef struct {
  char      *name;
  void      *pValue;
  uint8_t   type;
  union {
    int32_t   val_i;
    double    val_d;
    tPinDef   val_pin_def;
    };
} tConfigItem;

/* calculate the default values appropriate for your machine */
tConfigItem config_lookup [] = 
{
  //  
  // general config
  //
  
  { "machine_model",      &config.machine_model, TYPE_INT, {.val_i=0}},
  { "acceleration",       &config.acceleration, TYPE_DOUBLE, {.val_d=100.0}},         /* 100mm / second^2 */
  { "junction_deviation", &config.junction_deviation, TYPE_DOUBLE, {.val_d=0.05}},  

  { "debug_flags",        &config.debug_flags, TYPE_INT, {.val_i=0}},
  { "step_led_flash_method", &config.step_led_flash_method, TYPE_INT, {.val_i = STEP_LED_FLASH_VARIABLE}},
  
  { "auto_power_off_time", &config.auto_power_off_time, TYPE_INT, {.val_i = 0}},
  
  { "control_panel",      &config.control_panel, TYPE_INT, {.val_i = 0}},
  
  { "tcp_ip_enabled",    &config.tcp_ip_enabled, TYPE_INT, {.val_i = 0}},
  { "network_interface", &config.network_interface, TYPE_INT, {.val_i = 0}},

  //  
  // axis config
  //
    
  { "steps_per_mm_x", &config.axis[X_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=80}},
  { "steps_per_mm_y", &config.axis[Y_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=80}},
  { "steps_per_mm_z", &config.axis[Z_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=6400}},
  { "steps_per_mm_e", &config.axis[E_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=36}},    /* Wades extruder, NEMA 17 geared extruder (1/39 * 6.5mm) */

  /* used for G0 rapid moves and as a cap for all other feedrates */
  { "maximum_feedrate_x", &config.axis[X_AXIS].maximum_feedrate, TYPE_INT, {.val_i=3000}}, /* 50mm / second */
  { "maximum_feedrate_y", &config.axis[Y_AXIS].maximum_feedrate, TYPE_INT, {.val_i=3000}},
  { "maximum_feedrate_z", &config.axis[Z_AXIS].maximum_feedrate, TYPE_INT, {.val_i=60}},   /* 1mm / second */
  { "maximum_feedrate_e", &config.axis[E_AXIS].maximum_feedrate, TYPE_INT, {.val_i=3000}}, /* 50mm / second */

  // if axis acceleration is 0, general default acceleration will be used
  { "x.acceleration", &config.axis[X_AXIS].acceleration, TYPE_INT, {.val_i=0}},
  { "y.acceleration", &config.axis[Y_AXIS].acceleration, TYPE_INT, {.val_i=0}},
  { "z.acceleration", &config.axis[Z_AXIS].acceleration, TYPE_INT, {.val_i=0}},
  { "e.acceleration", &config.axis[E_AXIS].acceleration, TYPE_INT, {.val_i=0}},

  { "x.dir.invert", &config.axis[X_AXIS].dir_invert, TYPE_INT, {.val_i=0}},
  { "y.dir.invert", &config.axis[Y_AXIS].dir_invert, TYPE_INT, {.val_i=0}},
  { "z.dir.invert", &config.axis[Z_AXIS].dir_invert, TYPE_INT, {.val_i=0}},

  /* used when searching endstops and similar */
  { "search_feedrate_x", &config.axis[X_AXIS].search_feedrate, TYPE_INT, {.val_i=120}},
  { "search_feedrate_y", &config.axis[Y_AXIS].search_feedrate, TYPE_INT, {.val_i=120}},
  { "search_feedrate_z", &config.axis[Z_AXIS].search_feedrate, TYPE_INT, {.val_i=60}},
  { "search_feedrate_e", &config.axis[E_AXIS].search_feedrate, TYPE_INT, {.val_i=1600}},  // does E have endstop??
  
  { "homing_feedrate_x", &config.axis[X_AXIS].homing_feedrate, TYPE_INT, {.val_i=3000}},
  { "homing_feedrate_y", &config.axis[Y_AXIS].homing_feedrate, TYPE_INT, {.val_i=3000}},
  { "homing_feedrate_z", &config.axis[Z_AXIS].homing_feedrate, TYPE_INT, {.val_i=60}},
  
  // home pos is left front
  { "home_direction_x", &config.axis[X_AXIS].home_direction, TYPE_INT, {.val_i=-1}}, 
  { "home_direction_y", &config.axis[Y_AXIS].home_direction, TYPE_INT, {.val_i=-1}},
  { "home_direction_z", &config.axis[Z_AXIS].home_direction, TYPE_INT, {.val_i=-1}},
  
  { "home_pos_x", &config.axis[X_AXIS].home_pos, TYPE_INT, {.val_i=0}},
  { "home_pos_y", &config.axis[Y_AXIS].home_pos, TYPE_INT, {.val_i=0}},
  { "home_pos_z", &config.axis[Z_AXIS].home_pos, TYPE_INT, {.val_i=0}},

  { "printing_vol_x", &config.axis[X_AXIS].printing_vol , TYPE_INT, {.val_i=0}},
  { "printing_vol_y", &config.axis[Y_AXIS].printing_vol , TYPE_INT, {.val_i=0}},
  { "printing_vol_z", &config.axis[Z_AXIS].printing_vol , TYPE_INT, {.val_i=0}},
  
  //  
  // config for printers
  //

  // dump pos
  { "have_dump_pos", &config.have_dump_pos , TYPE_INT, {.val_i=0}},
  { "dump_pos_x", &config.dump_pos_x , TYPE_INT, {.val_i=0}},
  { "dump_pos_y", &config.dump_pos_x , TYPE_INT, {.val_i=0}},
  
  // rest pos
  { "have_rest_pos", &config.have_rest_pos , TYPE_INT, {.val_i=0}},
  { "rest_pos_x", &config.rest_pos_x , TYPE_INT, {.val_i=0}},
  { "rest_pos_y", &config.rest_pos_y , TYPE_INT, {.val_i=0}},

  // wipe pos
  { "have_wipe_pos",   &config.have_wipe_pos , TYPE_INT, {.val_i=0}},
  { "wipe_entry_pos_x", &config.wipe_entry_pos_x , TYPE_INT, {.val_i=0}},
  { "wipe_entry_pos_y", &config.wipe_entry_pos_y , TYPE_INT, {.val_i=0}},
  { "wipe_pos_x", &config.wipe_entry_pos_x , TYPE_INT, {.val_i=0}},     // DEPRECATED
  { "wipe_pos_y", &config.wipe_entry_pos_y , TYPE_INT, {.val_i=0}},     // DEPRECATED
  { "wipe_exit_pos_x", &config.wipe_exit_pos_x , TYPE_INT, {.val_i=0}},
  { "wipe_exit_pos_y", &config.wipe_exit_pos_y , TYPE_INT, {.val_i=0}},

  { "steps_per_revolution_e", &config.steps_per_revolution_e, TYPE_INT, {.val_i=3200}},  // 200 * 16
  
  { "wait_on_temp", &config.wait_on_temp, TYPE_INT, {.val_i=0}},
    
  { "enable_extruder_1", &config.enable_extruder_1, TYPE_INT, {.val_i=1}},
};

#define NUM_TOKENS(table) (sizeof(table)/sizeof(tConfigItem))

// This has R2C2 defaults
tConfigItem config_lookup_pindef [] = 
{

  // X Axis (axis 0)
  { "x.pin_step",   &config.axis[X_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = X_STEP_PIN  }},
  { "x.pin_dir",    &config.axis[X_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = X_DIR_PIN   }},
  { "x.pin_enable", &config.axis[X_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = X_ENABLE_PIN}},
  { "x.pin_reset",  &config.axis[X_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "x.pin_min_limit", &config.axis[X_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = X_MIN_PIN}},
  { "x.pin_max_limit", &config.axis[X_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  // Y Axis (axis 1)
  { "y.pin_step",   &config.axis[Y_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = Y_STEP_PIN   }},
  { "y.pin_dir",    &config.axis[Y_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = Y_DIR_PIN    }},
  { "y.pin_enable", &config.axis[Y_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = Y_ENABLE_PIN }},
  { "y.pin_reset",  &config.axis[Y_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "y.pin_min_limit", &config.axis[Y_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = Y_MIN_PIN    }},
  { "y.pin_max_limit", &config.axis[Y_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // Z Axis (axis 2)
  { "z.pin_step",   &config.axis[Z_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = Z_STEP_PIN   }},
  { "z.pin_dir",    &config.axis[Z_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = Z_DIR_PIN    }},
  { "z.pin_enable", &config.axis[Z_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = Z_ENABLE_PIN }},
  { "z.pin_reset",  &config.axis[Z_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "z.pin_min_limit", &config.axis[Z_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = Z_MIN_PIN        }},
  { "z.pin_max_limit", &config.axis[Z_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // E Axis (axis 3)
  { "e.pin_step",   &config.axis[E_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = E_STEP_PIN   }},
  { "e.pin_dir",    &config.axis[E_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = E_DIR_PIN    }},
  { "e.pin_enable", &config.axis[E_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = E_ENABLE_PIN }},
  { "e.pin_reset",  &config.axis[E_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "e.pin_min_limit", &config.axis[E_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},
  { "e.pin_max_limit", &config.axis[E_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},

};

uint16_t read_u16 (FIL *file, char *line)
{
  f_gets(line, MAX_LINE_LEN, file); /* read one line */
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

// a version of strtok(), recognises identifiers, integers, and single-char symbols
// NB: very unsafe; not re-entrant. Use with caution!
char *get_token (char *pLine)
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

  if (!pNext)
    return NULL;

  // skip white space
  while (*pNext && char_match (*pNext, " \t\n") )
  {
    pNext ++;
  }
      
  if (*pNext == 0)
    // reached end of string
    return NULL;
  else
  {  
    // find next token
    pToken = pNext;

    if (isalpha (*pNext))
    {
      // identifier is alpha (alpha|digit|"_"|".")*
      while (*pNext && ( isalpha(*pNext) || isdigit(*pNext) || (*pNext == '_' ) || (*pNext == '.' )) )
      {
        pNext ++;
      }
    }
    else if (isdigit (*pNext) || char_match (*pNext, "+-"))
    {
      // number is [+|-] (digit)+ [. digit+]
      pNext ++;
      while (*pNext && isdigit (*pNext) )
      {
        pNext ++;
      }
      if (*pNext && *pNext == '.')
      {
        pNext ++;
        while (*pNext && isdigit (*pNext) )
        {
          pNext ++;
        }
      }
    }
    else
    {
       // anything else is presumed to be single char token, e.g. "="
       pNext ++;
    }

    saved = *pNext;
    *pNext = 0;
    return pToken;
  }
}

double atod (char *s)
{
  double result = 0.0;
  int num_places = 0;
  double frac = 0.0;
  
  while (*s && *s != '.')
  {
    result *= 10.0;
    result += *s-'0';
    s++;
  }
  if (*s && *s=='.')
  {
    s++;
    
    while (*s)
    {
      frac *= 10.0;
      frac += *s-'0';
      s++;
      num_places++;
    }
    while (num_places--)
      frac /= 10.0;
    result += frac;
  }
  return result;
}

void print_config (void)
{
  unsigned j;
  
  for (j=0; (j < NUM_TOKENS(config_lookup)); j++)
  {
    switch (config_lookup[j].type)
    {
      case TYPE_INT:
      {
        int32_t *pVal = config_lookup[j].pValue;
        sersendf ("%s = %d\r\n", config_lookup[j].name, *pVal);
        break;
      }
      case TYPE_DOUBLE:
      {
        double *pVal = config_lookup[j].pValue;
        sersendf ("%s = %g\r\n", config_lookup[j].name, *pVal);
        break;
      }
    }
  }
}

FRESULT read_config_file (char *filename, tConfigItem *lookup, int num_tokens)
{
  FIL file;       /* file object */
  FRESULT res;
  char line [MAX_LINE_LEN];
  char *pLine;
  char *pToken;
  unsigned j;
  
  /* Open the config file */
  res = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
  if (res)
  {
    debug("Config: file not found\n");
  }
  else
  {
    bool    found;

    pLine = f_gets(line, sizeof(line), &file); /* read one line */
    while (pLine)
    {
      pToken = get_token (pLine);
      if (pToken && *pToken != '#')
      {
        found = false;      
        for (j=0; (j < num_tokens) && !found; j++)
        {
          if (stricmp (pToken, lookup[j].name) == 0)
          {
            found = true;
            pToken = get_token (NULL);
            if (pToken && (*pToken == '='))
            {
              // get value
              pToken = get_token (NULL);
            
              if (pToken)
              {
                switch (lookup[j].type)
                {
                  case TYPE_INT:
                  {
                    int32_t *pVal = lookup[j].pValue;
                    *pVal = atoi (pToken);
                    break;
                  }
                  case TYPE_DOUBLE:
                  {
                    double *pVal = lookup[j].pValue;
                    *pVal = atod(pToken);
                    break;
                  }
                }
                // debug  
                //sersendf ("Found: %s = %d\r\n", lookup[j].name, *lookup[j].pValue);
              }
              else
                sersendf ("Missing value for %s\r\n", lookup[j].name);
              
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

    /* Close config file */
    res = f_close(&file);
    if (res)
      debug("Config: error closing file\n");
  }
  
  return res;
}

static void set_defaults (tConfigItem *lookup, int num_tokens)
{
  unsigned j;

  for (j=0; j < num_tokens; j++)
  {
    switch (lookup[j].type)
    {
      case TYPE_INT:
      {
        int32_t *pVal = lookup[j].pValue;
        *pVal = lookup[j].val_i;
        break;
      }
      case TYPE_DOUBLE:
      {
        double *pVal = lookup[j].pValue;
        *pVal = lookup[j].val_d;
        break;
      }
      case TYPE_PIN_DEF:
      {
        tPinDef *pVal = lookup[j].pValue;
        *pVal = lookup[j].val_pin_def;
        break;
      }
    }
  }
}

void read_config (void)
{
  char *pLine;

  // first set defaults
  set_defaults (config_lookup, NUM_TOKENS(config_lookup));
  
  set_defaults (config_lookup_pindef, NUM_TOKENS(config_lookup_pindef));
  
  // set default axis map for R2C2 - printer
  config.num_axes = NUM_AXES;
  config.axis[0].letter_code = 'X';
  config.axis[0].is_configured = true;
  config.axis[1].letter_code = 'Y';
  config.axis[1].is_configured = true;
  config.axis[2].letter_code = 'Z';
  config.axis[2].is_configured = true;
  config.axis[3].letter_code = 'E';
  config.axis[3].is_configured = true;
    
  /* initialize SPI for SDCard */
  spi_init();

  /* access to "config.txt" file on SDCard */

  FATFS fs;       /* Work area (file system object) for logical drive */
  FIL file;       /* file object */
  FRESULT res;    /* FatFs function common result code */

  /* Register a work area for logical drive 0 */
  res = f_mount(0, &fs);
  if (res)
    debug("Err mount fs\n");
  else  
  {
    res = read_config_file ("config.txt", config_lookup, NUM_TOKENS(config_lookup));
    
    res = read_config_file ("config_pin.txt", config_lookup_pindef, NUM_TOKENS(config_lookup_pindef));
    
  
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
  
  }
  // 
  
  /* Initialize using values read from "config.txt" file */
  gcode_parse_init();

}


