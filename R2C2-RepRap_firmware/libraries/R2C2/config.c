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
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "spi.h"
#include "ff.h"
#include "lw_io.h"

// TODO: remove dependencies on these ?
#include "debug.h"    // may not be initialised yet
//#include "uart.h"     // may not be initialised yet

#define MAX_LINE_LEN 80

typedef struct
{
  uint8_t   type;
  union {
    int32_t   val_i;
    double    val_d;
    tPinDef   val_pin_def;
    };
} tParamVal;

#if 0
uint16_t read_u16 (FIL *file, char *line)
{
  f_gets(line, MAX_LINE_LEN, file); /* read one line */
  char *p_pos = strchr(line, '='); /* find the '=' position */
  
  if (p_pos != NULL)
    return (atoi(p_pos+1));
  else
    return 0;
}
#endif

// return true if c matches any character in s
static bool char_match (char c, char *s)
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
static char *get_token (char *pLine)
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

static double atod (char *s)
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

static bool parse_parameter_value (uint8_t type, tParamVal *param_val)
{
  char *pToken;

  pToken = get_token (NULL);

  if (pToken)
  {
    // should really derive type from token, not other way round
    param_val->type = type;
    switch (type)
    {
      case TYPE_INT:
      {
        param_val->val_i = atoi (pToken);
        break;
      }
      case TYPE_DOUBLE:
      {
        param_val->val_d = atod (pToken);
        break;
      }
      case TYPE_PIN_DEF:
      {
        // parse a list of up 4 to byte values
        // this is a bit hacky!
        int num=0;
        uint8_t val;
        uint8_t *pByte = (uint8_t *)&param_val->val_pin_def;

        param_val->val_pin_def.port = UNDEFINED_PORT;
        param_val->val_pin_def.pin_number = UNDEFINED_PIN_NUMBER;
        param_val->val_pin_def.active_low = 0;
        param_val->val_pin_def.reserved = 0;

        while (pToken && (num < 4) && isdigit (*pToken))
        {
          val = atoi (pToken);
          *pByte = val;

          num++;
          pByte++;

          pToken = get_token (NULL);
          if (pToken && (*pToken == ','))
            pToken = get_token (NULL);
          else
            // unexpected token
            pToken = NULL;
        }
        
      }
    }

    return true;
  }
  else
    return false;
}

// The output of this goes to Gcode interface
void print_config_table (const tConfigItem lookup[], int num_tokens)
{
  unsigned j;
  
  for (j=0; (j < num_tokens); j++)
  {
    switch (lookup[j].type)
    {
      case TYPE_INT:
      {
        int32_t *pVal = lookup[j].pValue;
        lw_printf ("%s = %d\r\n", lookup[j].name, *pVal);
        break;
      }
      case TYPE_DOUBLE:
      {
        double *pVal = lookup[j].pValue;
        lw_printf ("%s = %g\r\n", lookup[j].name, *pVal);
        break;
      }
      case TYPE_PIN_DEF:
      {
        tPinDef *pVal = (tPinDef *)lookup[j].pValue;
        lw_printf ("%s = %d,%d,%d,%d\r\n", lookup[j].name, pVal->port, pVal->pin_number, pVal->active_low, pVal->reserved );
        break;
      }
    }
  }
}

static unsigned long str_hash(char *str)
{
    unsigned long hash = 5381;
    int c;

    while (c = *str++)
        hash = ((hash << 5) + hash) + tolower(c); /* hash * 33 + c */

    return hash;
}

static int find_key (char *key, int num_tokens, const tConfigItem lookup[], tKeyHash hashes[])
{
  unsigned j;
  uint32_t hash;

  if (hashes == NULL)
  {
    for (j=0; (j < num_tokens); j++)
    {
      if (strcasecmp (key, lookup[j].name) == 0)
      {
        return j;
      }
    }
    return -1;
  }
  else
  {
    hash = str_hash (key);
    for (j=0; (j < num_tokens); j++)
    {
      if (hash == hashes[j].hash)
      {
        if (strcasecmp (key, lookup[j].name) == 0)
        {
          return j;
        }
      }
    }
    return -1;

  }

}


void create_key_hash_table (int num_tokens, const tConfigItem lookup[], tKeyHash hashes[])
{
  int j;
  for (j = 0; j < num_tokens; j++)
  {
    hashes[j].key = lookup[j].name;
    hashes[j].hash = str_hash(hashes[j].key);
  }
}


static  FIL file;       /* file object */

FRESULT read_config_file (char *filename, const tConfigItem lookup[], int num_tokens, tKeyHash hashes[])
{
  FRESULT res;
  char line [MAX_LINE_LEN];
  char *pLine;
  char *pToken;
  unsigned j;
  tParamVal param_val;
  
  /* Open the config file */
  res = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
  if (res)
  {
    lw_printf("Config: file not found\n");
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
        j = find_key (pToken, num_tokens, lookup, hashes);

        if (j != -1)
        {
            pToken = get_token (NULL);
            if (pToken && (*pToken == '='))
            {
              if (parse_parameter_value (lookup[j].type, &param_val))
              {
                switch (lookup[j].type)
                {
                  case TYPE_INT:
                  {
                    int32_t *pVal = lookup[j].pValue;
                    *pVal = param_val.val_i;
                    break;
                  }
                  case TYPE_DOUBLE:
                  {
                    double *pVal = lookup[j].pValue;
                    *pVal = param_val.val_d;
                    break;
                  }
                  case TYPE_PIN_DEF:
                  {
                    tPinDef *pVal = lookup[j].pValue;
                    *pVal = param_val.val_pin_def;
                    break;
                  }
                }
              }
              else
                lw_printf ("Missing value for %s\r\n", lookup[j].name);
            }
            else
              lw_printf ("Expected '='%s\r\n", line);              
        }
        else
          lw_printf ("Unknown config: %s\r\n", pToken);
      }
      
      pLine = f_gets(line, sizeof(line), &file); /* read next line */
    }

    /* Close config file */
    res = f_close(&file);
    if (res)
      lw_printf("Config: error closing file\n");
  }
  
  return res;
}


void set_defaults (const tConfigItem lookup[], int num_tokens)
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



