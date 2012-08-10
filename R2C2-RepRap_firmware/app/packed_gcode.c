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

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include  <string.h>
#include  <stdbool.h>
#include  <ctype.h>

#include "packed_gcode.h"
#include "gcode_defs.h"




// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

static inline void push_byte (tLineBuffer *pBuf, uint8_t byte_val)
{
  if (pBuf->len < MAX_LINE)
    pBuf->data [pBuf->len++] = byte_val;
}

static uint8_t get_byte ( tLineBuffer *pLine)
{
  return pLine->data [pLine->ch_pos++];
}



static int32_t get_int (tVariant *pVariant)
{
  switch (pVariant->arg_type)
  {
    case arg_int32:
      return pVariant->int32_val;
    break;

    case arg_bcd:
      return 0;
    break;

    case arg_float:
      return pVariant->float_val;
    break;

    case arg_string:
      return 0;
    break;
  }
} 

static float get_float (tVariant *pVariant)
{
  switch (pVariant->arg_type)
  {
    case arg_int32:
      return pVariant->int32_val;
    break;

    case arg_bcd:
      return 0;
    break;

    case arg_float:
      return pVariant->float_val;
    break;

    case arg_string:
      return 0;
    break;
  }
} 

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void gcode_add_packed_command(tLineBuffer *pBuf, uint8_t cmd)
{
  push_byte (pBuf, cmd);
}

// code should be a-z, A-Z or *
void gcode_add_packed_command_int (tLineBuffer *pBuf, uint8_t cmd, int32_t data)
{
  eArgType arg_type;

  if (data < 0)
  {
    // signed
    if ( (data < INT16_MIN))
      arg_type = arg_int32;
    else if ( (data < INT8_MIN))
      arg_type = arg_int16;
    else
      arg_type = arg_int8;

  }
  else
  {
    if ( (data > UINT16_MAX))
      arg_type = arg_int32;
    else if ( data > UINT8_MAX)
      arg_type = arg_uint16;
    else
      arg_type = arg_uint8;
  }

  cmd = toupper (cmd);

  if ( (cmd >= 'A') && (cmd <= 'Z'))
    cmd = ((cmd-64) << 3) | arg_type;
  else
    cmd = CODE_STAR;

  push_byte (pBuf,cmd);

  // must be at least one byte
  push_byte (pBuf, data & 0xff);
  data >>= 8;

  if (arg_type >= arg_uint16)
  {
    push_byte (pBuf, data & 0xff);
    data >>= 8;
  }

  if (arg_type >= arg_int32)
  {
    push_byte (pBuf, data & 0xff);
    data >>= 8;
    push_byte (pBuf, data & 0xff);
    data >>= 8;
  }
}

// code should be a-z, A-Z
void gcode_add_packed_command_float (tLineBuffer *pBuf, uint8_t cmd, float val)
{
  eArgType arg_type;
  tVariant data;

  arg_type = arg_float;

  cmd = toupper (cmd);
  if ( (cmd >= 'A') && (cmd <= 'Z'))
    cmd = ((cmd-64) << 3) | arg_type;
  else
    return;

  push_byte (pBuf,cmd);

  data.float_val = val;

  push_byte (pBuf, data.uint32_val & 0xff);
  data.uint32_val >>= 8;
  push_byte (pBuf, data.uint32_val & 0xff);
  data.uint32_val >>= 8;

  push_byte (pBuf, data.uint32_val & 0xff);
  data.uint32_val >>= 8;
  push_byte (pBuf, data.uint32_val & 0xff);
  data.uint32_val >>= 8;
}

// code should be a-z, A-Z, CODE_COMMENT, CODE_RAW
void gcode_add_packed_command_str (tLineBuffer *pBuf, uint8_t cmd, char *pData)
{
  eArgType arg_type;

  arg_type = arg_string;

  cmd = toupper (cmd);
  if ( (cmd >= 'A') && (cmd <= 'Z'))
    cmd = ((cmd-64) << 3) | arg_type;
  else
    return;

  push_byte (pBuf, cmd);

  int len = strlen (pData);

  while (len)
  {
    push_byte (pBuf, *pData++);
    len--;
  }
  push_byte (pBuf, 0);
}



bool parse_packed_gcode (tGcodeInputMsg *pGcodeInputMsg, GCODE_COMMAND *pCommand)
{
  bool result = false;
  tLineBuffer *pLine;
  uint8_t   cmd;
  uint8_t   type;
  int32_t   param;
  tVariant variant;

  pLine = pGcodeInputMsg->pLineBuf;

  while (pLine->ch_pos < pLine->len)
  {
    cmd = get_byte (pLine); 

    pCommand->seen_words = 0;

    while (cmd != CODE_END_COMMAND)
    {
      type = cmd & 7;
      cmd = cmd >> 3;


      switch (type)
      {
        case arg_uint8:
        {
          param = get_byte (pLine);

          variant.arg_type = arg_int32;
          variant.int32_val = param;
        }
        break;

        case arg_uint16:
        {
          param = get_byte (pLine);
          param = param | get_byte (pLine) << 8;

          variant.arg_type = arg_int32;
          variant.int32_val = param;
        }
        break;

        case arg_int8:
        {
          param = get_byte (pLine);
          if (param >= 0x80)
            param = param | 0xffffff00;

          variant.arg_type = arg_int32;
          variant.int32_val = param;
        }
        break;

        case arg_int16:
        {
          param = get_byte (pLine);
          param = param | get_byte (pLine) << 8;
          if (param >= 0x8000)
            param = param | 0xffff0000;

          variant.arg_type = arg_int32;
          variant.int32_val = param;
        }
        break;

        case arg_int32:
          param = get_byte (pLine);
          param = param | get_byte (pLine) << 8;
          param = param | get_byte (pLine) << 16;
          param = param | get_byte (pLine) << 24;

          variant.arg_type = arg_int32;
          variant.int32_val = param;
        break;

        case arg_bcd:
        break;

        case arg_float:
          param = get_byte (pLine);
          param = param | get_byte (pLine) << 8;
          param = param | get_byte (pLine) << 16;
          param = param | get_byte (pLine) << 24;

          variant.arg_type = arg_float;
          variant.uint32_val = param;
        break;

        case arg_string:
        break;
      }

      //pCommand->seen_words |= (1 << (cmd-1));
      cmd = cmd + 64;
      switch (cmd)
      {
        case 'G':
          pCommand->seen_G = 1;
          pCommand->G = get_int (&variant);
          break;
        case 'X':
          pCommand->seen_X = 1;
          pCommand->target.x = get_float (&variant);
          break;
        case 'Y':
          pCommand->seen_Y = 1;
          pCommand->target.y = get_float (&variant);
          break;
        case 'Z':
          pCommand->seen_Z = 1;
          pCommand->target.z = get_float (&variant);
          break;
      }

      cmd = get_byte (pLine); 
    }

    result = true;

    if (cmd == CODE_END_COMMAND)
      return true;
    
  }

  return result;
}


