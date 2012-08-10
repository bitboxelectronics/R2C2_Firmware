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

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include  <string.h>
#include  <stdbool.h>
#include  <ctype.h>

#include  "app_config.h"
#include	"gcode_parse.h"
#include	"gcode_process.h"
#include	"packed_gcode.h"
#include	"lw_io.h"
#include  "machine.h"


// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define crc(a, b)		(a ^ b)

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// This holds the parsed command used by gcode_process
GCODE_COMMAND gcode_command;

// tGcodeInterpreterState interpreter_state;

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// context variables used during parsing of a line
static struct {
  char last_field; // letter code

  struct {
    uint8_t   sign; 
    int       exponent;
  } read_digit;

  double value;

	uint8_t					seen_semi_comment		:1;
	uint8_t					seen_parens_comment	:1;
	uint8_t					getting_hack_string :1;
	uint8_t					getting_string			:1;

} parsing_context;

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// parse next character
static void gcode_parse_char(tLineBuffer *pLine, tGcodeInterpreterState *pInterpreterState);


// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

static uint8_t char_to_hex (char c)
{
  c= tolower(c);
  if ( (c >='0') && ( c <= '9'))
    return c - '0';
  else if ( (c >='a') && ( c <= 'f'))
    return c - 'a' + 10;
  else
    return 0;
}

static int hex_to_bin (char *s, int len)
{
  int ch_pos;
  int byte_pos;
  uint8_t byte_val;

  byte_pos = 0;
  ch_pos = 0;

  while (ch_pos < len)
  {
    byte_val = char_to_hex (s[ch_pos]);
    ch_pos ++;
    if (ch_pos < len)
    {
      byte_val = (byte_val << 4) + char_to_hex (s[ch_pos]);
    }
    ch_pos ++;

    s[byte_pos++] = byte_val;
  }
  return byte_pos;
}

/****************************************************************************
*                                                                           *
* Request a resend of the expected line - called by gcode_process_line()    *
*                                                                           *
* Relies on the global variable gcode_command.N_expected being valid.       *
*                                                                           *
****************************************************************************/

static void request_resend (tGcodeInputMsg *pGcodeInputMsg, tGcodeInterpreterState *pState) 
{
	lw_fprintf(pGcodeInputMsg->out_file, "rs %lu\r\n", pState->N_expected);
}

static void calc_checksum (tLineBuffer *pLine, tChecksumData *pChecksumData, GCODE_COMMAND *pCommand) 
{
  uint8_t c;
  int ch_pos;
  int sign = 1;
  bool skip = false;

  pCommand->seen_N = 0;
  pCommand->N = 0;

  pChecksumData->seen_checksum = 0;
  pChecksumData->checksum_calculated = 0;
  pChecksumData->checksum_read = 0;

  //TODO: what if '*' in comment?
  // a checksum should be at end of line in form "*nnn", but impossible to tell if is part of comment or string
  ch_pos = 0;
  while (ch_pos < pLine->len)
  {
    c = pLine->data [ch_pos];

    if ((c==10) || (c==13))
      break;

    if (pChecksumData->seen_checksum == 0)
    {
      if (c=='*')
      {
        pChecksumData->seen_checksum = 1;
      }
      else
        pChecksumData->checksum_calculated = crc(pChecksumData->checksum_calculated, c);
    }
    else
    {
      pChecksumData->checksum_read = pChecksumData->checksum_read * 10 + (c - '0');
    }
    
    //
    if (!skip)
    {
      if ( (c == ' ') || (c=='\t') )
        ;
      else if ( (c == '/') || (c==';') )
        skip = true;
      else
      {
        if ((c == 'N') || (c=='n'))
          pCommand->seen_N = 1;
        else if (c=='-')
          sign = -1;
        else if ( (c >= '0') && (c<='9'))
          pCommand->N = pCommand->N*10 + c-'0';
        else
          skip = true;
          
      }
    }

    ch_pos++;
  }

  pCommand->N = pCommand->N * sign;
}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

/*
	utility functions
*/

double power (double x, int exp)
{  
  double result = 1.0;
  while (exp--)  
    result = result * x;
  return result;
}

double inch_to_mm (double inches)
{  
  return inches * 25.4;
}


void gcode_parse_init(void)
{
#if 0
  steps_per_m_x = ((uint32_t) (config.steps_per_mm_x * 1000.0));
  steps_per_m_y = ((uint32_t) (config.steps_per_mm_y * 1000.0));
  steps_per_m_z = ((uint32_t) (config.steps_per_mm_z * 1000.0));
  steps_per_m_e = ((uint32_t) (config.steps_per_mm_e * 1000.0));

  // same as above with 25.4 scale factor
  steps_per_in_x = ((double) (25.4 * config.steps_per_mm_x));
  steps_per_in_y = ((double) (25.4 * config.steps_per_mm_y));
  steps_per_in_z = ((double) (25.4 * config.steps_per_mm_z));
  steps_per_in_e = ((double) (25.4 * config.steps_per_mm_e));
#endif  
  gcode_command.target.feed_rate = config.axis[Z_AXIS].homing_feedrate;
}


eParseResult gcode_parse_line (tGcodeInputMsg *pGcodeInputMsg) 
{
  //int j;
  int len;
  tLineBuffer *pLine;
  eParseResult result = PR_OK;
  tChecksumData checksum_data;


  if (pGcodeInputMsg->type == GC_PACKED)
  {
    pLine = pGcodeInputMsg->pLineBuf;
    pLine->ch_pos = 0;

    while (parse_packed_gcode(pGcodeInputMsg, &gcode_command))
    {
      // process
      result = process_gcode_command(pGcodeInputMsg, &gcode_command.state);
    }
    return result;
  }
  
  // Note: blank lines (zero length) should have been discarded already
  // should blank lines get an "ok"?

  // init
  parsing_context.seen_semi_comment = 0;
  parsing_context.seen_parens_comment = 0;

  parsing_context.last_field = 0;
  parsing_context.read_digit.sign = parsing_context.read_digit.exponent = 0;
  parsing_context.value = 0;

  pLine = pGcodeInputMsg->pLineBuf;

  // check checksum is valid before processing data!
  calc_checksum (pLine, &checksum_data, &gcode_command);

#ifdef	REQUIRE_CHECKSUM
  if ( (checksum_data.checksum_calculated == checksum_data.checksum_read) && (checksum_data.seen_checksum == 1))
#else
  if ( (checksum_data.checksum_calculated == checksum_data.checksum_read) || (checksum_data.seen_checksum == 0))
#endif
  {
    // checksum is valid or not present, or not required
  }
  else
  {
    lw_fprintf(pGcodeInputMsg->out_file, "Expected checksum %u\r\n", checksum_data.checksum_calculated);
    request_resend(pGcodeInputMsg, &gcode_command.state);
    result = PR_RESEND;
    return result;
  }
  
  // check line number

  //Note: according to GCode standard strict sequencing of line numbers is not required, but GCode was not designed
  //      to be a robust comms protocol...
  //Note: pronterface will send "M-1 M110" to set line number to zero. Assume that valid N should be >= 0 and ignore <0.
  if (
    #ifdef  REQUIRE_LINENUMBER
      (gcode_command.N == gcode_command.state.N_expected) && (gcode_command.seen_N == 1)
    #else
      (gcode_command.seen_N == 0) || (gcode_command.N <0) || (gcode_command.N == gcode_command.state.N_expected) 
    #endif
      ) 
  {
    // OK
  }
  else
  {
    lw_fprintf(pGcodeInputMsg->out_file, "Expected line number %lu\r\n", gcode_command.state.N_expected);
    request_resend(pGcodeInputMsg, &gcode_command.state);
    result = PR_RESEND;
    return result;
  }

  // parse the line
  //TODO: parse_char never detects errors? there must be some surely
  pLine->ch_pos = 0;
  while (pLine->ch_pos < pLine->len)
  {
    gcode_parse_char (pLine, &gcode_command.state);
    pLine->ch_pos++;
  }
    
  //TODO: more than one command per line
  //TODO: gcode context for each interface

  // --------------------------------------------------------------------------
  //     SD
  // --------------------------------------------------------------------------
  if (sd_writing_file)
  {
    // parser can bail at first M code?
    if (gcode_command.seen_M && (gcode_command.M >= 20) && (gcode_command.M <= 29) )
    {
      if (gcode_command.seen_M && (gcode_command.M == 29))
      { 
        // M29 - stop writing
        sd_writing_file = false;
        sd_close (&file);
        lw_fputs("Done saving file\r\n", pGcodeInputMsg->out_file);
      }
      else
      {
        // else - do not write SD M-codes to file
        lw_fputs("ok\r\n", pGcodeInputMsg->out_file);
      }
    }
    else
    {
      // lines in files must be LF terminated for sd_read_file to work
      if (pLine->data [pLine->len-1] == 13)
        pLine->data [pLine->len-1] = 10;
        
      if (gcode_command.seen_M && (gcode_command.M == 30))
      {
        if (file_mode == FM_HEX_BIN)
        {
          len = hex_to_bin (gcode_command.str_param, strlen(gcode_command.str_param));
        }
        else
        {
          strcat (gcode_command.str_param, "\n");
          len = strlen(gcode_command.str_param);
        }

        if (sd_write_to_file(gcode_command.str_param, len))
          lw_fputs("ok\r\n", pGcodeInputMsg->out_file);
        else
          lw_fputs("error writing to file\r\n", pGcodeInputMsg->out_file);

      }
      else
      {
        if (sd_write_to_file(pLine->data, pLine->len))
          lw_fputs("ok\r\n", pGcodeInputMsg->out_file);
        else
          lw_fputs("error writing to file\r\n", pGcodeInputMsg->out_file);
      }
    }
  }
  // --------------------------------------------------------------------------
  //     Not in SD write mode
  // --------------------------------------------------------------------------
  else
  {
    // process
    result = process_gcode_command(pGcodeInputMsg, &gcode_command.state);

    if (result != PR_BUSY)
    {
      // expect next line number
      if (gcode_command.seen_N == 1)
          gcode_command.state.N_expected = gcode_command.N + 1;
    }
  }

  // reset variables
  gcode_command.seen_words = 0;
  gcode_command.str_pos = 0;

  // dont assume a G1 by default
  gcode_command.G = 0;

  //TODO:
  if (gcode_command.state.option_relative) 
  {
    gcode_command.target.x = gcode_command.target.y = gcode_command.target.z = 0.0;
    gcode_command.target.e = 0.0;
  }

	return result;    
}

/****************************************************************************
*                                                                           *
* Character Received - add it to our command                                *
*                                                                           *
****************************************************************************/

void gcode_parse_char(tLineBuffer *pLine, tGcodeInterpreterState *pInterpreterState) 
{
  char c;
   
  c = pLine->data[pLine->ch_pos];

	// uppercase version for case-insensitive comparisons of GCode letters
	if (c >= 'a' && c <= 'z')
		c &= ~32;

	// process previous field
	if (parsing_context.last_field) {
		// check if we're seeing a new field or end of line
		// any character will start a new field, even invalid/unknown ones
		if ((c >= 'A' && c <= 'Z') || c == '*' || (c == 10) || (c == 13)) {
		
		  // before using value, apply the sign
		  if (parsing_context.read_digit.sign)
		    parsing_context.value = -parsing_context.value;
		    
			switch (parsing_context.last_field) {
				case 'G':
					gcode_command.G = parsing_context.value;
					break;
				case 'M':
					gcode_command.M = parsing_context.value;
					// this is a horrible hack since string parameters don't fit in general G code syntax
					// NB: filename MUST start with a letter and MUST NOT contain spaces
					// letters will also be converted to uppercase
					if ( (gcode_command.M == 23) || (gcode_command.M == 28) || (gcode_command.M == 302) )
					{
					    parsing_context.getting_string = 1;
					    parsing_context.getting_hack_string = 1;
					}
					break;

        // axis coords, feed rate, converted to mm
				case 'X':
					if (pInterpreterState->option_inches)
						gcode_command.target.x = inch_to_mm(parsing_context.value);
					else
						gcode_command.target.x = parsing_context.value;
					break;
				case 'Y':
					if (pInterpreterState->option_inches)
						gcode_command.target.y = inch_to_mm(parsing_context.value);
					else
						gcode_command.target.y = parsing_context.value;
					break;
				case 'Z':
					if (pInterpreterState->option_inches)
						gcode_command.target.z = inch_to_mm(parsing_context.value);
					else
						gcode_command.target.z = parsing_context.value;
					break;
				case 'E':
					if (pInterpreterState->option_inches)
						gcode_command.target.e = inch_to_mm(parsing_context.value);
					else
						gcode_command.target.e = parsing_context.value;
					break;
				case 'F':
          // TODO : is this valid for inverse feed-rates?
					if (pInterpreterState->option_inches)
						gcode_command.target.feed_rate = inch_to_mm(parsing_context.value);
					else
						gcode_command.target.feed_rate = parsing_context.value;
					break;

        // other integer parameters
				case 'S':
						gcode_command.S = parsing_context.value;
					break;
				case 'P':
					// if this is dwell, multiply by 1000 to convert seconds to milliseconds
					if (gcode_command.G == 4)
						gcode_command.P = parsing_context.value * 1000.0;
					else
						gcode_command.P = parsing_context.value;
					break;

				case 'N':
					gcode_command.N = parsing_context.value;
					break;
			}
			// reset for next field
			parsing_context.last_field = 0;
			parsing_context.read_digit.sign = parsing_context.read_digit.exponent = 0;
			parsing_context.value = 0;
		}
	}

  if (parsing_context.getting_string)
  {
    if ( (c == 10) || (c == 13) || ( c == '\"') ||
         ( parsing_context.getting_hack_string && (( c == ' ')  || ( c == '*')) )
    )
    {
      parsing_context.getting_string = 0;
      parsing_context.getting_hack_string = 0;
    }
    else
    {
      if (gcode_command.str_pos < sizeof(gcode_command.str_param))
      {
        gcode_command.str_param [gcode_command.str_pos++] = pLine->data[pLine->ch_pos];
        gcode_command.str_param [gcode_command.str_pos] = 0;
      }
    }      
  }

	// skip comments, strings, filenames
	if (parsing_context.seen_semi_comment == 0 && parsing_context.seen_parens_comment == 0 && parsing_context.getting_string == 0) {
		// new field?
		if ((c >= 'A' && c <= 'Z') || c == '*') {
			parsing_context.last_field = c;
		}

		// process character
		switch (c) {
			// each currently known command is either G or M, so preserve previous G/M unless a new one has appeared
			// FIXME: same for T command
			case 'G':
				gcode_command.seen_G = 1;
				break;
			case 'M':
				gcode_command.seen_M = 1;
				break;
			case 'X':
				gcode_command.seen_X = 1;
				break;
			case 'Y':
				gcode_command.seen_Y = 1;
				break;
			case 'Z':
				gcode_command.seen_Z = 1;
				break;
			case 'E':
				gcode_command.seen_E = 1;
				break;
			case 'F':
				gcode_command.seen_F = 1;
				break;
			case 'S':
				gcode_command.seen_S = 1;
				break;
			case 'P':
				gcode_command.seen_P = 1;
				break;
			case 'N':
				gcode_command.seen_N = 1;
				break;

			// comments
			case ';':
			case '^':
				parsing_context.seen_semi_comment = 1;
				break;
			case '(':
				parsing_context.seen_parens_comment = 1;

        // look for "meta-comment" e.g. (STR"some text")
        // also (msg,...) etc

        if ((pLine->ch_pos < pLine->len-4) && (strncmp (&pLine->data[pLine->ch_pos+1], "STR\"", 4) == 0))
        {
          gcode_command.str_pos = 0;
          parsing_context.getting_string = 1;
          pLine->ch_pos += 4;
        }
				break;

			// now for some numeracy
			case '-':
				parsing_context.read_digit.sign = 1;
				// force sign to be at start of number, so 1-2 = -2 instead of -12
				parsing_context.read_digit.exponent = 0;
				break;
			case '.':
				if (parsing_context.read_digit.exponent == 0)
					parsing_context.read_digit.exponent = 1;
				break;


			default:
				// can't do ranges in switch..case, so process actual digits here
				if (c >= '0' && c <= '9') {
					if (parsing_context.read_digit.exponent == 0)
					{
					  parsing_context.value = parsing_context.value * 10 + (c - '0');
					}
					else
					  parsing_context.value += (double)(c - '0') / power(10, parsing_context.read_digit.exponent);

					if (parsing_context.read_digit.exponent)
						parsing_context.read_digit.exponent++;
				}
		}
	} else if ( parsing_context.seen_parens_comment == 1)
  {
    if (c == ')')
      parsing_context.seen_parens_comment = 0; // recognize stuff after a (comment)
    else if (parsing_context.getting_string)
    {
      if (c == '\"')
        parsing_context.getting_string = 0;
    }
  }

}

