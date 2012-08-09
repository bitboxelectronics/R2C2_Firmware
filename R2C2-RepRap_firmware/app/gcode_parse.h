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

#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

#include "gcode_task.h"
#include "planner.h"


// whether to insist on N line numbers
// if not defined, line numbers are optional
//#define	REQUIRE_LINENUMBER

// whether to insist on a checksum
//#define	REQUIRE_CHECKSUM

#ifndef ABS
#define ABS(v)          (((v) >= 0)?(v):(-(v)))
#endif


/*
	Internal representations of values for motion axes, feedrate, uses double.
*/

typedef struct {

  // persistent states, retained between Gcode commands
	int32_t				  N_expected;

	uint8_t					option_relative			:1;
	uint8_t					option_inches				:1;

} tGcodeInterpreterState;

typedef struct 
{
  eArgType arg_type;
  union {
    float   float_val;
    int32_t int32_val;
    uint32_t uint32_val;
    char    *pStr;
  };
} tVariant;

// this holds all the possible data for a received command
typedef struct {
  // per command
  union {
    struct {
      uint8_t					seen_A	:1;
      uint8_t					seen_B	:1;
      uint8_t					seen_C	:1;
      uint8_t					seen_D	:1;
      uint8_t					seen_E	:1;
      uint8_t					seen_F	:1;
      uint8_t					seen_G	:1;
      uint8_t					seen_H	:1;
      uint8_t					seen_I	:1;
      uint8_t					seen_J	:1;
      uint8_t					seen_K	:1;
      uint8_t					seen_L	:1;
      uint8_t					seen_M	:1;
      uint8_t					seen_N	:1;
      uint8_t					seen_O	:1;
      uint8_t					seen_P	:1;
      uint8_t					seen_Q	:1;
      uint8_t					seen_R	:1;
      uint8_t					seen_S	:1;
      uint8_t					seen_T	:1;
      uint8_t					seen_U	:1;
      uint8_t					seen_V	:1;
      uint8_t					seen_W	:1;
      uint8_t					seen_X	:1;
      uint8_t					seen_Y	:1;
      uint8_t					seen_Z	:1;
    };
    uint32_t seen_words;
  };
  
  // command words, parameters
  double          A;
  double          B;
  double          C;

	uint8_t         G;
	uint8_t					G_fraction;

	int16_t				  M;
	int16_t				  T;
  tTarget					target; // info required for motion in planner/stepper X,Y,Z,E,F
	int32_t				  N;
	int16_t			  	P;
	int16_t					S;

  // for functions requiring a string parameter eg SD files
	uint8_t					str_pos;
	char            str_param [121];

  tGcodeInterpreterState state;

} GCODE_COMMAND;

  
typedef struct {
  // per command line, only used during checksum verify
	uint8_t					seen_checksum	: 1;
	uint8_t					checksum_read;          // value passed by host
	uint8_t					checksum_calculated;    // value calculated during parsing
} tChecksumData;



  
// the command that has been parsed
extern GCODE_COMMAND gcode_command;


void gcode_parse_init(void);

eParseResult gcode_parse_line (tGcodeInputMsg *pGcodeInputMsg);


#endif	/* GCODE_PARSE_H */
