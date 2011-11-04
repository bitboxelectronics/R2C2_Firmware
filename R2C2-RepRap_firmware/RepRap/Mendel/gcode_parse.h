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

#ifndef	GCODE_PARSE_H
#define	GCODE_PARSE_H

#include	<stdint.h>

#include "planner.h"

// whether the asterisk (checksum-command) is included for checksum calculation
// undefined for RepRap host software
//#define ASTERISK_IN_CHECKSUM_INCLUDED

// wether to insist on N line numbers
// if not defined, N's are completely ignored
//#define	REQUIRE_LINENUMBER

// wether to insist on a checksum
//#define	REQUIRE_CHECKSUM

#ifndef ABS
#define ABS(v)          (((v) >= 0)?(v):(-(v)))
#endif

#if 0
// this is a very crude decimal-based floating point structure. a real floating point would at least have signed exponent
typedef struct {
	uint32_t	sign			:1;
	uint64_t	mantissa	:24;
	uint32_t	exponent	:7;
} decfloat;

// target is simply a point in space/time
typedef struct {
        int64_t                                         X;
        int64_t                                         Y;
        int64_t                                         Z;
//        int64_t                                         E;
        double E;
        uint64_t                                        F;
} TARGET;
#endif

// this holds all the possible data from a received command
typedef struct {
	uint8_t					seen_G	:1;
	uint8_t					seen_M	:1;
	uint8_t					seen_X	:1;
	uint8_t					seen_Y	:1;
	uint8_t					seen_Z	:1;
	uint8_t					seen_E	:1;
	uint8_t					seen_F	:1;
	uint8_t					seen_S	:1;

	uint8_t					seen_P	:1;
	uint8_t					seen_N	:1;
	uint8_t					seen_checksum				:1;
	uint8_t					seen_semi_comment		:1;
	uint8_t					seen_parens_comment	:1;
	uint8_t					getting_string				:1;

	uint8_t					option_relative			:1;
	uint8_t					option_inches				:1;

	uint8_t						G;
	uint16_t				  M;
	tTarget						target;

	int16_t						S;
	uint16_t					P;

	uint32_t					N;
	uint32_t					N_expected;

	uint8_t						checksum_read;
	uint8_t						checksum_calculated;

  // for SD functions
	uint8_t						chpos;
	char              filename [120];
} GCODE_COMMAND;

#define MAX_LINE 120
typedef struct
{
  char    data [MAX_LINE];
  int     len;
  uint8_t seen_lf :1;
} tLineBuffer;

typedef enum {
  PR_OK,
  PR_RESEND,
  PR_ERROR,
  PR_BUSY
  } eParseResult;

  
// the command being processed
extern GCODE_COMMAND next_target;

// utility functions
//int32_t	decfloat_to_int(decfloat *df, int32_t multiplicand, int32_t denominator);


void gcode_parse_init(void);

eParseResult gcode_parse_line (tLineBuffer *pLine);


#endif	/* GCODE_PARSE_H */
