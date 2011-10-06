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

#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include <stdbool.h>
#include	"gcode_parse.h"
#include "ff.h"


// for SD functions
extern FIL       file;
extern uint32_t  filesize;
extern uint32_t  sd_pos;
extern bool      sd_printing;     // printing from SD file
extern bool      sd_active;       // SD card active
extern bool      sd_writing_file; // writing to SD file

extern void sd_close (FIL *pFile);
extern bool sd_write_to_file(char *pStr, unsigned bytes_to_write);

extern bool sd_read_file (tLineBuffer *pLine);

// when we have a whole line, feed it to this
eParseResult process_gcode_command(void);

#endif	/* _GCODE_PROCESS_H */
