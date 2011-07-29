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

#include "config.h"
#include "spi.h"
#include "ff.h"
#include "debug.h"
#include "stdlib.h"
#include "string.h"
#include "gcode_parse.h"
#include "dda.h"

/* values reflecting the gearing of your machine
 * all numbers are integers, so no decimals, please :-)
 */
struct configuration config =
{
  /* calculate these values appropriate for your machine */
  .steps_per_mm_x = 80,
  .steps_per_mm_y = 80,
  .steps_per_mm_z = 6400,
  .steps_per_mm_e = 36, /* Wades extruder, NEMA 17 geared extruder (1/39 * 6.5mm) */

  /* used for G0 rapid moves and as a cap for all other feedrates */
  .maximum_feedrate_x = 3000, /* 50mm / second */
  .maximum_feedrate_y = 3000,
  .maximum_feedrate_z = 60,  /* 1mm / second */
  .maximum_feedrate_e = 3000, /* 50mm / second */

  /* used when searching endstops and similar */
  .search_feedrate_x = 120,
  .search_feedrate_y = 120,
  .search_feedrate_z = 60,
  .search_feedrate_e = 1600
};

uint16_t read_u16 (FIL *file, uint8_t *line)
{
  f_gets(line, 80, file); /* read one line */
  uint8_t *p_pos = strchr(line, '='); /* find the '=' position */
  return (atoi(p_pos+1));
}

void read_config (void)
{
  uint8_t line[80];

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
    uint16_t temp;
    temp = read_u16(&file, line);
    if (temp) config.steps_per_mm_x = temp;
    temp = read_u16(&file, line);
    if (temp) config.steps_per_mm_y = temp;
    temp = read_u16(&file, line);
    if (temp) config.steps_per_mm_z = temp;
    temp = read_u16(&file, line);
    if (temp) config.steps_per_mm_e = temp;
    temp = read_u16(&file, line);
    if (temp) config.maximum_feedrate_x = temp;
    temp = read_u16(&file, line);
    if (temp) config.maximum_feedrate_y = temp;
    temp = read_u16(&file, line);
    if (temp) config.maximum_feedrate_z = temp;
    temp = read_u16(&file, line);
    if (temp) config.maximum_feedrate_e = temp;
    temp = read_u16(&file, line);
    if (temp) config.search_feedrate_x = temp;
    temp = read_u16(&file, line);
    if (temp) config.search_feedrate_y = temp;
    temp = read_u16(&file, line);
    if (temp) config.search_feedrate_z = temp;
    temp = read_u16(&file, line);
    if (temp) config.search_feedrate_e = temp;

    /* Close config.txt file */
    res = f_close(&file);
    if (res)
      debug("Error closing config.txt\n");
  }

  /* Initialize using values readed from "config.txt" file */
  gcode_parse_init();
  dda_init();
}
