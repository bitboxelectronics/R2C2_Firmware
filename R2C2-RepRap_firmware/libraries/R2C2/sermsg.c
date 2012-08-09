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

#include	"sermsg.h"
#include	"lw_io.h"

// --------------------------------------------------------------------------
// to stdout
// --------------------------------------------------------------------------

void serwrite_hex4(uint8_t v)
{
  fserwrite_hex4 (stdout, v);
}

void serwrite_hex8(uint8_t v)
{
  fserwrite_hex8 (stdout, v);
}

void serwrite_hex16(uint16_t v)
{
  fserwrite_hex16 (stdout, v);
}

void serwrite_hex32(uint32_t v)
{
  fserwrite_hex32 (stdout, v);
}

// functions for sending decimal

void serwrite_uint32(uint32_t v)
{
  fserwrite_uint32 (stdout, v);
}

void serwrite_int32(int32_t v)
{
  fserwrite_int32 (stdout, v);
}

void serwrite_double(double v)
{
  fserwrite_double (stdout, v);
}



// --------------------------------------------------------------------------
// to LW_FILE
// --------------------------------------------------------------------------

void fserwrite_hex4(LW_FILE *f, uint8_t v) {
	v &= 0xF;
	if (v < 10)
		lw_fputc('0' + v, f);
	else
		lw_fputc('A' - 10 + v, f);
}

void fserwrite_hex8(LW_FILE *f, uint8_t v) {
	fserwrite_hex4(f, v >> 4);
	fserwrite_hex4(f, v & 0x0F);
}

void fserwrite_hex16(LW_FILE *f, uint16_t v) {
	fserwrite_hex8(f, v >> 8);
	fserwrite_hex8(f, v & 0xFF);
}

void fserwrite_hex32(LW_FILE *f, uint32_t v) {
	fserwrite_hex16(f, v >> 16);
	fserwrite_hex16(f, v & 0xFFFF);
}


void fserwrite_uint32(LW_FILE *f, uint32_t v) {
	uint8_t t = 0;
	if (v >= 1000000000) {
		for (t = 0; v >= 1000000000; v -= 1000000000, t++);
		lw_fputc(t + '0', f);
	}

	if (v >= 100000000) {
		for (t = 0; v >= 100000000; v -= 100000000, t++);
		lw_fputc(t + '0', f);
	}
	else if (t != 0)
		lw_fputc('0', f);

	if (v >= 10000000) {
		for (t = 0; v >= 10000000; v -= 10000000, t++);
		lw_fputc(t + '0', f);
	}
	else if (t != 0)
		lw_fputc('0', f);

	if (v >= 1000000) {
		for (t = 0; v >= 1000000; v -= 1000000, t++);
		lw_fputc(t + '0', f);
	}
	else if (t != 0)
		lw_fputc('0', f);

	if (v >= 100000) {
		for (t = 0; v >= 100000; v -= 100000, t++);
		lw_fputc(t + '0', f);
	}
	else if (t != 0)
		lw_fputc('0', f);

	if (v >= 10000) {
		for (t = 0; v >= 10000; v -= 10000, t++);
		lw_fputc(t + '0', f);
	}
	else if (t != 0)
		lw_fputc('0', f);

	if (v >= 1000) {
		for (t = 0; v >= 1000; v -= 1000, t++);
		lw_fputc(t + '0', f);
	}
	else if (t != 0)
		lw_fputc('0', f);

	if (v >= 100) {
		t = v / 100;
		lw_fputc(t + '0', f);
		v -= (t * 100);
	}
	else if (t != 0)
		lw_fputc('0', f);

	if (v >= 10) {
	  /* 99 > v > 10 */
		t = v / 10;
		lw_fputc(t + '0', f);
		v -= (t * 10);
	}
	else if (t != 0)
		lw_fputc('0', f);

	lw_fputc(v + '0', f);
}

void fserwrite_int32(LW_FILE *f, int32_t v) {
	if (v < 0) {
		lw_fputc('-', f);
		v = -v;
	}

	fserwrite_uint32(f, v);
}

void fserwrite_int32_wz (LW_FILE *f, int32_t v, uint8_t width, uint8_t zero) 
{
  char buf [12];
  uint16_t pos;
  int16_t sign = 0;

  buf [11] = 0;
  pos = 11;

	if (v < 0) 
  {
    sign = 1;
		v = -v;
	}

  if (v== 0)
  {
    pos--;
    buf[pos]= '0';
    sign = 0;
    if (width > 0)
      width--;
  }
  else
    while (v > 0)
    {
      pos--;
      buf[pos]= (v % 10) + '0';
      v = v / 10;
      if (width > 0)
        width--;
    }

  while(width && (pos>sign))
  {
    pos--;
    if (zero)
      buf[pos] = '0';
    else
      buf[pos] = ' ';
    width--;
  }

  if (sign)
  {
    pos--;
    buf[pos] = '-';
  }

	lw_fputs (&buf[pos], f);
}

void fserwrite_double(LW_FILE *f, double v)
{
  if (v < 0)
  {
    lw_fputc('-', f);
    v = -v;
  }
  
  /* print first part before '.' */
  fserwrite_uint32(f, (uint32_t) v);

  /* print the '.' */
  lw_fputc('.', f);

  /* print last part after '.' */
  v = v - (int32_t)v;

  v = v * 1000.0;
  if (v < 100.0)
  	lw_fputc('0', f);
  if (v < 10.0)
  	lw_fputc('0', f);
  fserwrite_uint32(f, (uint32_t) v);  	
  
}

