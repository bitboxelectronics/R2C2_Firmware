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
#include	"serial.h"

void serwrite_hex4(uint8_t v) {
	v &= 0xF;
	if (v < 10)
		serial_writechar('0' + v);
	else
		serial_writechar('A' - 10 + v);
}

void serwrite_hex8(uint8_t v) {
	serwrite_hex4(v >> 4);
	serwrite_hex4(v & 0x0F);
}

void serwrite_hex16(uint16_t v) {
	serwrite_hex8(v >> 8);
	serwrite_hex8(v & 0xFF);
}

void serwrite_hex32(uint32_t v) {
	serwrite_hex16(v >> 16);
	serwrite_hex16(v & 0xFFFF);
}

void serwrite_uint32(uint32_t v) {
	uint8_t t = 0;
	if (v >= 1000000000) {
		for (t = 0; v >= 1000000000; v -= 1000000000, t++);
		serial_writechar(t + '0');
	}

	if (v >= 100000000) {
		for (t = 0; v >= 100000000; v -= 100000000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 10000000) {
		for (t = 0; v >= 10000000; v -= 10000000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 1000000) {
		for (t = 0; v >= 1000000; v -= 1000000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 100000) {
		for (t = 0; v >= 100000; v -= 100000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 10000) {
		for (t = 0; v >= 10000; v -= 10000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 1000) {
		for (t = 0; v >= 1000; v -= 1000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 100) {
		t = v / 100;
		serial_writechar(t + '0');
		v -= (t * 100);
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 10) {
	        /* 99 > v > 10 */
		t = v / 10;
		serial_writechar(t + '0');
		v -= (t * 10);
	}
	else if (t != 0)
		serial_writechar('0');

	serial_writechar(v + '0');
}

void serwrite_int32(int32_t v) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32(v);
}

void serwrite_double(double v)
{
  if (v < 0)
  {
    serial_writechar ('-');
    v = -v;
  }
  
  /* print first part before '.' */
  serwrite_uint32((uint32_t) v);

  /* print the '.' */
  serial_writechar('.');

  /* print last part after '.' */
  v = v - (int32_t)v;

  v = v * 1000.0;
  if (v < 100.0)
  	serial_writechar('0');
  if (v < 10.0)
  	serial_writechar('0');
  serwrite_uint32((uint32_t) v);  	
  
}

