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

#include "serial.h"

void serial_init()
{
  fifo_init(&rxfifo, rxbuf);
  fifo_init(&txfifo, txbuf);
}

/*
        Read
*/

int serial_rxchars()
{
  return fifo_avail(&rxfifo);
}

unsigned char serial_popchar()
{
  unsigned char c = 0;

  fifo_get(&rxfifo, &c);

  return c;
}

/*
        Write
*/

int serial_txchars()
{
  return fifo_avail(&txfifo);
}

void serial_writechar(unsigned char data)
{
  fifo_put(&txfifo, data);
}

void serial_writeblock(void *data, int datalen)
{
  int i;

  for (i = 0; i < datalen; i++)
    serial_writechar(((unsigned char *) data)[i]);
}

void serial_writestr(void *data)
{
  int i = 0;
	unsigned char r;

  while ((r = ((unsigned char *) data)[i++]))
    serial_writechar(r);
}
