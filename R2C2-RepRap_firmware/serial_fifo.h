/*
	LPCUSB, an USB device driver for LPC microcontrollers	
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in the
	   documentation and/or other materials provided with the distribution.
	3. The name of the author may not be used to endorse or promote products
	   derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef SerialFifo_h
#define SerialFifo_h

#include "lpcusb_type.h"

#define SERIAL_FIFO_SIZE 256

typedef struct {
	volatile int head;
	volatile int tail;
	volatile U8 *buf;
} fifo_t;

void fifo_init(fifo_t *fifo, unsigned char *buf);
unsigned char fifo_put(fifo_t *fifo, unsigned char c);
unsigned char _fifo_put(fifo_t *fifo, unsigned char c);
unsigned char fifo_get(fifo_t *fifo, unsigned char *pc);
unsigned char _fifo_get(fifo_t *fifo, unsigned char *pc);
int  fifo_avail(fifo_t *fifo);
int  _fifo_avail(fifo_t *fifo);
int fifo_free(fifo_t *fifo);
int _fifo_free(fifo_t *fifo);

#endif
