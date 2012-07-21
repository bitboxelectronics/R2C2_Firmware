/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com              */
/* **************************************************************************
   All rights reserved.

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
****************************************************************************/
// **************************************************************************
// Description:
//
// **************************************************************************

#ifndef _STR_BUFFER_H
#define _STR_BUFFER_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "stdbool.h"
#include "stdint.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define MAX_SIZE 128

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef struct {
  volatile int head;
  volatile int tail;
  volatile char *pData;
  int max_size;

  // flags
  volatile uint16_t  in_use :1;
  volatile uint16_t  err_overflow :1;

 } tStrBufHeader;

typedef struct {
  tStrBufHeader header;
  char          data [MAX_SIZE];
} tStrBuffer;

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void str_buf_init (tStrBuffer *pHeader, char *pData, int max_size);
void str_buf_init_std (tStrBuffer *pBuffer);

bool str_buf_putc (tStrBuffer *pHeader, char c);
bool str_buf_puts (tStrBuffer *pHeader, char *s);

int str_buf_getc (tStrBuffer *pHeader);
int str_buf_gets (tStrBuffer *pHeader, char *s, int max_len);

int str_buf_get_size (tStrBuffer *pHeader);

bool str_buf_is_full (tStrBuffer *pHeader);
bool str_buf_is_empty (tStrBuffer *pHeader);


// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif // _STR_BUFFER_H

