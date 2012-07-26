/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
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

// **************************************************************************
// Description:
//
// **************************************************************************

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "string.h"

#include "str_buffer.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

static inline int inc_wrap (tStrBuffer *pHeader, int ptr)
{
  ptr++;
  if (ptr == pHeader->header.max_size)
    return 0;
  else
    return ptr;
}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void str_buf_init (tStrBuffer *pHeader, char *pData, int max_size)
{
  memset (&pHeader->header, 0, sizeof (tStrBufHeader));

  pHeader->header.head = 0;
  pHeader->header.tail = 0;
  pHeader->header.max_size = max_size;
  pHeader->header.pData = pData;
}

void str_buf_init_std (tStrBuffer *pBuffer)
{
  str_buf_init (pBuffer, pBuffer->data, MAX_SIZE);
}


bool str_buf_putc (tStrBuffer *pHeader, char c)
{
  if (str_buf_is_full(pHeader))
    return false;
  else
  {
    pHeader->header.pData[pHeader->header.tail] = c;
    pHeader->header.tail = inc_wrap (pHeader, pHeader->header.tail);
    return true;
  }

}

bool str_buf_puts (tStrBuffer *pHeader, char *s)
{
  char c;
  int len = strlen (s);
  while (len--)
  {
    if (str_buf_is_full(pHeader))
      return false;
    else
    {
      pHeader->header.pData[pHeader->header.tail] = c;
      pHeader->header.tail = inc_wrap (pHeader, pHeader->header.tail);
    }
  }

  c = 0;
  if (str_buf_is_full(pHeader))
    return false;
  else
  {
    pHeader->header.pData[pHeader->header.tail] = c;
    pHeader->header.tail = inc_wrap (pHeader, pHeader->header.tail);
  }
}


int str_buf_getc (tStrBuffer *pHeader)
{
  char c;

  if (str_buf_is_empty(pHeader))
    return 0;
  else
  {   
    c = pHeader->header.pData[pHeader->header.head];
    pHeader->header.head = inc_wrap (pHeader, pHeader->header.head);
    return c;
  }
}

int str_buf_gets (tStrBuffer *pHeader, char *s, int max_len)
{
  int count = 0;
  char c; 

  while (max_len--)
  {
    if (str_buf_is_empty(pHeader))
      break;
    else
    {
      c = pHeader->header.pData[pHeader->header.head];
      pHeader->header.head = inc_wrap (pHeader, pHeader->header.head);

      *s = c;
      s++;
      count++;
      if (c==0) break;
    }
  }
  return count;
}


int str_buf_get_size (tStrBuffer *pHeader)
{
  //TODO
  return 0;
}


bool str_buf_is_full (tStrBuffer *pHeader)
{
  int next_tail = inc_wrap (pHeader, pHeader->header.tail);

  if ( pHeader->header.head == next_tail)
    return true;
  else
    return false;
}

bool str_buf_is_empty (tStrBuffer *pHeader)
{
  if ( pHeader->header.head == pHeader->header.tail)
    return true;
  else
    return false;
}


// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
