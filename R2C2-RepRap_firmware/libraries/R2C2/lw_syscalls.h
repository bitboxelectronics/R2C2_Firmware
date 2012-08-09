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


#ifndef _LW_SYSCALLS_H
#define _LW_SYSCALLS_H

#include <stdarg.h>

#if 0
// NB these numbers must correspond to 'Devices' table
#define DEV_UART0       0
#define DEV_UART1       1
#define DEV_UART2       2
#define DEV_UART3       3
#define DEV_USB_SERIAL  4

#define R2C2_STANDARD_INTERFACES

#ifdef R2C2_STANDARD_INTERFACES
// Standard config for R2C2 board

// Debug on "debug" UART, 
#define DEV_DBGOUT    DEV_UART3

#else
#error not defined
#endif

#endif
// ===================================================================


int _open (const char *name, int flags, int mode);
int _close(int file);
int _read(int file, char *ptr, int len);
int _write(int file, const char *ptr, int len) ;

int _ioctl(int file, int vmd, va_list args) ;

void _sys_init_devices(void);
#endif