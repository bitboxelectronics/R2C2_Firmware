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


#include <inttypes.h>
#include <errno.h>

#ifndef _CROSSWORKS
#include <sys/unistd.h>
#endif

#include "uart.h"
#include "usb_serial.h" //TODO: usb ***

#include "lw_syscalls.h"
#include "lw_io.h"

extern LW_FILE file_table [];

#ifdef _CROSSWORKS
  #define STDIN_FILENO  0
  #define STDOUT_FILENO 1
  #define STDERR_FILENO 2
#endif

#ifndef EBADF
#define     EBADF 9
#endif

typedef struct {
  char *name;
  void (*dev_init) (void);
  void (*dev_putc) (char c);  
  char (*dev_getc) (void);
  int (*dev_bytes_available) (void);
} tDeviceDesc;

// DEVICES TABLE
// index by device number, not file number
static tDeviceDesc Devices[5] = {
  
  {"uart0",  uart0_init, uart0_send, uart0_receive, uart0_data_available}, 
  {"uart1",  uart1_init, uart1_send, uart1_receive, uart1_data_available}, 
  {"uart2",  uart2_init, uart2_send, uart2_receive, uart2_data_available}, 
  {"uart3",  uart3_init, uart3_send, uart3_receive, uart3_data_available}, 
  {"usbcdc", usb_serial_init, usb_serial_writechar, usb_serial_popchar, usb_serial_rxchars} // USB serial
};

static void dev_write_block (tDeviceDesc *pDevice, char *ptr, int len)
{
  int j;
  for (j=0; j < len; j++)
    pDevice->dev_putc (*ptr++);
}


/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len) 
{
  // write to a file
  int dev_num;
     
  if (file < MAX_FILES)
  {
    dev_num = file_table [file].dev_major;
    dev_write_block (&Devices[dev_num], ptr, len);
    return len;
  }

  errno = EBADF;
  return -1;
}