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

/* --------------------------------------------------------------------------

  Adding a new device, e.g. "mydev"

  1. write functions mydev_init, mydev_putchar, mydev_getchar as needed
  2. include "mydev.h" in this file
  3. put device entry into Devices table (below)

  That's it!

  open the device with :

    LW_FILE *my_file = lw_fopen ("mydev", "rw");

  then use LW_IO

    lw_fprintf (my_file, "Hello mydev world\n");

---------------------------------------------------------------------------*/



#include <inttypes.h>
#include <errno.h>
#include "stdbool.h"
#include "string.h"

#ifndef _CROSSWORKS
#include <sys/unistd.h>
#endif

#include "uart.h"
#include "usb_serial.h"
#include "LiquidCrystal.h"

#include "lw_syscalls.h"
#include "lw_io.h"
#include "lw_ioctl.h"

extern LW_FILE file_table [];


#ifndef EBADF
#define     EBADF 9
#endif

typedef struct {
  char *name;
//  int   dev_major;
  void (*dev_init) (void);
  void (*dev_putc) (char c);  
  char (*dev_getc) (void);
  int (*dev_rx_avail) (void);
//  int (*dev_ioctl) (int cmd, argList args);
} tDeviceDesc;

// DEVICES TABLE
// index by device number, not file number
static tDeviceDesc Devices[] = {
  
  {"uart0", uart0_init, uart0_send, uart0_receive, uart0_data_available}, 
  {"uart1",  uart1_init, uart1_send, uart1_receive, uart1_data_available}, 
//TODO:  {"uart2",  uart2_init, uart2_send, uart2_receive, uart2_data_available}, 
  {"uart3",  uart3_init, uart3_send, uart3_receive, uart3_data_available}, 
  {"usbser", usb_serial_init, usb_serial_writechar, usb_serial_popchar, usb_serial_rxchars}, // USB serial
  {"lcd", lcd_initialise, lcd_writechar, NULL, NULL} 
};

#define NUM_DEVICES sizeof(Devices) / sizeof(tDeviceDesc)

static void dev_write_block (tDeviceDesc *pDevice, const char *ptr, int len)
{
  int j;
  for (j=0; j < len; j++)
    pDevice->dev_putc (*ptr++);
}

static void dev_read_block (tDeviceDesc *pDevice, char *ptr, int len)
{
  int j;
  for (j=0; j < len; j++)
  {
    *ptr = pDevice->dev_getc ();
    ptr++;
  }
}

void _sys_init_devices(void)
{
  int j;
  for (j=0; j < NUM_DEVICES; j++)
    Devices[j].dev_init();
}

// ---------------------------------------------------------------------------
// File/device functions
// ---------------------------------------------------------------------------

int _open (const char *name, int flags, int mode)
{
  bool found;
  int dev_num;
  int handle;

  found = false;
  for (dev_num=0 ; dev_num < NUM_DEVICES ; dev_num++ )
  {
    if (strcmp (Devices[dev_num].name, name) == 0)
    {
      found = true;
      break;
    }
  }

  if (!found)
    return -1;

  // find next free
  found = false;
  for (handle = 0; handle < MAX_FILES; handle++)
  {
    if (!file_table[handle].in_use)
    {
      found = true;
      break;
    }
  }

  if (found)
  {
    file_table[handle].in_use = 1;
    file_table[handle].dev_major = dev_num;
    file_table[handle].dev_minor = 0;
    file_table[handle].handle = handle;
    file_table[handle].flags = flags;
    file_table[handle].mode = mode;

    return handle;
  }
  else
    return -1;
}

int _close(int file) 
{
  return -1;
}

/*
 read
 Read characters from a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */
int _read(int file, char *ptr, int len) 
{
  int dev_num;

  if (len == 0)
    return 0;
       
  if (file < MAX_FILES)
  {
    dev_num = file_table [file].dev_major;
    dev_read_block (&Devices[dev_num], ptr, len);
    return len;
  }
  else
  {
    errno = EBADF;
    return -1;
  }
}

/*
 write
 Write characters to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, const char *ptr, int len) 
{
  // write to a file
  int dev_num;
   
  if (file < MAX_FILES)
  {
    dev_num = file_table [file].dev_major;
    dev_write_block (&Devices[dev_num], ptr, len);
    return len;
  }
  else
  {
    errno = EBADF;
    return -1;
  }
}

int _ioctl(int file, int cmd, va_list args) 
{
  int dev_num;
  int result;
   
  if (file < MAX_FILES)
  {
    dev_num = file_table [file].dev_major;

    switch (cmd)
    {
      case LW_FIONREAD:
      {
        int *pNum = va_arg (args, int *);

        if (pNum != NULL)
          *pNum = Devices[dev_num].dev_rx_avail();
         
        result = 0;
        //errno = 0;
      }
      break;

      default:
        errno = EINVAL;
        result = -1;
    }
  }
  else
  {
    errno = EBADF;
    result = -1;
  }

  return result;

}
