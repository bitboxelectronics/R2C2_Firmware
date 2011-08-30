/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs                          */
/* (C)ChaN, 2007                                                         */
/* Copyright (c) 2010, Martin Thomas                                     */
/* Copyright (c) 2010, Jorge Pinto, Hardware_Box                         */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "integer.h"
#include "diskio.h"
#include "sdcard.h"


/*-----------------------------------------------------------------------*/
/* disk-timer - forwarded to low-level drivers                           */
/*-----------------------------------------------------------------------*/

void disk_timerproc(void)
{
      MMC_disk_timerproc();
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE drv)
{
  (void) drv;

  DSTATUS stat;
  stat = MMC_disk_initialize();
  return stat;
}

/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(BYTE drv)
{
  (void) drv;

  DSTATUS stat;
  stat = MMC_disk_status();
  return stat;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(BYTE drv, /* Physical drive number (0..) */
                BYTE *buff, /* Data buffer to store read data */
                DWORD sector, /* Sector address (LBA) */
                BYTE count /* Number of sectors to read (1..255) */
)
{
  (void) drv;

  DRESULT res;

  res = MMC_disk_read(buff, sector, count);
  return res;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
/* The FatFs module will issue multiple sector transfer request
 /  (count > 1) to the disk I/O layer. The disk function should process
 /  the multiple sector transfer properly Do. not translate it into
 /  multiple single sector transfers to the media, or the data read/write
 /  performance may be drastically decreased. */

#if _READONLY == 0
DRESULT disk_write(BYTE drv, /* Physical drive number (0..) */
        const BYTE *buff, /* Data to be written */
        DWORD sector, /* Sector address (LBA) */
        BYTE count /* Number of sectors to write (1..255) */
)
{
  (void) drv;

  DRESULT res;
  res = MMC_disk_write(buff, sector, count);
  return res;
}
#endif /* _READONLY */

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(BYTE drv, /* Physical drive number (0..) */
BYTE ctrl, /* Control code */
void *buff /* Buffer to send/receive control data */
)
{
  (void) drv;

  DRESULT res;

  res = MMC_disk_ioctl(ctrl, buff);
  return res;
}

