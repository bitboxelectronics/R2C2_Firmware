#ifndef  _BLOCKDEV_FLASH_H
#define  _BLOCKDEV_FLASH_H

#include "lpcusb_type.h"

int BlockDevGetSize(U32 *pdwDriveSize);
int BlockDevWrite(U32 dwSector, U8 * pbBuf);
int BlockDevRead(U32 dwSector, U8 * pbBuf);

#endif
