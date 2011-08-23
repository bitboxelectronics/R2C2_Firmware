#ifndef  _BLOCKDEV_FLASH_H
#define  _BLOCKDEV_FLASH_H

int BlockDevGetSize(unsigned long *pdwDriveSize);
int BlockDevWrite(unsigned long dwSector, unsigned char * pbBuf);
int BlockDevRead(unsigned long dwSector, unsigned char * pbBuf);

#endif
