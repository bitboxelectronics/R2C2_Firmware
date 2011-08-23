#include "sdcard.h"
#include "sersendf.h"

int BlockDevGetSize(unsigned long *pdwDriveSize)
{
  MMC_disk_ioctl(GET_SECTOR_COUNT, pdwDriveSize);

  return 0;
}

int BlockDevWrite(unsigned long dwSector, unsigned char * pbBuf)
{
  MMC_disk_write(pbBuf, dwSector, 512);

  return 0;
}

int BlockDevRead(unsigned long dwSector, unsigned char * pbBuf)
{
  MMC_disk_read(pbBuf, dwSector, 512);

  return 0;
}
