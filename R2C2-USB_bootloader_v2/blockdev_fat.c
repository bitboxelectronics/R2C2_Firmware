#include	"diskio.h"
#include	"blockdev.h"

DSTATUS disk_initialize (BYTE id) {
	return BlockDevInit();
}

DSTATUS disk_status (BYTE b) {
	return BlockDevGetStatus();
}

DRESULT disk_read (BYTE Drive, BYTE* buffer, DWORD SectorNumber, BYTE SectorCount) {
	int i;
	for (i = 0; i < SectorCount; i++) {
		if (BlockDevRead(SectorNumber + i, &buffer[i * 512]))
			return RES_ERROR;
	}
	return 0;
}

#if     _READONLY == 0
DRESULT disk_write (BYTE Drive, const BYTE* buffer, DWORD SectorNumber, BYTE SectorCount) {
	int i;
	for (i = 0; i < SectorCount, i++) {
		if (BlockDevWrite(SectorNumber + i, &buffer[i * 512]))
			return RES_ERROR;
	}
	return 0;
}
#endif

DRESULT disk_ioctl (BYTE Drive, BYTE Command, void* buffer) {
	switch(Command) {
		case CTRL_SYNC:
			break;
		case GET_SECTOR_SIZE:
			*((WORD *) buffer) = 512;
			break;
		case GET_SECTOR_COUNT:
			BlockDevGetSize((DWORD *) buffer);
			break;
		case GET_BLOCK_SIZE:
			*((DWORD *) buffer) = 512;
			break;
		case CTRL_ERASE_SECTOR:
			break;
	}
	return 0;
}

void disk_timerproc( void );
