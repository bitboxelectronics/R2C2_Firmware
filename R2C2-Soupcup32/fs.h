#ifndef	_FS_H
#define	_FS_H

#include	"ff.h"

extern FATFS		fatfs;
extern FIL			file;
extern FILINFO	fileinfo;

FRESULT fs_init(void);

#endif	/* _FS_H */
