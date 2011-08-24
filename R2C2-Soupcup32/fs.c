#include	"fs.h"

#include	"sdcard.h"
#include	"spi.h"
#include	"ff.h"

FATFS		fatfs;
FIL			file;
FILINFO	fileinfo;

FRESULT fs_init() {
	spi_init();
	return f_mount(0, &fatfs);
}
