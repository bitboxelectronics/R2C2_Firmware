#include	"soupcup32.h"

#include	"fs.h"
#include	"serial.h"

#include	"config.h"

void soupcup32() {
	if (fs_init() != FR_OK) {
		serial_writestr("SD init failed! rebooting!\n");
		return;
	}


}
