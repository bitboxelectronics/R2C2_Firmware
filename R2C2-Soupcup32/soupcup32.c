#include	"soupcup32.h"

#include	"fs.h"
#include	"serial.h"

#include	"config.h"

U8 linebuf[128];
int linebuf_p;

void soupcup32() {
	if (fs_init() != FR_OK) {
		serial_writestr("SD init failed! rebooting!\n");
		return;
	}

	for (;;) {
		if (serial_rxchars()) {
			U8 c;
			linebuf[linebuf_p++] = c = serial_popchar();
			if (c < 32) {
				linebuf_p = 0;
				serial_writestr("OK\n");
			}
		}
	}
}
