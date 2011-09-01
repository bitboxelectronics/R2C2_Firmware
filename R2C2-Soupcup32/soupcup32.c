#include	"soupcup32.h"

#include	"fs.h"
#include	"serial.h"

#include	"config.h"

#define	MAX_LINE_LENGTH 128

unsigned char ser_linebuf[MAX_LINE_LENGTH];
int ser_linebuf_p;

unsigned char sd_linebuf[MAX_LINE_LENGTH];
int sd_linebuf_p;

int sd_printing = 0;

void soupcup32() {
	if (fs_init() != FR_OK) {
		serial_writestr("SD init failed! rebooting!\n");
		return;
	}

	for (;;) {
		if (serial_rxchars()) {
			unsigned char c;
			ser_linebuf[ser_linebuf_p++] = c = serial_popchar();
			if (c < 32) {
				ser_linebuf[ser_linebuf_p] = 0;
				ser_linebuf_p = 0;
				serial_writestr("OK\n");
			}
		}
		
		if (sd_printing) {
			
		}
	}
}
