// CMSIS
#include	"LPC17xx.h"
#include	"core_cm3.h"

// NXP
#include	"lpc17xx_nvic.h"
#include	"lpc17xx_pinsel.h"
#include	"lpc17xx_gpio.h"

// FatFs
#include	"ff.h"

// LPCUSB
#include	"usbapi.h"

// Local
#include	"blockdev.h"
#include	"sbl_iap.h"
#include	"sbl_config.h"
#include	"msc_scsi.h"
#include	"spi.h"

FATFS fatfs;
FIL f;

typedef U16 CHECKSUM;

CHECKSUM checksum_file(FIL *file) {
	return 0;
}

CHECKSUM checksum_flash(void *addr, U32 length) {
	return 0;
}

// in msc_usb_start.c
void init_usb_msc_device(void);

// use variable from msc_scsi.c
BOOL usb_msc_not_ejected(void)
{
	return (ejected == 0)?TRUE:FALSE;
}

// from sbl_iap
BOOL bootloader_button_pressed(void)
{
	/* Configure bootloader IO button P4.29 */
	PINSEL_CFG_Type pin;
	pin.Portnum = 4;
	pin.Pinnum = 29;
	pin.Funcnum = PINSEL_FUNC_0;
	pin.Pinmode = PINSEL_PINMODE_PULLUP;
	pin.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&pin);

	/* set as input */
	GPIO_SetDir(4, (1<<29), 0);

	/* Verify if bootloader pin is activated */
	if(GPIO_ReadValue(4) & (1<<29))
		return FALSE;
	return TRUE;
}

int main() {
	// DeInit NVIC and SCBNVIC
	NVIC_DeInit();
	NVIC_SCBDeInit();

	/* Configure the NVIC Preemption Priority Bits:
	 * two (2) bits of preemption priority, six (6) bits of sub-priority.
	 * Since the Number of Bits used for Priority Levels is five (5), so the
	 * actual bit number of sub-priority is three (3)
	 */
	NVIC_SetPriorityGrouping(0x05);

	NVIC_SetVTOR(0x00000000);

	BlockDevInit();

	if (bootloader_button_pressed() || (user_code_present() == 0)) {
		init_usb_msc_device();

		for (;usb_msc_not_ejected();)
			USBHwISR();

		USBHwConnect(FALSE);
		spi_close();

		NVIC_SystemReset();
	}
	else {
		f_mount(0, &fatfs);
		if (f_open(&f, "/firmware.bin", FA_READ | FA_OPEN_EXISTING)) {
			unsigned int fs = f_size(&f);
			if ((fs > 0) && (fs <= USER_FLASH_SIZE)) {
				U16 cs_sd = checksum_file(&f);
				U16 cs_flash = checksum_flash((void *)USER_FLASH_START, USER_FLASH_START + fs);
				if (cs_sd != cs_flash) {
					f_lseek(&f, 0);
					U8 buffer[512];
					for (unsigned int i = 0; i < fs; i += 512) {
						unsigned int j = 512;
						if (i + j > fs)
							j = fs - i;
						f_read(&f, buffer, 512, NULL);
						write_flash((unsigned int *) (USER_FLASH_START + i), (char *) &buffer, 512);
					}
				}
				f_close(&f);
			}
		}
		// elm-chan's fatfs doesn't have an unmount function
		// f_umount(&fatfs);
		spi_close();

		if (user_code_present())
			execute_user_code();

		NVIC_SystemReset();
	}
}
