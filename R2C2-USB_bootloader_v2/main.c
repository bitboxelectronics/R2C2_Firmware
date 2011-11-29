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
#include	"uart.h"
#include	"debug.h"

FATFS fatfs;
FIL f;
FRESULT r;

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

	#ifdef	UARTDEBUG
		uart_init();
	#endif

        if (BlockDevInit() == 0)
        {
        /* uSDCard is present */
#ifdef	UARTDEBUG
            if (1) {
                U32 size;
                BlockDevGetSize(&size);
                DBG("Found SD card of size %d", size);
                BlockDevGetBlockLength(&size);
                DBG("block length %d", size);
            }
#endif

            if (bootloader_button_pressed() || (user_code_present() == 0)) {
                DBG("entering bootloader");
                init_usb_msc_device();

                for (;usb_msc_not_ejected();)
                  USBHwISR();

                DBG("usb ejected, rebooting");

                USBHwConnect(FALSE);
                spi_close();
            }
            else {
                if ((r = f_mount(0, &fatfs)) == FR_OK) {
                    if ((r = f_open(&f, "/firmware.bin", FA_READ | FA_OPEN_EXISTING)) == FR_OK) {
                        unsigned int fs = f_size(&f);
                        DBG("found firmware.bin with %u bytes", fs);
                        if ((fs > 0) && (fs <= USER_FLASH_SIZE)) {
                            U8 buffer[FLASH_BUF_SIZE];
                            for (unsigned int i = 0; i < fs; i += FLASH_BUF_SIZE) {
                                unsigned int j = FLASH_BUF_SIZE;
                                if (i + j > fs)
                                  j = fs - i;
                                DBG("writing %d-%d", i, i+j);
                                if ((r = f_read(&f, buffer, j, &j)) == FR_OK) {
                                    // pad last block to a full sector size
                                    while (j < FLASH_BUF_SIZE) {
                                        buffer[j++] = 0xFF;
                                    }
                                    write_flash((unsigned int *) (USER_FLASH_START + i), (char *) &buffer, j);
                                }
                                else {
                                    DBG("read failed: %d", r);
                                    i = fs;
                                }
                            }
                            r = f_close(&f);
                            r = f_unlink("/firmware.bck");
                            r = f_rename("/firmware.bin", "/firmware.bck");
                        }
                    }
                    else {
                        DBG("open \"/firmware.bin\" failed: %d", r);
                    }
#ifdef	GENERATE_FIRMWARE_CUR
                    if (f_open(&f, "/firmware.bck", FA_READ | FA_OPEN_EXISTING)) {
                        f_close(&f);
                    }
                    else {
                        // no firmware.bck, generate one!
                        if (f_open(&f, "/firmware.bck", FA_WRITE | FA_CREATE_NEW) == FR_OK) {
                            U8 *flash = (U8 *) USER_FLASH_START;

                            f_close(&f);
                        }
                    }
#endif
                    // elm-chan's fatfs doesn't have an unmount function
                    // f_umount(&fatfs);
                }
                else {
                    DBG("mount failed: %d", r);
                }
                spi_close();

                if (user_code_present()) {
                    DBG("starting user code...");
                    execute_user_code();
                }
                else {
                    DBG("user code invalid, rebooting");
                }
            }
            NVIC_SystemReset();
        }

        else
        {
          /* probably uSDCard is not on the socket so we should execute user code/firmware */
          DBG("uSDCard init failed...");
          DBG("starting user code...");
          execute_user_code();
        }
}
