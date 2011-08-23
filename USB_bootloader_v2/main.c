#include	<type.h>
#include	<sdcard.h>
#include	<usb.h>
#include	<ff.h>

int main() {
	init_lpc();

	init_sd();

	if (BOOTLOADER_BUTTON_PRESSED) {
		init_usb_msc_device();

		for (;usb_msc_not_ejected();)
			usb_msc_isr();

		shutdown_usb();
		shutdown_sd();

		full_reboot();
	}
	else {
		mount_sd();
		FILE f;
		int fs = f_sizeof("/firmware.bin");
		if ((fs > 0) && (fs <= (FLASH_END - BOOTLOADER_SIZE))) {
			if (f = f_open("/firmware.bin", O_READ)) {
				U16 checksum_sd = checksum_file(f);
				U16 checksum_flash = checksum_flash(BOOTLOADER_SIZE, BOOTLOADER_SIZE + fs);
				if (checksum_sd != checksum_flash) {
					f_seek(f, 0);
					U8 buffer[512];
					for (int i = 0; i < fs; i += 512) {
						int j = 512;
						if (i + j > fs)
							j = fs - i;
						f_read(f, buffer, 512);
						IAP_WRITE(65536 + i, buffer, 512);
					}
				}
				f_close(f);
			}
		}
		umount_sd();

		user_program();
	}
}
