#include "sbl_config.h"
#include "sbl_iap.h"
#include "lpcusb_type.h"
#include "disk.h"

extern uint32_t user_flash_erased; /* from main_bootloader.c */

uint8_t offset_address_flag = FALSE;
uint32_t offset_address;


int BlockDevGetSize(uint32_t *pdwDriveSize)
{
  *pdwDriveSize = (512 * 1024)- sector_start_map[USER_START_SECTOR];

  return 0;
}

int BlockDevWrite(uint32_t dwSector, uint8_t * pbBuf)
{
  uint8_t * firmware;
  firmware = (uint8_t *)USER_FLASH_START;
  uint32_t address;
  uint32_t length = 512;
  uint32_t i;

  address = 512 * dwSector;

  if (( address >= BOOT_SECT_SIZE) && \
      ( address < (BOOT_SECT_SIZE + FAT_SIZE + ROOT_DIR_SIZE)))
  {
    for ( i = 0; i<length; i++)
    {
      Fat_RootDir[(address+i) - BOOT_SECT_SIZE] = pbBuf[i];

      if ( pbBuf[i] == 0xe5 )
      {
        if ( (address+i) == BOOT_SECT_SIZE + FAT_SIZE + 32 )
        {
          /* Delete user flash when firmware.bin is erased */
          if( user_flash_erased == FALSE )
          {
            erase_user_flash();
            user_flash_erased = TRUE;
            offset_address_flag = TRUE;
          }
        }
      }
    }
  }
  else if (address >= (BOOT_SECT_SIZE + FAT_SIZE + ROOT_DIR_SIZE))
  {
    /* Save offset_address -- Linux OS may decide to write on any part of the
     * free space??  */
    if (offset_address_flag == TRUE)
    {
      offset_address = address;
      offset_address_flag = FALSE;
    }

    write_flash((unsigned *)(firmware + (address - offset_address)),(char *)pbBuf,length);
  }

  return 0;
}

int BlockDevRead(uint32_t dwSector, uint8_t * pbBuf)
{
  uint32_t address;
  uint32_t i;
  uint8_t data;
  uint8_t * firmware;
  firmware = (uint8_t *)USER_FLASH_START;

  uint32_t length =512;

  address = 512 * dwSector;

  for ( i = 0; i<length; i++)
  {
    if (address < BOOT_SECT_SIZE)
    {
      switch (address)
      {
        case 19:
          /* Number of sectors - byte 1 */
        data = (uint8_t)(MSC_BlockCount & 0xFF);
        break;

        case 20:
          /* Number of sectors - byte 2 */
        data = (uint8_t)((MSC_BlockCount >> 8) & 0xFF);
        break;

        case 510:
          /* Validity check - byte 1 */
        data = 0x55;
        break;

        case 511:
          /* Validity check - byte 2 */
        data = 0xAA;
        break;

        default:
        if ( address > 29 )
        {
            data = 0x0;
        }
        else
        {
            /* From 0 -- 29 */
          data = BootSect[address];
        }
        break;
      }
    }
    else if (address < (BOOT_SECT_SIZE + FAT_SIZE + ROOT_DIR_SIZE))
    {
      data = Fat_RootDir[address - BOOT_SECT_SIZE];
    }
    else
    {
      data = *(firmware + (address - offset_address));
    }

    pbBuf[i] = data;
    address++;
  }
  return 0;
}
