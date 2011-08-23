//-----------------------------------------------------------------------------
// Software that is described herein is for illustrative purposes only
// which provides customers with programming information regarding the
// products. This software is supplied "AS IS" without any warranties.
// NXP Semiconductors assumes no responsibility or liability for the
// use of the software, conveys no license or title under any patent,
// copyright, or mask work right to the product. NXP Semiconductors
// reserves the right to make changes in the software without
// notification. NXP Semiconductors also make no representation or
// warranty that such application will be suitable for the specified
// use without further testing or modification.
//-----------------------------------------------------------------------------

#include "sbl_iap.h"
#include "sbl_config.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "LPC17xx.h"
#include "debug.h"
#include "type.h"

// If COMPUTE_BINARY_CHECKSUM is defined, then code will check that checksum
// contained within binary image is valid.
//#define COMPUTE_BINARY_CHECKSUM

const unsigned sector_start_map[MAX_FLASH_SECTOR] = {SECTOR_0_START,             \
SECTOR_1_START,SECTOR_2_START,SECTOR_3_START,SECTOR_4_START,SECTOR_5_START,      \
SECTOR_6_START,SECTOR_7_START,SECTOR_8_START,SECTOR_9_START,SECTOR_10_START,     \
SECTOR_11_START,SECTOR_12_START,SECTOR_13_START,SECTOR_14_START,SECTOR_15_START, \
SECTOR_16_START,SECTOR_17_START,SECTOR_18_START,SECTOR_19_START,SECTOR_20_START, \
SECTOR_21_START,SECTOR_22_START,SECTOR_23_START,SECTOR_24_START,SECTOR_25_START, \
SECTOR_26_START,SECTOR_27_START,SECTOR_28_START,SECTOR_29_START};

const unsigned sector_end_map[MAX_FLASH_SECTOR] = {SECTOR_0_END,SECTOR_1_END,    \
SECTOR_2_END,SECTOR_3_END,SECTOR_4_END,SECTOR_5_END,SECTOR_6_END,SECTOR_7_END,   \
SECTOR_8_END,SECTOR_9_END,SECTOR_10_END,SECTOR_11_END,SECTOR_12_END,             \
SECTOR_13_END,SECTOR_14_END,SECTOR_15_END,SECTOR_16_END,SECTOR_17_END,           \
SECTOR_18_END,SECTOR_19_END,SECTOR_20_END,SECTOR_21_END,SECTOR_22_END,           \
SECTOR_23_END,SECTOR_24_END,SECTOR_25_END,SECTOR_26_END,                         \
SECTOR_27_END,SECTOR_28_END,SECTOR_29_END};

static unsigned param_table[5];
static unsigned result_table[5];

char flash_buf[FLASH_BUF_SIZE];

static unsigned byte_ctr = 0;

void write_data(unsigned cclk,unsigned dst,void * flash_data_buf, unsigned count);
void find_erase_prepare_sector(unsigned cclk, unsigned dst);
void erase_sector(unsigned start_sector,unsigned end_sector,unsigned cclk);
void prepare_sector(unsigned start_sector,unsigned end_sector,unsigned cclk);
void iap_entry(unsigned param_tab[],unsigned result_tab[]);
void compare_data(unsigned cclk, unsigned dst, void * flash_data_buf, unsigned count);

unsigned write_flash(unsigned * dst, char * src, unsigned no_of_bytes)
{
  unsigned i;

  for(i = 0;i<no_of_bytes;i++)
  {
    flash_buf[(byte_ctr+i)] = *(src+i);
  }
  byte_ctr = byte_ctr + no_of_bytes;

  if(byte_ctr == FLASH_BUF_SIZE)
  {
    /* We have accumulated enough bytes to trigger a flash write */
    find_erase_prepare_sector(SystemCoreClock/1000, (unsigned)dst);
    if(result_table[0] != CMD_SUCCESS)
    {
      DBG("Error: prepare sectors\n");
      while(1); /* No way to recover. Just let OS report a write failure */
    }
    write_data( SystemCoreClock/1000,
                (unsigned)dst,
                (void *)flash_buf,
                FLASH_BUF_SIZE);
    if(result_table[0] != CMD_SUCCESS)
    {
      DBG("Error: writing data\n");
      while(1); /* No way to recover. Just let OS report a write failure */
    }

    compare_data( SystemCoreClock/1000,
                (unsigned)dst,
                (void *)flash_buf,
                FLASH_BUF_SIZE);
    if(result_table[0] != CMD_SUCCESS)
    {
      DBG("Error: verifying data\n");
      while(1); /* No way to recover. Just let OS report a write failure */
    }

    /* Reset byte counter and flash address */
    byte_ctr = 0;
    dst = 0;
  }
  return(CMD_SUCCESS);
}

void find_erase_prepare_sector(unsigned cclk, unsigned dst)
{
  unsigned i;

  __disable_irq();
  for(i = USER_START_SECTOR; i <= MAX_USER_SECTOR; i++)
  {
    if(dst < sector_end_map[i])
    {
      if(dst == sector_start_map[i])
      {
        prepare_sector(i, i, cclk);
        erase_sector(i, i, cclk);
      }
      prepare_sector(i , i, cclk);
      break;
    }
  }
  __enable_irq();
}

void write_data(unsigned cclk, unsigned dst, void * flash_data_buf, unsigned count)
{
  __disable_irq();
  param_table[0] = COPY_RAM_TO_FLASH;
  param_table[1] = dst;
  param_table[2] = (unsigned)flash_data_buf;
  param_table[3] = count;
  param_table[4] = cclk;

  // for debug, print the address and number of bytes
  DBG("Writting address: %d -- ", param_table[1]);
  DBG("Nr bytes: %d\n", param_table[3]);

  iap_entry(param_table,result_table);
  __enable_irq();
}

void compare_data(unsigned cclk, unsigned dst, void * flash_data_buf, unsigned count)
{
  __disable_irq();
  param_table[0] = COMPARE;
  param_table[1] = dst;
  param_table[2] = (unsigned)flash_data_buf;
  param_table[3] = count;
  param_table[4] = cclk;
  iap_entry(param_table,result_table);
  __enable_irq();
}

void erase_sector(unsigned start_sector, unsigned end_sector, unsigned cclk)
{
  param_table[0] = ERASE_SECTOR;
  param_table[1] = start_sector;
  param_table[2] = end_sector;
  param_table[3] = cclk;
  iap_entry(param_table,result_table);
}

void prepare_sector(unsigned start_sector,unsigned end_sector,unsigned cclk)
{
  param_table[0] = PREPARE_SECTOR_FOR_WRITE;
  param_table[1] = start_sector;
  param_table[2] = end_sector;
  param_table[3] = cclk;
  iap_entry(param_table,result_table);
}

void iap_entry(unsigned param_tab[], unsigned result_tab[])
{
  void (*iap)(unsigned [], unsigned []);

  iap = (void (*)(unsigned [], unsigned []))IAP_ADDRESS;
  iap(param_tab,result_tab);
}


void execute_user_code(void)
{
  void (*user_code_entry)(void);

  unsigned *p;  // used for loading address of reset handler from user flash

  /* Change the Vector Table to the USER_FLASH_START
  in case the user application uses interrupts */
  SCB->VTOR = (USER_FLASH_START & 0x1FFFFF80);

  /* SP (stack pointer) register must be set correctly before jump to the
   *  user application: http://www.keil.com/forum/17342/
   */
  __set_PSP(USER_FLASH_START);

  // Load contents of second word of user flash - the reset handler address
  // in the applications vector table
  p = (unsigned *)(USER_FLASH_START +4);

  // Set user_code_entry to be the address contained in that second word
  // of user flash
  user_code_entry = (void *) *p;

  // Jump to user application
  user_code_entry();
}

int user_code_present(void)
{
  param_table[0] = BLANK_CHECK_SECTOR;
  param_table[1] = USER_START_SECTOR;
  param_table[2] = USER_START_SECTOR;
  iap_entry(param_table,result_table);
  if( result_table[0] == CMD_SUCCESS )
    return (FALSE);

#ifdef COMPUTE_BINARY_CHECKSUM
	unsigned *pmem, checksum,i;
/*
 * The reserved Cortex-M3 exception vector location 7 (offset 0x001C
 * in the vector table) should contain the 2’s complement of the
 * checksum of table entries 0 through 6. This causes the checksum
 * of the first 8 table entries to be 0. This code checksums the
 * first 8 locations of the start of user flash. If the result is 0,
 * then the contents is deemed a 'valid' image.
 */
  checksum = 0;
  pmem = (unsigned *)USER_FLASH_START;
  for (i = 0; i <= 7; i++)
  {
    checksum += *pmem;
    pmem++;
  }
  if (checksum != 0)
  {
    return (FALSE);
  }
  else
#endif

  {
    return (TRUE);
  }
}

void check_isp_entry_pin (void)
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
  {
    execute_user_code();
  }
}

void erase_user_flash(void)
{
  prepare_sector(USER_START_SECTOR,MAX_USER_SECTOR,SystemCoreClock/1000);
  erase_sector(USER_START_SECTOR,MAX_USER_SECTOR,SystemCoreClock/1000);
  if(result_table[0] != CMD_SUCCESS)
  {
    while(1); /* No way to recover. Just let OS report a write failure */
  }
}
