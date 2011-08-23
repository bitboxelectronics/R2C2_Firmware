#include "LPC17xx.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "sbl_iap.h"
#include "sbl_config.h"
#include "type.h"
#include "uart.h"
#include "sersendf.h"
#include "spi.h"
#include "sdcard.h"

/* USB mass storage driver - in msc_usb_start.c */
void usb_msc_start (void);

/*********************************************************************//**
 * @brief	Main sub-routine
 **********************************************************************/
int main(void)
{
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

  uart_init(); // initialize UART for printing debug messages
  spi_init(); // initialize SPI for SDCard
  MMC_disk_initialize(); // initialize SDCard

#if 0
  // Check to see if there is a user application in the LPC1768's flash memory.
  if(user_code_present())
  {
    // There is an application, but need to check if user is pressing the button
    // to indicate they want to upload a new application.
    check_isp_entry_pin();
  }
#endif

  // User code not present or isp entry requested
  usb_msc_start();

  // Note - should never actually return from enter_usb_isp ().
  while (1);		// loop forever
  return 0;
}

void startup_delay(void)
{
  for (volatile unsigned long i = 0; i < 500000; i++) { ; }
}
