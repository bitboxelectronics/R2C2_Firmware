/*
	LPCUSB, an USB device driver for LPC microcontrollers
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in the
	   documentation and/or other materials provided with the distribution.
	3. The name of the author may not be used to endorse or promote products
	   derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*********************************************************************
 * Code Red Technologies - modification of LPCUSB mass storage example
 * to provide a flash-based USB secondary bootloader for RDB1768
 * development board fitted with NXP LPC1768 MCU.
 *********************************************************************/

#include <stdint.h>
#include "usbapi.h"
#include "usbdebug.h"
#include "msc_bot.h"

/*
 * The LPCUSB stack can be configured to print debug messages - for
 * which the Code Red port uses semihosted printf's. However doing
 * this is not compatible with the use of the bootloader. Thus we
 * check whether the symbol controlling the use of debug messages
 * in the LPCUSB file usbdebug.h is defined. If it is, then LPCUSB
 * is configured to use semihosted printf's, and hence we give
 * an error here.
 */

#define BAUD_RATE	115200

#define MAX_PACKET_SIZE	64

#define LE_WORD(x)		((x)&0xFF),((x)>>8)


static U8 abClassReqData[4];

static const U8 abDescriptors[] = {

// device descriptor
	0x12,
	DESC_DEVICE,
	LE_WORD(0x0200),		// bcdUSB
	0x00,					// bDeviceClass
	0x00,					// bDeviceSubClass
	0x00,					// bDeviceProtocol
	MAX_PACKET_SIZE0,		// bMaxPacketSize
	LE_WORD(0xFFFF),		// idVendor
	LE_WORD(333),		        // idProduct
	LE_WORD(0x0100),		// bcdDevice
	0x01,					// iManufacturer
	0x02,					// iProduct
	0x03,					// iSerialNumber
	0x01,					// bNumConfigurations

// configuration descriptor
	0x09,
	DESC_CONFIGURATION,
	LE_WORD(102),			        // wTotalLength
	0x01,					// bNumInterfaces
	0x01,					// bConfigurationValue
	0x00,					// iConfiguration
	0xC0,					// bmAttributes
	0x32,					// bMaxPower

// interface
	0x09,
	DESC_INTERFACE,
	0x00,					// bInterfaceNumber
	0x00,					// bAlternateSetting
	0x02,					// bNumEndPoints
	0x08,					// bInterfaceClass = mass storage
	0x06,					// bInterfaceSubClass = transparent SCSI
	0x50,					// bInterfaceProtocol = BOT
	0x00,					// iInterface
// EP
	0x07,
	DESC_ENDPOINT,
	MSC_BULK_IN_EP,			// bEndpointAddress
	0x02,					// bmAttributes = bulk
	LE_WORD(MAX_PACKET_SIZE),// wMaxPacketSize
	0x00,					// bInterval
// EP
	0x07,
	DESC_ENDPOINT,
	MSC_BULK_OUT_EP,		// bEndpointAddress
	0x02,					// bmAttributes = bulk
	LE_WORD(MAX_PACKET_SIZE),// wMaxPacketSize
	0x00,					// bInterval

// string descriptors
	0x04,
	DESC_STRING,
	LE_WORD(0x0409),

	14,
	DESC_STRING,
	'b', 0, 'i', 0, 't', 0, 'B', 0, 'O', 0, 'X', 0,

	40,
	DESC_STRING,
	'R', 0, '2', 0, 'C', 0, '2', 0, ' ', 0, 'U', 0, 'S', 0, 'B', 0,' ',0,'B',0,'o',0,'o',0,'t',0,'l',0,'o',0,'a',0,'d',0,'e',0,'r', 0,

	12,
	DESC_STRING,
	'2', 0, '.', 0, '1', 0, '.', 0, '1', 0,/* bootloader version 2.1.1 */

// terminating zero
	0
};


/*************************************************************************
	HandleClassRequest
	==================
		Handle mass storage class request

**************************************************************************/
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
	if (pSetup->wIndex != 0) {
		DBG("Invalid idx %X\n", pSetup->wIndex);
		return FALSE;
	}
	if (pSetup->wValue != 0) {
		DBG("Invalid val %X\n", pSetup->wValue);
		return FALSE;
	}

	switch (pSetup->bRequest) {

	// get max LUN
	case 0xFE:
		*ppbData[0] = 0;		// No LUNs
		*piLen = 1;
		break;

	// MSC reset
	case 0xFF:
		if (pSetup->wLength > 0) {
			return FALSE;
		}
		MSCBotReset();
		break;

	default:
		DBG("Unhandled class\n");
		return FALSE;
	}
	return TRUE;
}

void init_usb_msc_device(void)
{
	DBG("Initialising USB stack\n");

	// initialise stack
	USBInit();

	// enable bulk-in interrupts on NAKs
	// these are required to get the BOT protocol going again after a STALL
	USBHwNakIntEnable(INACK_BI);

	// register descriptors
	USBRegisterDescriptors(abDescriptors);

	// register class request handler
	USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

	// register endpoint handlers
	USBHwRegisterEPIntHandler(MSC_BULK_IN_EP, MSCBotBulkIn);
	USBHwRegisterEPIntHandler(MSC_BULK_OUT_EP, MSCBotBulkOut);

	DBG("Starting USB communication\n");

	// connect to bus
	USBHwConnect(TRUE);
}

void usb_msc_start (void)
{
	init_usb_msc_device();

  // call USB interrupt handler continuously
  while (1) {
    USBHwISR();
  }
}

