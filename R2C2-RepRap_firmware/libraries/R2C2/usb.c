/*
        LPCUSB, an USB device driver for LPC microcontrollers
        Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)
        Copyright (c) 2011 Jorge Pinto - casainho@gmail.com

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

#include "LPC17xx.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include <stdio.h>
#include <string.h>         // memcpy
#include "usbapi.h"
#include "usbdebug.h"
#include "serial_fifo.h"

#define BAUD_RATE   115200

#define INT_IN_EP       0x81
#define BULK_OUT_EP     0x05
#define BULK_IN_EP      0x82

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)      ((x)&0xFF),((x)>>8)

// CDC definitions
#define CS_INTERFACE            0x24
#define CS_ENDPOINT             0x25

#define SET_LINE_CODING         0x20
#define GET_LINE_CODING         0x21
#define SET_CONTROL_LINE_STATE  0x22

// data structure for GET_LINE_CODING / SET_LINE_CODING class requests
typedef struct {
    U32     dwDTERate;
    U8      bCharFormat;
    U8      bParityType;
    U8      bDataBits;
} TLineCoding;

fifo_t txfifo;
fifo_t rxfifo;

static unsigned char txbuf[SERIAL_FIFO_SIZE];
static unsigned char rxbuf[SERIAL_FIFO_SIZE];

static TLineCoding LineCoding = {115200, 0, 0, 8};
static U8 abBulkBuf[64];
static U8 abClassReqData[8];

// forward declaration of interrupt handler
void USBIntHandler(void);

static const U8 abDescriptors[] = {

// device descriptor
  0x12,
  DESC_DEVICE,
  LE_WORD(0x0101),            // bcdUSB
  0x02,                       // bDeviceClass
  0x00,                       // bDeviceSubClass
  0x00,                       // bDeviceProtocol
  MAX_PACKET_SIZE0,           // bMaxPacketSize
  LE_WORD(0xFFFF),            // idVendor
  LE_WORD(334),               // idProduct
  LE_WORD(0x0100),            // bcdDevice
  0x01,                       // iManufacturer
  0x02,                       // iProduct
  0x03,                       // iSerialNumber
  0x01,                       // bNumConfigurations

// configuration descriptor
  0x09,
  DESC_CONFIGURATION,
  LE_WORD(147),               // wTotalLength
  0x02,                       // bNumInterfaces
  0x01,                       // bConfigurationValue
  0x00,                       // iConfiguration
  0xC0,                       // bmAttributes
  0x32,                       // bMaxPower
// control class interface
  0x09,
  DESC_INTERFACE,
  0x00,                       // bInterfaceNumber
  0x00,                       // bAlternateSetting
  0x01,                       // bNumEndPoints
  0x02,                       // bInterfaceClass
  0x02,                       // bInterfaceSubClass
  0x01,                       // bInterfaceProtocol, linux requires value of 1 for the cdc_acm module
  0x00,                       // iInterface
// header functional descriptor
  0x05,
  CS_INTERFACE,
  0x00,
  LE_WORD(0x0110),
// call management functional descriptor
  0x05,
  CS_INTERFACE,
  0x01,
  0x01,                       // bmCapabilities = device handles call management
  0x01,                       // bDataInterface
// ACM functional descriptor
  0x04,
  CS_INTERFACE,
  0x02,
  0x02,                       // bmCapabilities
// union functional descriptor
  0x05,
  CS_INTERFACE,
  0x06,
  0x00,                       // bMasterInterface
  0x01,                       // bSlaveInterface0
// notification EP
  0x07,
  DESC_ENDPOINT,
  INT_IN_EP,                  // bEndpointAddress
  0x03,                       // bmAttributes = intr
  LE_WORD(8),                 // wMaxPacketSize
  0x0A,                       // bInterval
// data class interface descriptor
  0x09,
  DESC_INTERFACE,
  0x01,                       // bInterfaceNumber
  0x00,                       // bAlternateSetting
  0x02,                       // bNumEndPoints
  0x0A,                       // bInterfaceClass = data
  0x00,                       // bInterfaceSubClass
  0x00,                       // bInterfaceProtocol
  0x00,                       // iInterface
// data EP OUT
  0x07,
  DESC_ENDPOINT,
  BULK_OUT_EP,                // bEndpointAddress
  0x02,                       // bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),   // wMaxPacketSize
  0x00,                       // bInterval
// data EP in
  0x07,
  DESC_ENDPOINT,
  BULK_IN_EP,                 // bEndpointAddress
  0x02,                       // bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),   // wMaxPacketSize
  0x00,                       // bInterval

// string descriptors
  0x04,
  DESC_STRING,
  LE_WORD(0x0409),

  14,
  DESC_STRING,
  'b', 0, 'i', 0, 't', 0, 'B', 0, 'O', 0, 'X', 0,

  58,
  DESC_STRING,
  'R', 0, '2', 0, 'C', 0, '2', 0, ' ', 0, '-', 0, ' ', 0, //14
  '3', 0, 'D', 0, ' ', 0, 'P', 0, 'r', 0, 'i', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0,'t', 0,'r', 0,'o', 0,'l', 0,'l', 0,'e', 0,'r', 0, //42

  4,
  DESC_STRING,
  '1', 0,

// terminating zero
  0
};


/**
    Local function to handle incoming bulk data

    @param [in] bEP
    @param [in] bEPStatus
 */
static void BulkOut(U8 bEP, U8 bEPStatus)
{
  int i, iLen;

  if (_fifo_free(&rxfifo) < MAX_PACKET_SIZE)
  {
    // may not fit into fifo
    return;
  }

  // get data from USB into intermediate buffer
  iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
  for (i = 0; i < iLen; i++)
  {
    // put into FIFO
    if (!_fifo_put(&rxfifo, abBulkBuf[i]))
    {
      // overflow... :(
      ASSERT(FALSE);
      break;
    }
  }
}


/**
    Local function to handle outgoing bulk data

    @param [in] bEP
    @param [in] bEPStatus
 */
static void BulkIn(U8 bEP, U8 bEPStatus)
{
  int i, iLen;

  if (_fifo_avail(&txfifo) == 0)
  {
    // no more data, disable further NAK interrupts until next USB frame
    USBHwNakIntEnable(0);
    return;
  }

  // get bytes from transmit FIFO into intermediate buffer
  for (i = 0; i < MAX_PACKET_SIZE; i++)
  {
    if (!_fifo_get(&txfifo, &abBulkBuf[i]))
    {
      break;
    }
  }
  iLen = i;

  // send over USB
  if (iLen > 0)
  {
    USBHwEPWrite(bEP, abBulkBuf, iLen);
  }
}


/**
    Local function to handle the USB-CDC class requests

    @param [in] pSetup
    @param [out] piLen
    @param [out] ppbData
 */
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
  switch (pSetup->bRequest)
  {
    // set line coding
    case SET_LINE_CODING:
    memcpy((U8 *)&LineCoding, *ppbData, 7);
    *piLen = 7;
    break;

    // get line coding
    case GET_LINE_CODING:
    *ppbData = (U8 *)&LineCoding;
    *piLen = 7;
    break;

    // set control line state
    case SET_CONTROL_LINE_STATE:
    // bit0 = DTR, bit = RTS
    break;

    default:
    return FALSE;
  }
  return TRUE;
}

static void USBFrameHandler(U16 wFrame)
{
  if (_fifo_avail(&txfifo) > 0)
  {
    // data available, enable NAK interrupt on bulk in
    USBHwNakIntEnable(INACK_BI);
  }
}

/**
    Interrupt handler

    Simply calls the USB ISR
 */
//void USBIntHandler(void)
void USB_IRQHandler(void)
{
  USBHwISR();
}

void enable_USB_interrupts(void);

void USBSerial_Init(void)
{
  // initialise stack
  USBInit();

  // register descriptors
  USBRegisterDescriptors(abDescriptors);

  // register class request handler
  USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

  // register endpoint handlers
  USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
  USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
  USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);

  // register frame handler
  USBHwRegisterFrameHandler(USBFrameHandler);

  // enable bulk-in interrupts on NAKs
  USBHwNakIntEnable(INACK_BI);

  // initialise VCOM
  fifo_init(&rxfifo, rxbuf);
  fifo_init(&txfifo, txbuf);

  NVIC_EnableIRQ(USB_IRQn);

  // connect to bus
  USBHwConnect(TRUE);
}
