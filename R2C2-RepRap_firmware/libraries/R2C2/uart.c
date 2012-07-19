/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

#include "stdbool.h"

#include "uart.h"


// for LPC1758
#define MAX_UART 4

static LPC_UART_TypeDef *uart_def_p [MAX_UART] = {
  LPC_UART0,
  (LPC_UART_TypeDef *)LPC_UART1,  //TODO: is UART1 actually different?
  LPC_UART2,
  LPC_UART3};

// ---------------------------------------------------------------------------
// Private generic helper functions
// ---------------------------------------------------------------------------

static LPC_UART_TypeDef *_get_uart_def (int uart_num)
{
  if ((uart_num >= 0) && (uart_num < MAX_UART))
    return uart_def_p [uart_num];
  else
    return NULL;
}

// uart_num can be 0 to MAX_UART-1
static bool _uart_init (int uart_num)
{
  LPC_UART_TypeDef *pUart;

  pUart = _get_uart_def (uart_num);
  if (pUart == NULL)
    return false;

	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	// Pin configuration for UART
	PINSEL_CFG_Type PinCfg;

	/*
	* Initialize UART pin connect
  * default setup                         for R2C2 usage:
	* UART0: P0.2  -> TXD0, P0.3  -> RXD0   not available, used for ADCs
	* UART1: P2.0  -> TXD1; P2.1  -> RXD1   available on expansion header
	* UART2: P0.10 -> TXD2; P0.11 -> RXD2   not available
	* UART3: P4.28 -> TXD3; P4.29 -> RXD3   debug port, RXD is shared with boot switch
  
	*/
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
  switch (uart_num)
  {
    case 0:
    {
      PinCfg.Funcnum = PINSEL_FUNC_1;
      PinCfg.Portnum = 0;
      PinCfg.Pinnum = 2;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Portnum = 0;
      PinCfg.Pinnum = 3;
      PINSEL_ConfigPin(&PinCfg);
    }
    break;

    case 1:
    {
      PinCfg.Funcnum = PINSEL_FUNC_2;
      PinCfg.Portnum = 2;
      PinCfg.Pinnum = 0;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Portnum = 2;
      PinCfg.Pinnum = 1;
      PINSEL_ConfigPin(&PinCfg);
    }
    break;

    case 2:
    {
      PinCfg.Funcnum = PINSEL_FUNC_1;
      PinCfg.Portnum = 0;
      PinCfg.Pinnum = 10;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Portnum = 0;
      PinCfg.Pinnum = 11;
      PINSEL_ConfigPin(&PinCfg);
    }
    break;

    case 3:
    {
      PinCfg.Funcnum = PINSEL_FUNC_3;
      PinCfg.Portnum = 4;
      PinCfg.Pinnum = 28;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Portnum = 4;
      PinCfg.Pinnum = 29;
      PINSEL_ConfigPin(&PinCfg);
    }
    break;

    default:
      return false;
      break;
  }

	/* Initialize UART Configuration parameter structure to default state:
		* Baudrate = as below
		* 8 data bit
		* 1 Stop bit
		* None parity
		*/
	UART_ConfigStructInit(&UARTConfigStruct);
	UARTConfigStruct.Baud_rate = 57600;

	// Initialize UART peripheral with given to corresponding parameter
	UART_Init(pUart, &UARTConfigStruct);

	/* Initialize FIFOConfigStruct to default state:
	*                              - FIFO_DMAMode = DISABLE
	*                              - FIFO_Level = UART_FIFO_TRGLEV0
	*                              - FIFO_ResetRxBuf = ENABLE
	*                              - FIFO_ResetTxBuf = ENABLE
	*                              - FIFO_State = ENABLE
	*/
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	// Initialize FIFO for UART peripheral
	UART_FIFOConfig(pUart, &UARTFIFOConfigStruct);

	// Enable UART Transmit
	UART_TxCmd(pUart, ENABLE);

  return true;
}

static int _uart_data_available(int uart_num)
{
  LPC_UART_TypeDef *pUart;

  pUart = _get_uart_def (uart_num);
  if (pUart == NULL)
    return 0;

	if (pUart->LSR & UART_LSR_RDR)
	  return 1;
	else
	  return 0;
}


static void _uart_send(int uart_num, char byte)
{
  LPC_UART_TypeDef *pUart;

  pUart = _get_uart_def (uart_num);
  if (pUart == NULL)
    return;

	while ( (pUart->LSR & UART_LSR_THRE) == 0) ;
	UART_SendByte(pUart, byte);
}

static char _uart_receive (int uart_num)
{
  LPC_UART_TypeDef *pUart;

  pUart = _get_uart_def (uart_num);
  if (pUart == NULL)
    return 0;
	
  return UART_ReceiveByte(pUart);
}

static void _uart_writestr(int uart_num, char *data)
{
	uint8_t i = 0;
	char r;
  LPC_UART_TypeDef *pUart;

  pUart = _get_uart_def (uart_num);
  if (pUart == NULL)
    return;

 	while ((r = data[i++]))
  {
		// uart_send(r);
    while ( (pUart->LSR & UART_LSR_THRE) == 0) ;
    UART_SendByte(pUart, r);
  }
}

// ---------------------------------------------------------------------------
// Public functions
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Public functions, UART n
// ---------------------------------------------------------------------------

void uart_init (int uart_num)
{
  _uart_init (uart_num);
}

int  uart_data_available (int uart_num)
{	
  return _uart_data_available (uart_num);
}

char uart_receive (int uart_num)
{
  return _uart_receive (uart_num);
}

void uart_send (int uart_num, char byte)
{
  _uart_send (uart_num, byte);
}

void uart_writestr(int uart_num, char *data)
{
  _uart_writestr (uart_num, data);
}

// ---------------------------------------------------------------------------
// Public functions, UART0
// ---------------------------------------------------------------------------

void uart0_init(void)
{
  _uart_init (0);
}

int  uart0_data_available(void)
{	
  return _uart_data_available (0);
}

char uart0_receive(void)
{
	return _uart_receive (0);
}

void uart0_send(char byte)
{
  _uart_send (0, byte);
}

void uart0_writestr(char *data)
{
  _uart_writestr (0, data);
}

// ---------------------------------------------------------------------------
// Public functions, UART1
// ---------------------------------------------------------------------------

void uart1_init(void)
{
  _uart_init (1);
}

int  uart1_data_available(void)
{	
  return _uart_data_available (1);
}

char uart1_receive(void)
{
  return _uart_receive (1);
}

void uart1_send(char byte)
{
  _uart_send (1, byte);
}

// ---------------------------------------------------------------------------
// Public functions, UART2
// ---------------------------------------------------------------------------

void uart2_init(void)
{
  uart_init (2);
}

int  uart2_data_available(void)
{	
  return uart_data_available (2);
}

char uart2_receive(void)
{
  return uart_receive (2);
}

void uart2_send(char byte)
{
  uart_send (2, byte);
}

// ---------------------------------------------------------------------------
// Public functions, UART3
// ---------------------------------------------------------------------------

void uart3_init(void)
{
  uart_init (3);
}

int  uart3_data_available(void)
{	
  return uart_data_available (3);
}

char uart3_receive(void)
{
  return uart_receive (3);
}

void uart3_send(char byte)
{
  _uart_send (3, byte);
}

void uart3_writestr(char *data)
{
  _uart_writestr (3, data);
}



