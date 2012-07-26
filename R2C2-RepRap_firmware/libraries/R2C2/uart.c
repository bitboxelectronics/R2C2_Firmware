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

#include "str_buffer.h"

#include "uart.h"



typedef struct {

  LPC_UART_TypeDef *uart_def_p;

  // Current Tx Interrupt enable state
  __IO FlagStatus TxIntStat;

  tStrBuffer rx_buffer;
  tStrBuffer tx_buffer;

} tUartControl;


// --------------------------------------------------------------------------
// CONFIG here

#define USE_INTERRUPT

#define CFG_MAX_UART  4

// LPC1758 has max of 4, we use UART1 and UART3
// don't allocate buffers for UARTs we don't use

//tUartControl uart0 = {.uart_def_p = LPC_UART0};
tUartControl uart1 = {.uart_def_p = (LPC_UART_TypeDef *)LPC_UART1};
//tUartControl uart2 = {.uart_def_p = LPC_UART2};
tUartControl uart3 = {.uart_def_p = LPC_UART3};

static tUartControl *uart_control [CFG_MAX_UART] = 
{
  NULL,
  &uart1,
  NULL,
  &uart3
};
// --------------------------------------------------------------------------

  
//  {.uart_def_p = (LPC_UART_TypeDef *)LPC_UART1},  //TODO: UART1 features
  

// ---------------------------------------------------------------------------
// Private generic helper functions
// ---------------------------------------------------------------------------

static tUartControl *_get_uart_control (int uart_num)
{
  if ((uart_num >= 0) && (uart_num < CFG_MAX_UART))
  {
    return uart_control [uart_num];
  }
  else
    return NULL;
}

static LPC_UART_TypeDef *_get_uart_def (int uart_num)
{
  if ((uart_num >= 0) && (uart_num < CFG_MAX_UART))
  {
    if (uart_control [uart_num] == NULL)
      return NULL;
    else
      return uart_control [uart_num]->uart_def_p;
  }
  else
    return NULL;
}

extern Status uart_set_divisors(LPC_UART_TypeDef *UARTx, uint32_t baudrate);

static void configure_uart (LPC_UART_TypeDef *UARTx, UART_CFG_Type *UART_ConfigStruct)
{
	uint32_t tmp;

	// Set Line Control register ----------------------------

	uart_set_divisors(UARTx, (UART_ConfigStruct->Baud_rate));

	if (((LPC_UART1_TypeDef *)UARTx) == LPC_UART1)
	{
		tmp = (((LPC_UART1_TypeDef *)UARTx)->LCR & (UART_LCR_DLAB_EN | UART_LCR_BREAK_EN)) \
				& UART_LCR_BITMASK;
	}
	else
	{
		tmp = (UARTx->LCR & (UART_LCR_DLAB_EN | UART_LCR_BREAK_EN)) & UART_LCR_BITMASK;
	}

	switch (UART_ConfigStruct->Databits){
	case UART_DATABIT_5:
		tmp |= UART_LCR_WLEN5;
		break;
	case UART_DATABIT_6:
		tmp |= UART_LCR_WLEN6;
		break;
	case UART_DATABIT_7:
		tmp |= UART_LCR_WLEN7;
		break;
	case UART_DATABIT_8:
	default:
		tmp |= UART_LCR_WLEN8;
		break;
	}

	if (UART_ConfigStruct->Parity == UART_PARITY_NONE)
	{
		// Do nothing...
	}
	else
	{
		tmp |= UART_LCR_PARITY_EN;
		switch (UART_ConfigStruct->Parity)
		{
		case UART_PARITY_ODD:
			tmp |= UART_LCR_PARITY_ODD;
			break;

		case UART_PARITY_EVEN:
			tmp |= UART_LCR_PARITY_EVEN;
			break;

		case UART_PARITY_SP_1:
			tmp |= UART_LCR_PARITY_F_1;
			break;

		case UART_PARITY_SP_0:
			tmp |= UART_LCR_PARITY_F_0;
			break;
		default:
			break;
		}
	}

	switch (UART_ConfigStruct->Stopbits){
	case UART_STOPBIT_2:
		tmp |= UART_LCR_STOPBIT_SEL;
		break;
	case UART_STOPBIT_1:
	default:
		// Do no thing
		break;
	}


	// Write back to LCR, configure FIFO and Disable Tx
	if (((LPC_UART1_TypeDef *)UARTx) ==  LPC_UART1)
	{
		((LPC_UART1_TypeDef *)UARTx)->LCR = (uint8_t)(tmp & UART_LCR_BITMASK);
	}
	else
	{
		UARTx->LCR = (uint8_t)(tmp & UART_LCR_BITMASK);
	}
}

/********************************************************************//**
 * @brief 		UART transmit function (ring buffer used)
 *********************************************************************/
void UART_IntTransmit (tUartControl *pControl)
{
  // Disable THRE interrupt
  UART_IntConfig(pControl->uart_def_p, UART_INTCFG_THRE, DISABLE);

	/* Wait for FIFO buffer empty, transfer UART_TX_FIFO_SIZE bytes
	 * of data or break whenever ring buffers are empty */
	
  /* Wait until THR empty ?? */
  while (UART_CheckBusy(pControl->uart_def_p) == SET);

	while (!str_buf_is_empty (&pControl->tx_buffer))
  {
#if 0
    /* Move a piece of data into the transmit FIFO */
    if (UART_Send(pControl->uart_def_p, (uint8_t *)&rb.tx[rb.tx_tail], 1, NONE_BLOCKING))
    {
        /* Update transmit ring FIFO tail pointer */
        __BUF_INCR(rb.tx_tail);
    	} else {
    		break;
    	}
#endif
    char c = str_buf_getc (&pControl->tx_buffer);
    UART_SendByte (pControl->uart_def_p, c);

    if ( UART_CheckBusy (pControl->uart_def_p) == SET)
      break;
  }

  /* If there is no more data to send, disable the transmit
       interrupt - else enable it or keep it enabled */
	if (str_buf_is_empty (&pControl->tx_buffer)) 
  {
    UART_IntConfig (pControl->uart_def_p, UART_INTCFG_THRE, DISABLE);
    // Reset Tx Interrupt state
    pControl->TxIntStat = RESET;
  }
  else
  {
    // Set Tx Interrupt state
		pControl->TxIntStat = SET;
    UART_IntConfig (pControl->uart_def_p, UART_INTCFG_THRE, ENABLE);
  }
}

/********************************************************************//**
 * @brief 		UART receive function (ring buffer used)
 *********************************************************************/
void UART_IntReceive(tUartControl *pControl)
{
	uint8_t tmpc;
	uint32_t rLen;

	while(1)
  {
		// Call UART read function in UART driver
		rLen = UART_Receive(pControl->uart_def_p, &tmpc, 1, NONE_BLOCKING);
		// If data received
		if (rLen)
    {
			/* Check if buffer has more space
			 * If no space, data will be discarded
			 */
			if (str_buf_is_full (&pControl->rx_buffer))
      {
        pControl->rx_buffer.header.err_overflow = 1;
      }
      else
      {
        str_buf_putc (&pControl->rx_buffer, tmpc);
			}
		}
		// no more data
		else {
			break;
		}
	}
}



/*********************************************************************//**
 * @brief		generic UART interrupt handler sub-routine
 **********************************************************************/
static void UART_IRQHandler (tUartControl *pControl)
{
  uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId (pControl->uart_def_p);
	tmp = intsrc & UART_IIR_INTID_MASK;

#if 0
  // Receive Line Status
	if (tmp == UART_IIR_INTID_RLS)
  {
		// Check line status
		tmp1 = UART_GetLineStatus(LPC_UART0);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}
#endif

	// Receive Data Available or Character time-out?
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI))
  {
			UART_IntReceive(pControl);
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE)
  {
			UART_IntTransmit(pControl);
	}

}

/*********************************************************************//**
 * @brief		UART0 interrupt handler sub-routine
 **********************************************************************/
void UART0_IRQHandler(void)
{
  UART_IRQHandler (uart_control[0]);
}

void UART1_IRQHandler(void)
{
  UART_IRQHandler (uart_control[1]);
}

void UART2_IRQHandler(void)
{
  UART_IRQHandler (uart_control[2]);
}

void UART3_IRQHandler(void)
{
  UART_IRQHandler (uart_control[3]);
}


// uart_num can be 0 to MAX_UART-1
static bool _uart_init (int uart_num)
{
  LPC_UART_TypeDef *pUart;
  tUartControl *pControl;
	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	// Pin configuration for UART
	PINSEL_CFG_Type PinCfg;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return false;

  pUart = pControl->uart_def_p;

  str_buf_init_std (&pControl->rx_buffer);
  str_buf_init_std (&pControl->tx_buffer);


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

#ifdef USE_INTERRUPT
	// Enable UART Transmit
	UART_TxCmd(pUart, ENABLE);

    /* Enable UART Rx interrupt */
	UART_IntConfig(pUart, UART_INTCFG_RBR, ENABLE);

#if 0
  /* Enable UART line status interrupt */
	UART_IntConfig(pUart, UART_INTCFG_RLS, ENABLE);
#endif
  /*
	 * Do not enable transmit interrupt here, since it is handled by
	 * UART_Send() function, just to reset Tx Interrupt state for the
	 * first time
	 */
	pControl->TxIntStat = RESET;

  switch (uart_num)
  {
    case 0:
      /* preemption = 1, sub-priority = 1 */
      NVIC_SetPriority(UART0_IRQn, ((0x01<<3)|0x01));
      /* Enable Interrupt for UART0 channel */
      NVIC_EnableIRQ(UART0_IRQn);
      break;

    case 1:
      /* preemption = 1, sub-priority = 1 */
      NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
      /* Enable Interrupt for UART1 channel */
      NVIC_EnableIRQ(UART1_IRQn);
      break;

    case 3:
      /* preemption = 1, sub-priority = 1 */
      NVIC_SetPriority(UART3_IRQn, ((0x01<<3)|0x01));
      /* Enable Interrupt for UART3 channel */
      NVIC_EnableIRQ(UART3_IRQn);
      break;
  }
#endif

  return true;
}

bool uart_get_config (int uart_num, tPortSettings *port_settings_p)
{
  //TODO: get the current UART settings

  return false;
}

bool uart_configure(int uart_num, tPortSettings *port_settings_p)
{
  tUartControl *pControl;
  LPC_UART_TypeDef *pUart;
	UART_CFG_Type UARTConfigStruct;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return false;
  pUart = pControl->uart_def_p;

	UART_ConfigStructInit(&UARTConfigStruct);

	UARTConfigStruct.Baud_rate = port_settings_p->baud_rate;
  UARTConfigStruct.Databits = port_settings_p->data_bits;
	UARTConfigStruct.Parity = port_settings_p->parity;
	UARTConfigStruct.Stopbits = port_settings_p->stop_bits;

  configure_uart (pUart, &UARTConfigStruct);

  return true;
}

static int _uart_data_available(int uart_num)
{
  tUartControl *pControl;
  LPC_UART_TypeDef *pUart;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return 0;
  pUart = pControl->uart_def_p;

#ifdef USE_INTERRUPT
  return !str_buf_is_empty (&pControl->rx_buffer);
#else
  if (pUart->LSR & UART_LSR_RDR)
	  return 1;
	else
	  return 0;
#endif
}


static void _uart_send(int uart_num, char byte)
{
  tUartControl *pControl;
  LPC_UART_TypeDef *pUart;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return;

  pUart = pControl->uart_def_p;

#ifdef USE_INTERRUPT
  // put byte into buffer
  str_buf_putc (&pControl->tx_buffer, byte);

  if (pControl->TxIntStat == RESET) 
  {
		UART_IntTransmit(pControl);
	}
  else
  {
    // re-enable Tx Interrupt
  	UART_IntConfig (pUart, UART_INTCFG_THRE, ENABLE);
	}
#else
  
//
	while ( (pUart->LSR & UART_LSR_THRE) == 0) ;
	UART_SendByte(pUart, byte);
#endif
}

static char _uart_receive (int uart_num)
{
  tUartControl *pControl;
  LPC_UART_TypeDef *pUart;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return 0;
  pUart = pControl->uart_def_p;
	
#ifdef USE_INTERRUPT
  if (str_buf_is_empty (&pControl->rx_buffer))
    return 0;
  else
    return str_buf_getc (&pControl->rx_buffer);
#else
  return UART_ReceiveByte(pUart);
#endif
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



