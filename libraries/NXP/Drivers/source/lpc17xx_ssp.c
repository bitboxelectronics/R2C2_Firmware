/**
 * @file	: lpc17xx_ssp.c
 * @brief	: Contains all functions support for SSP firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 9. April. 2009
 * @author	: HieuNguyen
 *----------------------------------------------------------------------------
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#include "lpc17xx_ssp.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"


/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */


/************************** PRIVATE VARIABLES *************************/
#ifdef _SSP

#ifdef _SSP0

/** @addtogroup Private_Variables
 * @{
 */

/** @defgroup SSP_Private_Variables
 * @{
 */
const PINSEL_CFG_Type ssp0_sck_pin[SSP0_MAX_SCK_PIN] = { \
		SSP0_PINSEL_SCK_P0_15, \
		SSP0_PINSEL_SCK_P1_20 };
const PINSEL_CFG_Type ssp0_ssel_pin[SSP0_MAX_SSEL_PIN] = { \
		SSP0_PINSEL_SSEL_P0_16, \
		SSP0_PINSEL_SSEL_P1_21 };
const PINSEL_CFG_Type ssp0_miso_pin[SSP0_MAX_MISO_PIN] = { \
		SSP0_PINSEL_MISO_P0_17, \
		SSP0_PINSEL_MISO_P1_23 };
const PINSEL_CFG_Type ssp0_mosi_pin[SSP0_MAX_MOSI_PIN] = { \
		SSP0_PINSEL_MOSI_P0_18, \
		SSP0_PINSEL_MOSI_P1_24 };
#endif /* _SSP0 */

#ifdef _SSP1
const PINSEL_CFG_Type ssp1_sck_pin[SSP1_MAX_SCK_PIN] = { \
		SSP1_PINSEL_SCK_P0_7, \
		SSP1_PINSEL_SCK_P1_31 };
const PINSEL_CFG_Type ssp1_ssel_pin[SSP1_MAX_SSEL_PIN] = { \
		SSP1_PINSEL_SSEL_P0_6 };
const PINSEL_CFG_Type ssp1_miso_pin[SSP1_MAX_MISO_PIN] = { \
		SSP1_PINSEL_MISO_P0_8 };
const PINSEL_CFG_Type ssp1_mosi_pin[SSP1_MAX_MOSI_PIN] = { \
		SSP1_PINSEL_MOSI_P0_9 };
#endif /* _SSP1 */

/**
 * @}
 */

/**
 * @}
 */

/************************** GLOBAL/PUBLIC FUNCTIONS *************************/
/** @addtogroup Public_Functions
  * @{
  */

/** @defgroup SSP_Public_Functions
 * @{
 */
/*********************************************************************//**
 * @brief 		Setup clock rate for SSP device
 * @param[in] 	SSPx	SSP peripheral definition, should be
 * 						SSP0 or SSP1.
 * @param[in]	target_clock : clock of SSP (Hz)
 * @return 		Status of process (ERROR or SUCCESS)
 ***********************************************************************/
Status SSP_SetClock (SSP_TypeDef *SSPx, uint32_t target_clock)
{
    uint32_t prescale, cr0_div, cmp_clk, ssp_clk;

    CHECK_PARAM(PARAM_SSPx(SSPx));

    /* The SSP clock is derived from the (main system oscillator / 2),
       so compute the best divider from that clock */
    if (SSPx == SSP0)
    {
    	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP0);
    }
    else if (SSPx == SSP1)
    {
    	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP1);
    }

	/* Find closest divider to get at or under the target frequency.
	   Use smallest prescale possible and rely on the divider to get
	   the closest target frequency */
	cr0_div = 0;
	cmp_clk = 0xFFFFFFFF;
	prescale = 2;
	while (cmp_clk > target_clock)
	{
		cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
		if (cmp_clk > target_clock)
		{
			cr0_div++;
			if (cr0_div > 0xFF)
			{
				cr0_div = 0;
				prescale += 2;
			}
		}
	}

    /* Write computed prescaler and divider back to register */
    SSPx->CR0 &= (~SSP_CR0_SCR(0xFF)) & SSP_CR0_BITMASK;
    SSPx->CR0 |= (SSP_CR0_SCR(cr0_div)) & SSP_CR0_BITMASK;
    SSPx->CPSR = prescale & SSP_CPSR_BITMASK;

    return SUCCESS;
}



/*********************************************************************//**
 * @brief		Set all pins used as SSPx function corresponding to
 * 				parameter specified in SSPPinCfg.
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	SSPPinCfg	Pointer to a SSP_PinCFG_Type structure
*                    that contains the configuration information for the
*                    specified SSPx pin function.
 * @return 		None
 **********************************************************************/
void SSP_PinConfig(SSP_TypeDef *SSPx, SSP_PinCFG_Type *SSPPinCfg)
{
	PINSEL_CFG_Type default_pincfg;

	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_SSP_MODE(SSPPinCfg->SSPMode));
	CHECK_PARAM(PARAM_SSP_CS_FUNC(SSPPinCfg->CSPinConfig));

#ifdef _SSP0
	if (SSPx == SSP0)
	{
		CHECK_PARAM(PARAM_SSP0_SCK(SSPPinCfg->SCK_Pin));
		CHECK_PARAM(PARAM_SSP0_SSEL(SSPPinCfg->SSEL_Pin));
		CHECK_PARAM(PARAM_SSP0_MISO(SSPPinCfg->MISO_Pin));
		CHECK_PARAM(PARAM_SSP0_MOSI(SSPPinCfg->MOSI_Pin));

		// SCK pin
		PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp0_sck_pin[SSPPinCfg->SCK_Pin]));

		// SSEL pin
		if (SSPPinCfg->SSPMode == SSP_SLAVE_MODE)
		{
			PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp0_ssel_pin[SSPPinCfg->SSEL_Pin]));
		}
		else
		{
			if (SSPPinCfg->CSPinConfig == SSP_CS_GPIO)
			{
				// Release SSEL pin as default function
				default_pincfg.Portnum = ssp0_ssel_pin[SSPPinCfg->SSEL_Pin].Portnum;
				default_pincfg.Pinnum = ssp0_ssel_pin[SSPPinCfg->SSEL_Pin].Pinnum;
				// Reset default state of function pin
				default_pincfg.Funcnum = PINSEL_FUNC_0;
				// pull up resistor
				default_pincfg.Pinmode = PINSEL_PINMODE_PULLUP;
				// Normal
				default_pincfg.OpenDrain = PINSEL_PINMODE_NORMAL;
				// Apply these configuration above to CS pin
				PINSEL_ConfigPin(&default_pincfg);
			}
			else
			{
				PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp0_ssel_pin[SSPPinCfg->SSEL_Pin]));
			}
		}

		// MISO pin
		PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp0_miso_pin[SSPPinCfg->MISO_Pin]));

		// MOSI pin
		PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp0_mosi_pin[SSPPinCfg->MOSI_Pin]));
	}
#endif

#ifdef _SSP1
	if (SSPx == SSP1)
	{
		CHECK_PARAM(PARAM_SSP1_SCK(SSPPinCfg->SCK_Pin));
		CHECK_PARAM(PARAM_SSP1_SSEL(SSPPinCfg->SSEL_Pin));
		CHECK_PARAM(PARAM_SSP1_MISO(SSPPinCfg->MISO_Pin));
		CHECK_PARAM(PARAM_SSP1_MOSI(SSPPinCfg->MOSI_Pin));

		// SCK pin
		PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp1_sck_pin[SSPPinCfg->SCK_Pin]));

		// SSEL pin
		if (SSPPinCfg->SSPMode == SSP_SLAVE_MODE)
		{
			PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp1_ssel_pin[SSPPinCfg->SSEL_Pin]));
		}
		else
		{
			if (SSPPinCfg->CSPinConfig == SSP_CS_GPIO)
			{
				// Release SSEL pin as default function
				default_pincfg.Portnum = ssp1_ssel_pin[SSPPinCfg->SSEL_Pin].Portnum;
				default_pincfg.Pinnum = ssp1_ssel_pin[SSPPinCfg->SSEL_Pin].Pinnum;
				// Reset default state of function pin
				default_pincfg.Funcnum = PINSEL_FUNC_0;
				// pull up resistor
				default_pincfg.Pinmode = PINSEL_PINMODE_PULLUP;
				// Normal
				default_pincfg.OpenDrain = PINSEL_PINMODE_NORMAL;
				PINSEL_ConfigPin(&default_pincfg);
			}
			else
			{
				PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp1_ssel_pin[SSPPinCfg->SSEL_Pin]));
			}
		}

		// MISO pin
		PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp1_miso_pin[SSPPinCfg->MISO_Pin]));

		// MOSI pin
		PINSEL_ConfigPin((PINSEL_CFG_Type *)(&ssp1_mosi_pin[SSPPinCfg->MOSI_Pin]));
	}
#endif
}


/*****************************************************************************//**
 * @brief		Fills each SSP_PinInitStruct member with its default value:
 * 				- SSPMode = SSP_MASTER_MODE
				- CSPinConfig = SSP_CS_DEFAULT
 * 				if selected SSPx peripheral is SSP0:
 * 				- MISO_Pin = SSP0_MISO_P0_17
				- MOSI_Pin = SSP0_MOSI_P0_18
				- SCK_Pin = SSP0_SCK_P0_15
				- SSEL_Pin = SSP0_SSEL_P0_16
				if selected SSPx peripheral is SSP1:
 * 				- MISO_Pin = SSP1_MISO_P0_8
				- MOSI_Pin = SSP1_MOSI_P0_9
				- SCK_Pin = SSP1_SCK_P0_7
				- SSEL_Pin = SSP1_SSEL_P0_6
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	SSP_PinInitStruct Pointer to a SSP_PinCFG_Type structure
 *                    which will be initialized.
 * @return		None
 *******************************************************************************/
void SSP_PinConfigStructInit(SSP_TypeDef *SSPx, SSP_PinCFG_Type *SSP_PinInitStruct)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));

#ifdef _SSP0
	if (SSPx == SSP0)
	{
		SSP_PinInitStruct->MISO_Pin = SSP0_MISO_P0_17;
		SSP_PinInitStruct->MOSI_Pin = SSP0_MOSI_P0_18;
		SSP_PinInitStruct->SCK_Pin = SSP0_SCK_P0_15;
		SSP_PinInitStruct->SSEL_Pin = SSP0_SSEL_P0_16;
		SSP_PinInitStruct->SSPMode = SSP_MASTER_MODE;
		SSP_PinInitStruct->CSPinConfig = SSP_CS_DEFAULT;
	}
#endif

#ifdef _SSP1
	if (SSPx == SSP1)
	{
		SSP_PinInitStruct->MISO_Pin = SSP1_MISO_P0_8;
		SSP_PinInitStruct->MOSI_Pin = SSP1_MOSI_P0_9;
		SSP_PinInitStruct->SCK_Pin = SSP1_SCK_P0_7;
		SSP_PinInitStruct->SSEL_Pin = SSP1_SSEL_P0_6;
		SSP_PinInitStruct->SSPMode = SSP_MASTER_MODE;
		SSP_PinInitStruct->CSPinConfig = SSP_CS_DEFAULT;
	}
#endif
}


/*********************************************************************//**
 * @brief		De-initializes the SSPx peripheral registers to their
*                  default reset values.
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @return 		None
 **********************************************************************/
void SSP_DeInit(SSP_TypeDef* SSPx)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
#ifdef _SSP0
	if (SSPx == SSP0)
	{
		/* Set up clock and power for SSP0 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSSP0, DISABLE);
	}
#endif

#ifdef _SSP1
	if (SSPx == SSP1)
	{
		/* Set up clock and power for SSP1 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSSP1, DISABLE);
	}
#endif
}



/********************************************************************//**
 * @brief		Initializes the SSPx peripheral according to the specified
*               parameters in the SSP_ConfigStruct.
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	SSP_ConfigStruct Pointer to a SSP_CFG_Type structure
*                    that contains the configuration information for the
*                    specified SSP peripheral.
 * @return 		None
 *********************************************************************/
void SSP_Init(SSP_TypeDef *SSPx, SSP_CFG_Type *SSP_ConfigStruct)
{
	SSP_PinCFG_Type defaultSSPPinCfg;
	uint32_t tmp;

	CHECK_PARAM(PARAM_SSPx(SSPx));

#ifdef _SSP0
	if(SSPx == SSP0)
	{
		/* Set up clock and power for SSP0 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSSP0, ENABLE);
		/* As default, peripheral clock for SSP0 module
		 * is set to FCCLK / 2 */
		CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_SSP0, CLKPWR_PCLKSEL_CCLK_DIV_2);
	}
#endif

#ifdef _SSP1
	if(SSPx == SSP1)
	{
		/* Set up clock and power for SSP1 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSSP1, ENABLE);
		/* As default, peripheral clock for SSP1 module
		 * is set to FCCLK / 2 */
		CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_SSP1, CLKPWR_PCLKSEL_CCLK_DIV_2);
	}
#endif

	// Set SSPx function pin as default
	SSP_PinConfigStructInit(SSPx, &defaultSSPPinCfg);
	// Configure mode corresponding
	defaultSSPPinCfg.SSPMode = SSP_ConfigStruct->Mode;
	SSP_PinConfig(SSPx, &defaultSSPPinCfg);

	/* Configure SSP, interrupt is disable, LoopBack mode is disable,
	 * SSP is disable, Slave output is disable as default
	 */
	tmp = ((SSP_ConfigStruct->CPHA) | (SSP_ConfigStruct->CPOL) \
		| (SSP_ConfigStruct->FrameFormat) | (SSP_ConfigStruct->Databit))
		& SSP_CR0_BITMASK;
	// write back to SSP control register
	SSPx->CR0 = tmp;

	tmp = SSP_ConfigStruct->Mode & SSP_CR1_BITMASK;
	// Write back to CR1
	SSPx->CR1 = tmp;

	// Set clock rate for SSP peripheral
	SSP_SetClock(SSPx, SSP_ConfigStruct->ClockRate);
}



/*****************************************************************************//**
* @brief		Fills each SSP_InitStruct member with its default value:
* 				- CPHA = SSP_CPHA_FIRST
* 				- CPOL = SSP_CPOL_HI
* 				- ClockRate = 1000000
* 				- Databit = SSP_DATABIT_8
* 				- Mode = SSP_MASTER_MODE
* 				- FrameFormat = SSP_FRAME_SPI
* @param[in]	SSP_InitStruct Pointer to a SSP_CFG_Type structure
*                    which will be initialized.
* @return		None
*******************************************************************************/
void SSP_ConfigStructInit(SSP_CFG_Type *SSP_InitStruct)
{
	SSP_InitStruct->CPHA = SSP_CPHA_FIRST;
	SSP_InitStruct->CPOL = SSP_CPOL_HI;
	SSP_InitStruct->ClockRate = 1000000;
	SSP_InitStruct->Databit = SSP_DATABIT_8;
	SSP_InitStruct->Mode = SSP_MASTER_MODE;
	SSP_InitStruct->FrameFormat = SSP_FRAME_SPI;
}


/*********************************************************************//**
 * @brief		Enable or disable SSP peripheral's operation
 * @param[in]	SSPx	SSP peripheral, should be SSP0 or SSP1
 * @param[in]	NewState New State of SSPx peripheral's operation
 * @return 		none
 **********************************************************************/
void SSP_Cmd(SSP_TypeDef* SSPx, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		SSPx->CR1 |= SSP_CR1_SSP_EN;
	}
	else
	{
		SSPx->CR1 &= (~SSP_CR1_SSP_EN) & SSP_CR1_BITMASK;
	}
}



/*********************************************************************//**
 * @brief		Enable or disable Loop Back mode function in SSP peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	NewState	New State of Loop Back mode, should be:
 * 							- ENABLE: Enable this function
 * 							- DISABLE: Disable this function
 * @return 		None
 **********************************************************************/
void SSP_LoopBackCmd(SSP_TypeDef* SSPx, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		SSPx->CR1 |= SSP_CR1_LBM_EN;
	}
	else
	{
		SSPx->CR1 &= (~SSP_CR1_LBM_EN) & SSP_CR1_BITMASK;
	}
}



/*********************************************************************//**
 * @brief		Enable or disable Slave Output function in SSP peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	NewState	New State of Slave Output function, should be:
 * 							- ENABLE: Slave Output in normal operation
 * 							- DISABLE: Slave Output is disabled. This blocks
 * 							SSP controller from driving the transmit data
 * 							line (MISO)
 * Note: 		This function is available when SSP peripheral in Slave mode
 * @return 		None
 **********************************************************************/
void SSP_SlaveOutputCmd(SSP_TypeDef* SSPx, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		SSPx->CR1 &= (~SSP_CR1_SO_DISABLE) & SSP_CR1_BITMASK;
	}
	else
	{
		SSPx->CR1 |= SSP_CR1_SO_DISABLE;
	}
}



/*********************************************************************//**
 * @brief		Transmit a single data through SSPx peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be SSP
 * @param[in]	Data	Data to transmit (must be 16 or 8-bit long,
 * 						this depend on SSP data bit number configured)
 * @return 		none
 **********************************************************************/
void SSP_SendData(SSP_TypeDef* SSPx, uint16_t Data)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));

	SSPx->DR = SSP_DR_BITMASK(Data);
}



/*********************************************************************//**
 * @brief		Receive a single data from SSPx peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be SSP
 * @return 		Data received (16-bit long)
 **********************************************************************/
uint16_t SSP_ReceiveData(SSP_TypeDef* SSPx)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));

	return ((uint16_t) (SSP_DR_BITMASK(SSPx->DR)));
}



/*********************************************************************//**
 * @brief		Checks whether the specified SSP status flag is set or not
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	FlagType	Type of flag to check status, should be one
 * 							of following:
 *							- SSP_STAT_TXFIFO_EMPTY: TX FIFO is empty
 *							- SSP_STAT_TXFIFO_NOTFULL: TX FIFO is not full
 *							- SSP_STAT_RXFIFO_NOTEMPTY: RX FIFO is not empty
 *							- SSP_STAT_RXFIFO_FULL: RX FIFO is full
 *							- SSP_STAT_BUSY: SSP peripheral is busy
 * @return		New State of specified SSP status flag
 **********************************************************************/
FlagStatus SSP_GetStatus(SSP_TypeDef* SSPx, uint32_t FlagType)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_SSP_STAT(FlagType));

	return ((SSPx->SR & FlagType) ? SET : RESET);
}



/*********************************************************************//**
 * @brief		Enable or disable specified interrupt type in SSP peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	IntType	Interrupt type in SSP peripheral, should be:
 * 				- SSP_INTCFG_ROR: Receive Overrun interrupt
 * 				- SSP_INTCFG_RT: Receive Time out interrupt
 * 				- SSP_INTCFG_RX: RX FIFO is at least half full interrupt
 * 				- SSP_INTCFG_TX: TX FIFO is at least half empty interrupt
 * @param[in]	NewState New State of specified interrupt type, should be:
 * 				- ENABLE: Enable this interrupt type
 * 				- DISABLE: Disable this interrupt type
 * @return		None
 **********************************************************************/
void SSP_IntConfig(SSP_TypeDef *SSPx, uint32_t IntType, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_SSP_INTCFG(IntType));

	if (NewState == ENABLE)
	{
		SSPx->IMSC |= IntType;
	}
	else
	{
		SSPx->IMSC &= (~IntType) & SSP_IMSC_BITMASK;
	}
}


/*********************************************************************//**
 * @brief	Check whether the specified Raw interrupt status flag is
 * 			set or not
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	RawIntType	Raw Interrupt Type, should be:
 * 				- SSP_INTSTAT_RAW_ROR: Receive Overrun interrupt
 * 				- SSP_INTSTAT_RAW_RT: Receive Time out interrupt
 * 				- SSP_INTSTAT_RAW_RX: RX FIFO is at least half full interrupt
 * 				- SSP_INTSTAT_RAW_TX: TX FIFO is at least half empty interrupt
 * @return	New State of specified Raw interrupt status flag in SSP peripheral
 * Note: Enabling/Disabling specified interrupt in SSP peripheral does not
 * 		effect to Raw Interrupt Status flag.
 **********************************************************************/
IntStatus SSP_GetRawIntStatus(SSP_TypeDef *SSPx, uint32_t RawIntType)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_SSP_INTSTAT_RAW(RawIntType));

	return ((SSPx->RIS & RawIntType) ? SET : RESET);
}


/*********************************************************************//**
 * @brief	Check whether the specified interrupt status flag is
 * 			set or not
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	RawIntType	Raw Interrupt Type, should be:
 * 				- SSP_INTSTAT_ROR: Receive Overrun interrupt
 * 				- SSP_INTSTAT_RT: Receive Time out interrupt
 * 				- SSP_INTSTAT_RX: RX FIFO is at least half full interrupt
 * 				- SSP_INTSTAT_TX: TX FIFO is at least half empty interrupt
 * @return	New State of specified interrupt status flag in SSP peripheral
 * Note: Enabling/Disabling specified interrupt in SSP peripheral effects
 * 			to Interrupt Status flag.
 **********************************************************************/
IntStatus SSP_GetIntStatus (SSP_TypeDef *SSPx, uint32_t IntType)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_SSP_INTSTAT(IntType));

	return ((SSPx->MIS & IntType) ? SET :RESET);
}



/*********************************************************************//**
 * @brief				Clear specified interrupt pending in SSP peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	IntType	Interrupt pending to clear, should be:
 * 						- SSP_INTCLR_ROR: clears the "frame was received when
 * 						RxFIFO was full" interrupt.
 * 						- SSP_INTCLR_RT: clears the "Rx FIFO was not empty and
 * 						has not been read for a timeout period" interrupt.
 * @return		None
 **********************************************************************/
void SSP_ClearIntPending(SSP_TypeDef *SSPx, uint32_t IntType)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_SSP_INTCLR(IntType));

	SSPx->ICR = IntType;
}

/*********************************************************************//**
 * @brief				Enable/Disable DMA function for SSP peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be SSP0 or SSP1
 * @param[in]	DMAMode	Type of DMA, should be:
 * 						- SSP_DMA_TX: DMA for the transmit FIFO
 * 						- SSP_DMA_RX: DMA for the Receive FIFO
 * @param[in]	NewState	New State of DMA function on SSP peripheral,
 * 						should be:
 * 						- ENALBE: Enable this function
 * 						- DISABLE: Disable this function
 * @return		None
 **********************************************************************/
void SSP_DMACmd(SSP_TypeDef *SSPx, uint32_t DMAMode, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_SSPx(SSPx));
	CHECK_PARAM(PARAM_SSP_DMA(DMAMode));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		SSPx->DMACR |= DMAMode;
	}
	else
	{
		SSPx->DMACR &= (~DMAMode) & SSP_DMA_BITMASK;
	}
}
/**
 * @}
 */
/**
 * @}
 */
#endif /* _SSP */

