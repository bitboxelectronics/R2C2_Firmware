/**
 * @file	: lpc17xx_spi.c
 * @brief	: Contains all functions support for SPI firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 3. April. 2009
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

#include "lpc17xx_spi.h"
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

#ifdef _SPI


/************************** GLOBAL/PUBLIC FUNCTIONS *************************/
/** @addtogroup Public_Functions
  * @{
  */

/** @defgroup SPI_Public_Functions
 * @{
 */

/*********************************************************************//**
 * @brief 		Setup clock rate for SPI device
 * @param[in] 	SPIx	SPI peripheral definition, should be SPI
 * @param[in]	target_clock : clock of SPI (Hz)
 * @return 		Status of process (ERROR or SUCCESS)
 ***********************************************************************/
Status SPI_SetClock (SPI_TypeDef *SPIx, uint32_t target_clock)
{
	uint32_t spi_pclk;
	uint32_t prescale, temp = 0;

	CHECK_PARAM(PARAM_SPIx(SPIx));

	if (SPIx == SPI)
	{
		spi_pclk =  CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SPI);
	}

	prescale = 8;

	while (temp < spi_pclk)
	{
		prescale++;
		if(prescale > 255)
		{
			break;
		}

		temp = target_clock * prescale;
	}

	if ((prescale < 8) && (target_clock > spi_pclk))
	{
		return ERROR;
	}

	SPIx->SPCCR = SPI_SPCCR_COUNTER(prescale);

	return SUCCESS;
}



/*********************************************************************//**
 * @brief		Set all pins used as SPIx function corresponding to
 * 				parameter specified in SPIPinCfg.
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @param[in]	SPIPinCfg	Pointer to a SPI_PinCFG_Type structure
*                    that contains the configuration information for the
*                    specified SPIx pin function.
* @param[in]	spiMode	SPI mode, should be:
* 				- SPI_SLAVE_MODE: SLAVE mode
* 				- SPI_MASTER_MODE: MASTER mode
 * @return 		None
 **********************************************************************/
void SPI_PinConfig(SPI_TypeDef *SPIx, SPI_PinCFG_Type *SPIPinCfg, int32_t spiMode)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));
	CHECK_PARAM(PARAM_SPI_SCK(SPIPinCfg->SCK_Pin));
	CHECK_PARAM(PARAM_SPI_SSEL(SPIPinCfg->SSEL_Pin));
	CHECK_PARAM(PARAM_SPI_MISO(SPIPinCfg->MISO_Pin));
	CHECK_PARAM(PARAM_SPI_MOSI(SPIPinCfg->MOSI_Pin));

	// SCK pin
	switch (SPIPinCfg->SCK_Pin)
	{
	case SPI_SCK_P0_15:
		PINSEL_SetPinFunc (PINSEL_PORT_0, SPI_PINSEL_SCK_P0_15, SPI_PINFUNC_SCK_P0_15);
		PINSEL_SetResistorMode (PINSEL_PORT_0, SPI_PINSEL_SCK_P0_15, PINSEL_PINMODE_PULLUP);
		break;
	}


	// SSEL pin
	switch (SPIPinCfg->SSEL_Pin)
	{
	case SPI_SSEL_P0_16:
		if (spiMode == SPI_SLAVE_MODE)
		{
			PINSEL_SetPinFunc (PINSEL_PORT_0, SPI_PINSEL_SSEL_P0_16, SPI_PINFUNC_SSEL_P0_16);
			PINSEL_SetResistorMode (PINSEL_PORT_0, SPI_PINSEL_SSEL_P0_16, PINSEL_PINMODE_PULLUP);
		}
		else
		{
			// Release SSEL pin as default function
			PINSEL_SetPinFunc (PINSEL_PORT_0, SPI_PINSEL_SSEL_P0_16, PINSEL_FUNC_0);
			PINSEL_SetResistorMode (PINSEL_PORT_0, SPI_PINSEL_SSEL_P0_16, PINSEL_PINMODE_PULLUP);
		}
		break;
	}


	// MISO pin
	switch (SPIPinCfg->MISO_Pin)
	{
	case SPI_MISO_P0_17:
		PINSEL_SetPinFunc (PINSEL_PORT_0, SPI_PINSEL_MISO_P0_17, SPI_PINFUNC_MISO_P0_17);
		PINSEL_SetResistorMode (PINSEL_PORT_0, SPI_PINSEL_MISO_P0_17, PINSEL_PINMODE_PULLUP);
		break;
	}

	// MOSI pin
	switch (SPIPinCfg->MOSI_Pin)
	{
	case SPI_MOSI_P0_18:
		PINSEL_SetPinFunc (PINSEL_PORT_0, SPI_PINSEL_MOSI_P0_18, SPI_PINFUNC_MOSI_P0_18);
		PINSEL_SetResistorMode (PINSEL_PORT_0, SPI_PINSEL_MOSI_P0_18, PINSEL_PINMODE_PULLUP);
		break;
	}
}


/*****************************************************************************//**
* @brief		Fills each SPI_PinInitStruct member with its default value:
* 				- MISO_Pin = SPI_MISO_P0_17
				- MOSI_Pin = SPI_MOSI_P0_18
				- SCK_Pin = SPI_SCK_P0_15
				- SSEL_Pin = SPI_SSEL_P0_16
* @param[in]	SPI_PinInitStruct Pointer to a SPI_PinCFG_Type structure
*                    which will be initialized.
* @return		None
*******************************************************************************/
void SPI_PinConfigStructInit(SPI_PinCFG_Type *SPI_PinInitStruct)
{
	SPI_PinInitStruct->MISO_Pin = SPI_MISO_P0_17;
	SPI_PinInitStruct->MOSI_Pin = SPI_MOSI_P0_18;
	SPI_PinInitStruct->SCK_Pin = SPI_SCK_P0_15;
	SPI_PinInitStruct->SSEL_Pin = SPI_SSEL_P0_16;
}


/*********************************************************************//**
 * @brief		De-initializes the SPIx peripheral registers to their
*                  default reset values.
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @return 		None
 **********************************************************************/
void SPI_DeInit(SPI_TypeDef* SPIx)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));

	if (SPIx == SPI)
	{
		/* Set up clock and power for SPI module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSPI, DISABLE);
	}
}



/********************************************************************//**
 * @brief		Initializes the SPIx peripheral according to the specified
*               parameters in the UART_ConfigStruct.
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @param[in]	SPI_ConfigStruct Pointer to a SPI_CFG_Type structure
*                    that contains the configuration information for the
*                    specified SPI peripheral.
 * @return 		None
 *********************************************************************/
void SPI_Init(SPI_TypeDef *SPIx, SPI_CFG_Type *SPI_ConfigStruct)
{
	SPI_PinCFG_Type defaultSPIPinCfg;
	uint32_t tmp;

	CHECK_PARAM(PARAM_SPIx(SPIx));

	if(SPIx == SPI)
	{
		/* Set up clock and power for UART module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSPI, ENABLE);
		/* As default, peripheral clock for UART0 module
		 * is set to FCCLK / 2 */
		CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_SPI, CLKPWR_PCLKSEL_CCLK_DIV_2);
		// Set UART0 function pin as default
		defaultSPIPinCfg.SCK_Pin = SPI_SCK_P0_15;
		defaultSPIPinCfg.SSEL_Pin = SPI_SSEL_P0_16;
		defaultSPIPinCfg.MISO_Pin = SPI_MISO_P0_17;
		defaultSPIPinCfg.MOSI_Pin = SPI_MOSI_P0_18;
		SPI_PinConfig(SPIx, &defaultSPIPinCfg, SPI_ConfigStruct->Mode);
	}

	// Configure SPI, interrupt is disable as default
	tmp = ((SPI_ConfigStruct->CPHA) | (SPI_ConfigStruct->CPOL) \
		| (SPI_ConfigStruct->DataOrder) | (SPI_ConfigStruct->Databit) \
		| (SPI_ConfigStruct->Mode) | SPI_SPCR_BIT_EN) & SPI_SPCR_BITMASK;
	// write back to SPI control register
	SPIx->SPCR = tmp;

	// Set clock rate for SPI peripheral
	SPI_SetClock(SPIx, SPI_ConfigStruct->ClockRate);

	// If interrupt flag is set, Write '1' to Clear interrupt flag
	if (SPIx->SPINT & SPI_SPINT_INTFLAG)
	{
		SPIx->SPINT = SPI_SPINT_INTFLAG;
	}
}



/*****************************************************************************//**
* @brief		Fills each SPI_InitStruct member with its default value:
* 				- CPHA = SPI_CPHA_FIRST
* 				- CPOL = SPI_CPOL_HI
* 				- ClockRate = 1000000
* 				- DataOrder = SPI_DATA_MSB_FIRST
* 				- Databit = SPI_DATABIT_8
* 				- Mode = SPI_MASTER_MODE
* @param[in]	SPI_InitStruct Pointer to a SPI_CFG_Type structure
*                    which will be initialized.
* @return		None
*******************************************************************************/
void SPI_ConfigStructInit(SPI_CFG_Type *SPI_InitStruct)
{
	SPI_InitStruct->CPHA = SPI_CPHA_FIRST;
	SPI_InitStruct->CPOL = SPI_CPOL_HI;
	SPI_InitStruct->ClockRate = 1000000;
	SPI_InitStruct->DataOrder = SPI_DATA_MSB_FIRST;
	SPI_InitStruct->Databit = SPI_DATABIT_8;
	SPI_InitStruct->Mode = SPI_MASTER_MODE;
}


/*********************************************************************//**
 * @brief
 * @param[in]
 * @return none
 **********************************************************************/
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState)
{

}



/*********************************************************************//**
 * @brief		Transmit a single data through SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @param[in]	Data	Data to transmit (must be 16 or 8-bit long,
 * 						this depend on SPI data bit number configured)
 * @return 		none
 **********************************************************************/
void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));

	SPIx->SPDR = Data & SPI_SPDR_BITMASK;
}



/*********************************************************************//**
 * @brief		Receive a single data from SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @return 		Data received (16-bit long)
 **********************************************************************/
uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));

	return ((uint16_t) (SPIx->SPDR & SPI_SPDR_BITMASK));
}



/********************************************************************//**
 * @brief 		Enable or disable SPIx interrupt.
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @param[in]	NewState New state of specified UART interrupt type,
 * 				should be:
 * 				- ENALBE: Enable this SPI interrupt.
* 				- DISALBE: Disable this SPI interrupt.
 * @return 		None
 *********************************************************************/
void SPI_IntCmd(SPI_TypeDef *SPIx, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		SPIx->SPCR |= SPI_SPCR_SPIE;
	}
	else
	{
		SPIx->SPCR &= (~SPI_SPCR_SPIE) & SPI_SPCR_BITMASK;
	}
}


/********************************************************************//**
 * @brief 		Checks whether the SPI interrupt flag is set or not.
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @return 		The new state of SPI Interrupt Flag (SET or RESET)
 *********************************************************************/
IntStatus SPI_GetIntStatus (SPI_TypeDef *SPIx)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));

	return ((SPIx->SPINT & SPI_SPINT_INTFLAG) ? SET : RESET);
}


/********************************************************************//**
 * @brief 		Clear SPI interrupt flag.
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @return 		None
 *********************************************************************/
void SPI_ClearIntPending(SPI_TypeDef *SPIx)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));

	SPIx->SPINT = SPI_SPINT_INTFLAG;
}


/********************************************************************//**
 * @brief 		Get current value of SPI Status register in SPIx peripheral.
 * @param[in]	SPIx	SPI peripheral selected, should be SPI
 * @return		Current value of SPI Status register in SPI peripheral.
 * Note:	The return value of this function must be used with
 * 			SPI_CheckStatus() to determine current flag status
 * 			corresponding to each SPI status type. Because some flags in
 * 			SPI Status register will be cleared after reading, the next reading
 * 			SPI Status register could not be correct. So this function used to
 * 			read SPI status register in one time only, then the return value
 * 			used to check all flags.
 *********************************************************************/
uint32_t SPI_GetStatus(SPI_TypeDef* SPIx)
{
	CHECK_PARAM(PARAM_SPIx(SPIx));

	return (SPIx->SPSR & SPI_SPSR_BITMASK);
}



/********************************************************************//**
 * @brief 		Checks whether the specified SPI Status flag is set or not
 * 				via inputSPIStatus parameter.
 * @param[in]	inputSPIStatus Value to check status of each flag type.
 * 				This value is the return value from SPI_GetStatus().
 * @param[in]	SPIStatus	Specifies the SPI status flag to check,
 * 				should be one of the following:
				- SPI_STAT_ABRT: Slave abort.
				- SPI_STAT_MODF: Mode fault.
				- SPI_STAT_ROVR: Read overrun.
				- SPI_STAT_WCOL: Write collision.
				- SPI_STAT_SPIF: SPI transfer complete.
 * @return 		The new state of SPIStatus (SET or RESET)
 *********************************************************************/
FlagStatus SPI_CheckStatus (uint32_t inputSPIStatus,  uint8_t SPIStatus)
{
	CHECK_PARAM(PARAM_SPI_STAT(SPIStatus));

	return ((inputSPIStatus & SPIStatus) ? SET : RESET);
}
/**
 * @}
 */

/**
 * @}
 */

#endif /* _SPI */
