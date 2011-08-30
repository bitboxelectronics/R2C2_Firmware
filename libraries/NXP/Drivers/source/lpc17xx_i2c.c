/**
 * @file	: lpc17xx_i2c.c
 * @brief	: Contains all functions support for I2C firmware library on LPC17xx
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

#include "lpc17xx_i2c.h"
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


#ifdef _I2C


/************************** PRIVATE VARIABLES *************************/

#ifdef _I2C0
/** @addtogroup Private_Variables
 * @{
 */

/** @defgroup I2C_Private_Variables
 * @{
 */
const PINSEL_CFG_Type i2c0_scl_pin[I2C0_MAX_SCL_PIN] = { \
		I2C0_PINSEL_SCL_P0_28 };
const PINSEL_CFG_Type i2c0_sda_pin[I2C0_MAX_SDA_PIN] = { \
		I2C0_PINSEL_SDA_P0_27 };
#endif /* _I2C0 */

#ifdef _I2C1
const PINSEL_CFG_Type i2c1_scl_pin[I2C1_MAX_SCL_PIN] = { \
		I2C1_PINSEL_SCL_P0_1, \
		I2C1_PINSEL_SCL_P0_20 };
const PINSEL_CFG_Type i2c1_sda_pin[I2C1_MAX_SDA_PIN] = { \
		I2C1_PINSEL_SDA_P0_0, \
		I2C1_PINSEL_SDA_P0_19 };
#endif /* _I2C1 */

#ifdef _I2C2
const PINSEL_CFG_Type i2c2_scl_pin[I2C2_MAX_SCL_PIN] = { \
		I2C2_PINSEL_SCL_P0_11 };
const PINSEL_CFG_Type i2c2_sda_pin[I2C2_MAX_SDA_PIN] = { \
		I2C2_PINSEL_SDA_P0_10 };
#endif /* _I2C2 */

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

/** @defgroup I2C_Public_Functions
 * @{
 */
/*********************************************************************//**
 * @brief 		Setup clock rate for I2C peripheral
 * @param[in] 	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	target_clock : clock of SSP (Hz)
 * @return 		None
 ***********************************************************************/
void I2C_SetClock (I2C_TypeDef *I2Cx, uint32_t target_clock)
{
	uint32_t temp;

	CHECK_PARAM(PARAM_I2Cx(I2Cx));

	// Get PCLK of I2C controller
	if (I2Cx == I2C0)
	{
		temp = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_I2C0) / target_clock;
	}
	else if (I2Cx == I2C1)
	{
		temp = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_I2C1) / target_clock;
	}
	else if (I2Cx == I2C2)
	{
		temp = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_I2C1) / target_clock;
	}

	/* Set the I2C clock value to register */
	I2Cx->I2SCLH = (uint32_t)(temp / 2);
	I2Cx->I2SCLL = (uint32_t)(temp - I2Cx->I2SCLH);
}


/*********************************************************************//**
 * @brief		Set all pins used as I2C function corresponding to
 * 				parameter specified in I2CPinCfg.
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	I2CPinCfg	Pointer to a I2C_PinCFG_Type structure
*                    that contains the configuration information for the
*                    specified I2C pin function.
 * @return 		None
 **********************************************************************/
void I2C_PinConfig(I2C_TypeDef *I2Cx, I2C_PinCFG_Type *I2CPinCfg)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
#ifdef _I2C0
	if(I2Cx == I2C0)
	{
		CHECK_PARAM(PARAM_I2C0_SCL(I2CPinCfg->SCL_Pin));
		CHECK_PARAM(PARAM_I2C0_SDA(I2CPinCfg->SDA_Pin));

		PINSEL_ConfigPin((PINSEL_CFG_Type *)&(i2c0_scl_pin[I2CPinCfg->SCL_Pin]));
		PINSEL_ConfigPin((PINSEL_CFG_Type *)&(i2c0_sda_pin[I2CPinCfg->SDA_Pin]));
	}
#endif

#ifdef _I2C1
	if(I2Cx == I2C1)
	{
		CHECK_PARAM(PARAM_I2C1_SCL(I2CPinCfg->SCL_Pin));
		CHECK_PARAM(PARAM_I2C1_SDA(I2CPinCfg->SDA_Pin));

		PINSEL_ConfigPin((PINSEL_CFG_Type *)&(i2c1_scl_pin[I2CPinCfg->SCL_Pin]));
		PINSEL_ConfigPin((PINSEL_CFG_Type *)&(i2c1_sda_pin[I2CPinCfg->SDA_Pin]));
	}
#endif

#ifdef I2C2
	if(I2Cx == I2C2)
	{
		CHECK_PARAM(PARAM_I2C2_SCL(I2CPinCfg->SCL_Pin));
		CHECK_PARAM(PARAM_I2C2_SDA(I2CPinCfg->SDA_Pin));

		PINSEL_ConfigPin((PINSEL_CFG_Type *)&(i2c2_scl_pin[I2CPinCfg->SCL_Pin]));
		PINSEL_ConfigPin((PINSEL_CFG_Type *)&(i2c2_sda_pin[I2CPinCfg->SDA_Pin]));
	}
#endif
}


/*****************************************************************************//**
 * @brief		Fills each I2C_PinInitStruct member with its default value:
 * 				if selected I2Cx peripheral is I2C0:
 * 				- SCL_Pin = I2C0_SCL_P0_28
 * 				- SDA_Pin = I2C0_SDA_P0_27
 * 				if selected I2Cx peripheral is I2C1:
 * 				- SCL_Pin = I2C1_SCL_P0_1
 * 				- SDA_Pin = I2C1_SDA_P0_0
 * 				if selected I2Cx peripheral is I2C2:
 * 				- SCL_Pin = I2C2_SCL_P0_11
 * 				- SDA_Pin = I2C2_SDA_P0_10
 *
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	I2C_PinInitStruct Pointer to a I2C_PinCFG_Type structure
 *                    which will be initialized.
 * @return		None
 *******************************************************************************/
void I2C_PinConfigStructInit(I2C_TypeDef *I2Cx, I2C_PinCFG_Type *I2C_PinInitStruct)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));

#if _I2C0
	if (I2Cx==I2C0)
	{
		I2C_PinInitStruct->SCL_Pin = I2C0_SCL_P0_28;
		I2C_PinInitStruct->SDA_Pin = I2C0_SDA_P0_27;
	}
#endif
#if _I2C1
	if (I2Cx==I2C1)
	{
		I2C_PinInitStruct->SCL_Pin = I2C1_SCL_P0_1;
		I2C_PinInitStruct->SDA_Pin = I2C1_SDA_P0_0;
	}
#endif
#if _I2C2
	if (I2Cx==I2C2)
	{
		I2C_PinInitStruct->SCL_Pin = I2C2_SCL_P0_11;
		I2C_PinInitStruct->SDA_Pin = I2C2_SDA_P0_10;
	}
#endif

}


/*********************************************************************//**
 * @brief		De-initializes the I2C peripheral registers to their
*                  default reset values.
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @return 		None
 **********************************************************************/
void I2C_DeInit(I2C_TypeDef* I2Cx)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));

	/* Disable I2C control */
	I2Cx->I2CONCLR = I2C_I2CONCLR_I2ENC;

#if _I2C0
	if (I2Cx==I2C0)
	{
		/* Disable power for I2C0 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C0, DISABLE);
	}
#endif
#if _I2C1
	if (I2Cx==I2C1)
	{
		/* Disable power for I2C1 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C1, DISABLE);
	}
#endif
#if _I2C2
	if (I2Cx==I2C2)
	{
		/* Disable power for I2C2 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C2, DISABLE);
	}
#endif
}


/********************************************************************//**
 * @brief		Initializes the I2Cx peripheral with specified parameter.
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	clockrate Target clock rate value to initialized I2C
 * 				peripheral
 * @return 		None
 *********************************************************************/
void I2C_Init(I2C_TypeDef *I2Cx, uint32_t clockrate)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));

	I2C_PinCFG_Type defaultPin;

#if _I2C0
	if (I2Cx==I2C0)
	{
		/* Set up clock and power for I2C0 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C0, ENABLE);
		/* As default, peripheral clock for I2C0 module
		 * is set to FCCLK / 2 */
		CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_I2C0, CLKPWR_PCLKSEL_CCLK_DIV_2);
	}
#endif
#if _I2C1
	if (I2Cx==I2C1)
	{
		/* Set up clock and power for I2C1 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C1, ENABLE);
		/* As default, peripheral clock for I2C1 module
		 * is set to FCCLK / 2 */
		CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_I2C1, CLKPWR_PCLKSEL_CCLK_DIV_2);
	}
#endif
#if _I2C2
	if (I2Cx==I2C2)
	{
		/* Set up clock and power for I2C2 module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C2, ENABLE);
		/* As default, peripheral clock for I2C2 module
		 * is set to FCCLK / 2 */
		CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_I2C2, CLKPWR_PCLKSEL_CCLK_DIV_2);
	}
#endif

	/* Use default I2C pin function */
	I2C_PinConfigStructInit(I2Cx, &defaultPin);
	I2C_PinConfig(I2Cx, &defaultPin);

    /* Set clock rate */
    I2C_SetClock(I2Cx, clockrate);
    /* Set I2C operation to default */
    I2Cx->I2CONCLR = (I2C_I2CONCLR_AAC | I2C_I2CONCLR_STAC | I2C_I2CONCLR_I2ENC);
}


/*********************************************************************//**
 * @brief		Enable or disable I2C peripheral's operation
 * @param[in]	I2Cx I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	NewState New State of I2Cx peripheral's operation
 * @return 		none
 **********************************************************************/
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));
	CHECK_PARAM(PARAM_I2Cx(I2Cx));

	if (NewState == ENABLE)
	{
		I2Cx->I2CONSET = I2C_I2CONSET_I2EN;
	}
	else
	{
		I2Cx->I2CONCLR = I2C_I2CONCLR_I2ENC;
	}
}


/*********************************************************************//**
 * @brief		Checks whether if the I2C interrupt flag is set or not
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @return		New State of I2C interrupt flag
 **********************************************************************/
IntStatus I2C_GetIntStatus(I2C_TypeDef* I2Cx)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	return ((I2Cx->I2CONSET & I2C_I2CONSET_SI) ? SET : RESET);
}


/*********************************************************************//**
 * @brief		Generate a start/RepeatStart condition on I2C bus
 * 				(enter I2C master mode)
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @return		None
 **********************************************************************/
void I2C_SendStart(I2C_TypeDef *I2Cx)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	I2Cx->I2CONCLR = I2C_I2CONCLR_SIC | I2C_I2CONCLR_AAC;
	I2Cx->I2CONSET = I2C_I2CONSET_STA;
}


/*********************************************************************//**
 * @brief		Generate a stop condition on I2C bus (in I2C master mode)
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @return		None
 **********************************************************************/
void I2C_SendStop(I2C_TypeDef *I2Cx)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	/* Make sure start bit is not active */
	if (I2Cx->I2CONSET & I2C_I2CONSET_STA)
	{
		I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
	}
	I2Cx->I2CONSET = I2C_I2CONSET_STO;
	I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
}


/*********************************************************************//**
 * @brief		Send a slave address with Read/Write direction on I2C bus
 * 				(in I2C master mode)
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	SlaveAddr_7bit A 7-bit slave address value
 * @param[in]	RWDirection Read/Write direction followed by slave address value,
 * 				should be:
 * 				- I2C_DIRECTION_WR: Write command
 * 				- I2C_DIRECTION_RD: Read command
 * @return		None
 **********************************************************************/
void I2C_SendSlaveAddr(I2C_TypeDef *I2Cx, uint8_t SlaveAddr_7bit, uint8_t RWDirection)
{
	uint8_t addr;
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	CHECK_PARAM(PARAM_I2C_RWDIR(RWDirection));

	addr = (SlaveAddr_7bit << 1) | RWDirection;
	I2C_SendData(I2Cx, addr);
}


/*********************************************************************//**
 * @brief		Enable/Disable acknowledge (low level to SDA) that will be
 * 				returned during the acknowledge clock pulse on the SCL line.
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	NewState New State of this function, should be:
 * 				- ENABLE: Enable this function.
 * 				- DISABLE: Disable this function.
 * @return		None
 **********************************************************************/
void I2C_AssertAckCmd(I2C_TypeDef *I2Cx, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		I2Cx->I2CONSET = I2C_I2CONSET_AA;
	}
	else
	{
		I2Cx->I2CONCLR = I2C_I2CONCLR_AAC;
	}
}


/*********************************************************************//**
 * @brief		Send data on I2C bus
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	Data	8-bit Data to send
 * @return		None
 **********************************************************************/
void I2C_SendData(I2C_TypeDef *I2Cx, uint8_t Data)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	/* Make sure start bit is not active */
	if (I2Cx->I2CONSET & I2C_I2CONSET_STA)
	{
		I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
	}
	I2Cx->I2DAT = (uint32_t)(Data & I2C_I2DAT_BITMASK);
	I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
}


/*********************************************************************//**
 * @brief		Receive data from the I2C peripheral's buffer
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	ReceiveNext	Enable/Disable Receive in Next transmission,
 * 							should be:
 * 							- ENABLE: Enable receive in next transmission
 * 							- DISABLE: Disable receive in next transmission
 * @return		Received Data
 * Note: Since the interrupt Flag SI must be cleared after receiving data
 * 			to accept next transfer. So this function support ReceiveNext
 * 			input parameter to enable/disable this action after receiving data.
 * 			If ReceiveNext is enabled, SI flag will be clear after reading data
 * 			from I2C buffer, then next receiving data will occur again.
 * 			If ReceiveNext is disable, SI flag will not be clear, so the next
 * 			receiving data will not occur.
 **********************************************************************/
uint8_t I2C_ReceiveData(I2C_TypeDef *I2Cx, FunctionalState ReceiveNext)
{
	uint8_t tmp;
	CHECK_PARAM(PARAM_I2Cx(I2Cx));

	tmp = ((uint8_t)I2Cx->I2DAT) & I2C_I2DAT_BITMASK;
	if (ReceiveNext == ENABLE)
	{
		I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
	}
	return tmp;
}


/*********************************************************************//**
 * @brief		Set Own slave address in I2C peripheral corresponding to
 * 				parameter specified in OwnSlaveAddrConfigStruct.
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	OwnSlaveAddrConfigStruct	Pointer to a I2C_OWNSLAVEADDR_CFG_Type
 * 				structure that contains the configuration information for the
*               specified I2C slave address.
 * @return 		None
 **********************************************************************/
void I2C_SetOwnSlaveAddr(I2C_TypeDef *I2Cx, I2C_OWNSLAVEADDR_CFG_Type *OwnSlaveAddrConfigStruct)
{
	uint32_t tmp;
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	CHECK_PARAM(PARAM_I2C_SLAVEADDR_CH(OwnSlaveAddrConfigStruct->SlaveAddrChannel));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(OwnSlaveAddrConfigStruct->GeneralCallState));

	tmp = (((uint32_t)(OwnSlaveAddrConfigStruct->SlaveAddr_7bit << 1)) \
			| ((OwnSlaveAddrConfigStruct->GeneralCallState == ENABLE) ? 0x01 : 0x00))& I2C_I2ADR_BITMASK;
	switch (OwnSlaveAddrConfigStruct->SlaveAddrChannel)
	{
	case 0:
		I2Cx->I2ADR0 = tmp;
		I2Cx->I2MASK0 = I2C_I2MASK_MASK((uint32_t) \
				(OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
		break;
	case 1:
		I2Cx->I2ADR1 = tmp;
		I2Cx->I2MASK1 = I2C_I2MASK_MASK((uint32_t) \
				(OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
		break;
	case 2:
		I2Cx->I2ADR2 = tmp;
		I2Cx->I2MASK2 = I2C_I2MASK_MASK((uint32_t) \
				(OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
		break;
	case 3:
		I2Cx->I2ADR3 = tmp;
		I2Cx->I2MASK3 = I2C_I2MASK_MASK((uint32_t) \
				(OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
		break;
	}
}


/*********************************************************************//**
 * @brief		Get the last status code return from I2C status register
 * 				according to its last operation
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @return		The last status code in I2C peripheral
 **********************************************************************/
uint8_t I2C_GetLastStatusCode(I2C_TypeDef* I2Cx)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	return (uint8_t)(I2C_STAT_CODE_VALUE(I2Cx->I2STAT));
}


/*********************************************************************//**
 * @brief		Configures functionality in I2C monitor mode
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	MonitorCfgType Monitor Configuration type, should be:
 * 				- I2C_MONITOR_CFG_SCL_OUTPUT: I2C module can 'stretch'
 * 				the clock line (hold it low) until it has had time to
 * 				respond to an I2C interrupt.
 * 				- I2C_MONITOR_CFG_MATCHALL: When this bit is set to '1'
 * 				and the I2C is in monitor mode, an interrupt will be
 * 				generated on ANY address received.
 * @param[in]	NewState New State of this function, should be:
 * 				- ENABLE: Enable this function.
 * 				- DISABLE: Disable this function.
 * @return		None
 **********************************************************************/
void I2C_MonitorModeConfig(I2C_TypeDef *I2Cx, uint32_t MonitorCfgType, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	CHECK_PARAM(PARAM_I2C_MONITOR_CFG(MonitorCfgType));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		I2Cx->MMCTRL |= MonitorCfgType;
	}
	else
	{
		I2Cx->MMCTRL &= (~MonitorCfgType) & I2C_I2MMCTRL_BITMASK;
	}
}


/*********************************************************************//**
 * @brief		Enable/Disable I2C monitor mode
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @param[in]	NewState New State of this function, should be:
 * 				- ENABLE: Enable monitor mode.
 * 				- DISABLE: Disable monitor mode.
 * @return		None
 **********************************************************************/
void I2C_MonitorModeCmd(I2C_TypeDef *I2Cx, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		I2Cx->MMCTRL |= I2C_I2MMCTRL_MM_ENA;
	}
	else
	{
		I2Cx->MMCTRL &= (~I2C_I2MMCTRL_MM_ENA) & I2C_I2MMCTRL_BITMASK;
	}
}


/*********************************************************************//**
 * @brief		Get data from I2C data buffer in monitor mode.
 * @param[in]	I2Cx	I2C peripheral selected, should be I2C0, I2C1 or I2C2
 * @return		None
 * Note:	In monitor mode, the I2C module may lose the ability to stretch
 * the clock (stall the bus) if the ENA_SCL bit is not set. This means that
 * the processor will have a limited amount of time to read the contents of
 * the data received on the bus. If the processor reads the I2DAT shift
 * register, as it ordinarily would, it could have only one bit-time to
 * respond to the interrupt before the received data is overwritten by
 * new data.
 **********************************************************************/
uint8_t I2C_MonitorGetDatabuffer(I2C_TypeDef *I2Cx)
{
	CHECK_PARAM(PARAM_I2Cx(I2Cx));
	return ((uint8_t)(I2Cx->I2DATA_BUFFER));
}

/**
 * @}
 */

/**
 * @}
 */
#endif /* _I2C */
