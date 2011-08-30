/***********************************************************************//**
 * @file	: lpc17xx_i2c.h
 * @brief	: Contains all macro definitions and function prototypes
 * 				support for I2C firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 13. Apr. 2009
 * @author	: HieuNguyen
 **************************************************************************
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
 **************************************************************************/

#ifndef LPC17XX_I2C_H_
#define LPC17XX_I2C_H_

#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/****************************** PRIVATE MACROS ******************************/
/** @addtogroup PRIVATE_MACROS
 * @{
 */


/** @defgroup I2C_PINSEL
 * @{
 */

/* Pin Configuration selection must be defined in structure following:
 * - Port Number,
 * - Pin Number,
 * - Function Number,
 * - Pin Mode,
 * - Open Drain
 */

/** I2C0 function pin selection group 0 */
#define I2C0_PINSEL_SDA_P0_27	{0, 27, 1, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}
#define I2C0_PINSEL_SCL_P0_28	{0, 28, 1, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}

/** I2C1 function pin selection group 0 */
#define I2C1_PINSEL_SDA_P0_0	{0, 0, 3, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}
#define I2C1_PINSEL_SCL_P0_1	{0, 1, 3, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}

/** I2C1 function pin selection group 1 */
#define I2C1_PINSEL_SDA_P0_19	{0, 19, 3, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}
#define I2C1_PINSEL_SCL_P0_20	{0, 20, 3, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}

/** I2C2 function pin selection group 0 */
#define I2C2_PINSEL_SDA_P0_10	{0, 10, 2, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}
#define I2C2_PINSEL_SCL_P0_11	{0, 11, 2, \
					PINSEL_PINMODE_PULLUP, \
					PINSEL_PINMODE_NORMAL}

/* Max number of pin on each pin function */
#define I2C0_MAX_SCL_PIN	(1)
#define I2C0_MAX_SDA_PIN	(1)

#define I2C1_MAX_SCL_PIN	(2)
#define I2C1_MAX_SDA_PIN	(2)

#define I2C2_MAX_SCL_PIN	(1)
#define I2C2_MAX_SDA_PIN	(1)
/**
 * @}
 */

/** @defgroup I2C_REGISTER_BIT_DEFINITIONS
 * @{
 */
/*******************************************************************//**
 * I2C Control Set register description
 *********************************************************************/
#define I2C_I2CONSET_AA			((uint32_t)(0x04)) /*!< Assert acknowledge flag */
#define I2C_I2CONSET_SI			((uint32_t)(0x08)) /*!< I2C interrupt flag */
#define I2C_I2CONSET_STO		((uint32_t)(0x10)) /*!< STOP flag */
#define I2C_I2CONSET_STA		((uint32_t)(0x20)) /*!< START flag */
#define I2C_I2CONSET_I2EN		((uint32_t)(0x40)) /*!< I2C interface enable */


/*******************************************************************//**
 * I2C Control Clear register description
 *********************************************************************/
/** Assert acknowledge Clear bit */
#define I2C_I2CONCLR_AAC						((uint32_t)(1<<2))
/** I2C interrupt Clear bit */
#define I2C_I2CONCLR_SIC						((uint32_t)(1<<3))
/** START flag Clear bit */
#define I2C_I2CONCLR_STAC						((uint32_t)(1<<5))
/** I2C interface Disable bit */
#define I2C_I2CONCLR_I2ENC						((uint32_t)(1<<6))


/********************************************************************//**
 * I2C Status Code definition (I2C Status register)
 *********************************************************************/

/* Return Code in I2C status register */
#define I2C_STAT_CODE_VALUE(n)					((uint32_t)(n&0xF8))


/********************************************************************//**
 * I2C Data register definition
 *********************************************************************/
/** Mask for I2DAT register*/
#define I2C_I2DAT_BITMASK						((uint8_t)(0xFF))


/********************************************************************//**
 * I2C Monitor mode control register description
 *********************************************************************/
#define I2C_I2MMCTRL_MM_ENA			((uint32_t)(1<<0))		/**< Monitor mode enable */
#define I2C_I2MMCTRL_ENA_SCL		((uint32_t)(1<<1))		/**< SCL output enable */
#define I2C_I2MMCTRL_MATCH_ALL		((uint32_t)(1<<2))		/**< Select interrupt register match */
#define I2C_I2MMCTRL_BITMASK		((uint32_t)(0x07))		/**< Mask for I2MMCTRL register */


/********************************************************************//**
 * I2C Data buffer register description
 *********************************************************************/
/** I2C Data buffer register bit mask */
#define I2DATA_BUFFER_BITMASK		((uint32_t)(0xFF))


/********************************************************************//**
 * I2C Slave Address registers definition
 *********************************************************************/
/** General Call enable bit */
#define I2C_I2ADR_GC				((uint32_t)(1<<0))
/** I2C Slave Address registers bit mask */
#define I2C_I2ADR_BITMASK			((uint32_t)(0xFF))


/********************************************************************//**
 * I2C Mask Register definition
 *********************************************************************/
/** I2C Mask Register mask field */
#define I2C_I2MASK_MASK(n)			((uint32_t)(n&0xFE))


/********************************************************************//**
 * I2C SCL HIGH duty cycle Register definition
 *********************************************************************/
/** I2C SCL HIGH duty cycle Register bit mask */
#define I2C_I2SCLH_BITMASK			((uint32_t)(0xFFFF))


/********************************************************************//**
 * I2C SCL LOW duty cycle Register definition
 *********************************************************************/
/** I2C SCL LOW duty cycle Register bit mask */
#define I2C_I2SCLL_BITMASK			((uint32_t)(0xFFFF))

/**
 * @}
 */

/**
 * @}
 */
/**************************** GLOBAL/PUBLIC TYPES ***************************/
/** @addtogroup PUBLIC_TYPES
 * @{
 */

/** @defgroup I2C_TYPES
 * @{
 */
/** I2C pin configuration structure */
typedef struct {
	uint8_t	SCL_Pin;	/** SCL Pin configuration, should be:
						- I2C0_SCL_P0_28 (I2C0)
						- I2C1_SCL_P0_1 (I2C1)
						- I2C1_SCL_P0_20 (I2C1)
						- I2C2_SCL_P0_11 (I2C2)
						*/
	uint8_t	SDA_Pin;	/** SDA Pin configuration, should be:
						- I2C0_SDA_P0_27 (I2C0)
						- I2C1_SDA_P0_0 (I2C1)
						- I2C1_SDA_P0_19 (I2C1)
						- I2C2_SDA_P0_10 (I2C2)
						*/
} I2C_PinCFG_Type;

/** I2C Own slave address setting structure */
typedef struct {
	uint8_t SlaveAddrChannel;	/**< Slave Address channel in I2C control,
								should be in range from 0..3
								*/
	uint8_t SlaveAddr_7bit;		/**< Value of 7-bit slave address */
	uint8_t GeneralCallState;	/**< Enable/Disable General Call Functionality
								when I2C control being in Slave mode, should be:
								- ENABLE: Enable General Call function.
								- DISABLE: Disable General Call function.
								*/
	uint8_t SlaveAddrMaskValue;	/**< Any bit in this 8-bit value (bit 7:1)
								which is set to '1' will cause an automatic compare on
								the corresponding bit of the received address when it
								is compared to the SlaveAddr_7bit value associated with this
								mask register. In other words, bits in SlaveAddr_7bit value
								which are masked are not taken into account in determining
								an address match
								*/
} I2C_OWNSLAVEADDR_CFG_Type;

/**
 * @}
 */

/**
 * @}
 */
#define PARAM_I2C_SLAVEADDR_CH(n)	((n>=0) && (n<=3))

/*************************** GLOBAL/PUBLIC MACROS ***************************/

/** @addtogroup PUBLIC_MACROS
 * @{
 */

/** @defgroup I2C_MACROS
 * @{
 */
/** Macro to determine if it is valid SSP port number */
#define PARAM_I2Cx(n)	((((uint32_t *)n)==((uint32_t *)I2C0)) \
						|| (((uint32_t *)n)==((uint32_t *)I2C1)) \
						|| (((uint32_t *)n)==((uint32_t *)I2C2)))


/** I2C0 function pin selection defines */
#define I2C0_SCL_P0_28	((uint8_t)(0))
#define I2C0_SDA_P0_27	((uint8_t)(0))


/** I2C1 function pin selection defines */
#define I2C1_SDA_P0_0	((uint8_t)(0))
#define I2C1_SCL_P0_1	((uint8_t)(0))

#define I2C1_SDA_P0_19	((uint8_t)(1))
#define I2C1_SCL_P0_20	((uint8_t)(1))


/** I2C2 function pin selection defines */
#define I2C2_SDA_P0_10	((uint8_t)(0))
#define I2C2_SCL_P0_11	((uint8_t)(0))


/** Macro to check function configuration pin */
#define	PARAM_I2C0_SCL(n)	((n==I2C0_SCL_P0_28))
#define PARAM_I2C0_SDA(n)	((n==I2C0_SDA_P0_27))

#define PARAM_I2C1_SCL(n)	((n==I2C1_SCL_P0_1) || (n==I2C1_SCL_P0_20))
#define PARAM_I2C1_SDA(n)	((n==I2C1_SDA_P0_0) || (n==I2C1_SDA_P0_19))

#define PARAM_I2C2_SCL(n)	((n==I2C2_SCL_P0_11))
#define PARAM_I2C2_SDA(n)	((n==I2C2_SDA_P0_10))


/** Read/Write direction */
#define I2C_DIRECTION_WR	((uint8_t)(0))
#define I2C_DIRECTION_RD	((uint8_t)(1))
#define PARAM_I2C_RWDIR(n)	((n==I2C_DIRECTION_WR) || (n==I2C_DIRECTION_RD))


/** I2C return status code */
/* No relevant information */
#define I2C_I2STAT_NO_INF						((uint8_t)(0xF8))
/* Master transmit mode -------------------------------------------- */
/** A start condition has been transmitted */
#define I2C_I2STAT_M_TX_START					((uint8_t)(0x08))
/** A repeat start condition has been transmitted */
#define I2C_I2STAT_M_TX_RESTART					((uint8_t)(0x10))
/** SLA+W has been transmitted, ACK has been received */
#define I2C_I2STAT_M_TX_SLAW_ACK				((uint8_t)(0x18))
/** SLA+W has been transmitted, NACK has been received */
#define I2C_I2STAT_M_TX_SLAW_NACK				((uint8_t)(0x20))
/** Data has been transmitted, ACK has been received */
#define I2C_I2STAT_M_TX_DAT_ACK					((uint8_t)(0x28))
/** Data has been transmitted, NACK has been received */
#define I2C_I2STAT_M_TX_DAT_NACK				((uint8_t)(0x30))
/** Arbitration lost in SLA+R/W or Data bytes */
#define I2C_I2STAT_M_TX_ARB_LOST				((uint8_t)(0x38))


/* Master receive mode -------------------------------------------- */
/** A start condition has been transmitted */
#define I2C_I2STAT_M_RX_START					((uint8_t)(0x08))
/** A repeat start condition has been transmitted */
#define I2C_I2STAT_M_RX_RESTART					((uint8_t)(0x10))
/** Arbitration lost */
#define I2C_I2STAT_M_RX_ARB_LOST				((uint8_t)(0x38))
/** SLA+R has been transmitted, ACK has been received */
#define I2C_I2STAT_M_RX_SLAR_ACK				((uint8_t)(0x40))
/** SLA+R has been transmitted, NACK has been received */
#define I2C_I2STAT_M_RX_SLAR_NACK				((uint8_t)(0x48))
/** Data has been received, ACK has been returned */
#define I2C_I2STAT_M_RX_DAT_ACK					((uint8_t)(0x50))
/** Data has been received, NACK has been return */
#define I2C_I2STAT_M_RX_DAT_NACK				((uint8_t)(0x58))


/* Slave receive mode -------------------------------------------- */
/** Own slave address has been received, ACK has been returned */
#define I2C_I2STAT_S_RX_SLAW_ACK				((uint8_t)(0x60))

/** Arbitration lost in SLA+R/W as master */
#define I2C_I2STAT_S_RX_ARB_LOST_M_SLA			((uint8_t)(0x68))
/** Own SLA+W has been received, ACK returned */
//#define I2C_I2STAT_S_RX_SLAW_ACK				((uint8_t)(0x68))

/** General call address has been received, ACK has been returned */
#define I2C_I2STAT_S_RX_GENCALL_ACK				((uint8_t)(0x70))

/** Arbitration lost in SLA+R/W (GENERAL CALL) as master */
#define I2C_I2STAT_S_RX_ARB_LOST_M_GENCALL		((uint8_t)(0x78))
/** General call address has been received, ACK has been returned */
//#define I2C_I2STAT_S_RX_GENCALL_ACK				((uint8_t)(0x78))

/** Previously addressed with own SLV address;
 * Data has been received, ACK has been return */
#define I2C_I2STAT_S_RX_PRE_SLA_DAT_ACK			((uint8_t)(0x80))
/** Previously addressed with own SLA;
 * Data has been received and NOT ACK has been return */
#define I2C_I2STAT_S_RX_PRE_SLA_DAT_NACK		((uint8_t)(0x88))
/** Previously addressed with General Call;
 * Data has been received and ACK has been return */
#define I2C_I2STAT_S_RX_PRE_GENCALL_DAT_ACK		((uint8_t)(0x90))
/** Previously addressed with General Call;
 * Data has been received and NOT ACK has been return */
#define I2C_I2STAT_S_RX_PRE_GENCALL_DAT_NACK	((uint8_t)(0x98))
/** A STOP condition or repeated START condition has
 * been received while still addressed as SLV/REC
 * (Slave Receive) or SLV/TRX (Slave Transmit) */
#define I2C_I2STAT_S_RX_STA_STO_SLVREC_SLVTRX	((uint8_t)(0xA0))

/** Slave transmit mode -------------------------------------------- */
/** Own SLA+R has been received, ACK has been returned */
#define I2C_I2STAT_S_TX_SLAR_ACK				((uint8_t)(0xA8))

/** Arbitration lost in SLA+R/W as master */
#define I2C_I2STAT_S_TX_ARB_LOST_M_SLA			((uint8_t)(0xB0))
/** Own SLA+R has been received, ACK has been returned */
//#define I2C_I2STAT_S_TX_SLAR_ACK				((uint8_t)(0xB0))

/** Data has been transmitted, ACK has been received */
#define I2C_I2STAT_S_TX_DAT_ACK					((uint8_t)(0xB8))
/** Data has been transmitted, NACK has been received */
#define I2C_I2STAT_S_TX_DAT_NACK				((uint8_t)(0xC0))
/** Last data byte in I2DAT has been transmitted (AA = 0);
 ACK has been received */
#define I2C_I2STAT_S_TX_LAST_DAT_ACK			((uint8_t)(0xC8))


/*********************************************************************//**
 * I2C monitor control configuration defines
 **********************************************************************/
#define I2C_MONITOR_CFG_SCL_OUTPUT	I2C_I2MMCTRL_ENA_SCL		/**< SCL output enable */
#define I2C_MONITOR_CFG_MATCHALL	I2C_I2MMCTRL_MATCH_ALL		/**< Select interrupt register match */

#define PARAM_I2C_MONITOR_CFG(n) ((n==I2C_MONITOR_CFG_SCL_OUTPUT) || (I2C_MONITOR_CFG_MATCHALL))

/**
 * @}
 */

/**
 * @}
 */

/************************** GLOBAL/PUBLIC FUNCTIONS *************************/
/** @addtogroup PUBLIC_FUNCTION_PROTOTYPES
 * @{
 */

/** @defgroup I2C_FUNCTIONS
 * @{
 */
void I2C_SetClock (I2C_TypeDef *I2Cx, uint32_t target_clock);
void I2C_PinConfig(I2C_TypeDef *I2Cx, I2C_PinCFG_Type *I2CPinCfg);
void I2C_PinConfigStructInit(I2C_TypeDef *I2Cx, I2C_PinCFG_Type *I2C_PinInitStruct);
void I2C_DeInit(I2C_TypeDef* I2Cx);
void I2C_Init(I2C_TypeDef *I2Cx, uint32_t clockrate);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

IntStatus I2C_GetIntStatus(I2C_TypeDef* I2Cx);
void I2C_SendStart(I2C_TypeDef *I2Cx);
void I2C_SendStop(I2C_TypeDef *I2Cx);
void I2C_SendSlaveAddr(I2C_TypeDef *I2Cx, uint8_t SlaveAddr_7bit, uint8_t RWDirection);
void I2C_AssertAckCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_SendData(I2C_TypeDef *I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef *I2Cx, FunctionalState ReceiveNext);
void I2C_SetOwnSlaveAddr(I2C_TypeDef *I2Cx, I2C_OWNSLAVEADDR_CFG_Type *OwnSlaveAddrConfigStruct);
uint8_t I2C_GetLastStatusCode(I2C_TypeDef* I2Cx);

void I2C_MonitorModeConfig(I2C_TypeDef *I2Cx, uint32_t MonitorCfgType, FunctionalState NewState);
void I2C_MonitorModeCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
uint8_t I2C_MonitorGetDatabuffer(I2C_TypeDef *I2Cx);
/**
 * @}
 */

/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_I2C_H_ */
