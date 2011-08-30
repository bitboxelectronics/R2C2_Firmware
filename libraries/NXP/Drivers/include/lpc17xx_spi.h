/***********************************************************************//**
 * @file	: lpc17xx_spi.h
 * @brief	: Contains all macro definitions and function prototypes
 * 				support for SPI firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 3. April. 2009
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

#ifndef LPC17XX_SPI_H_
#define LPC17XX_SPI_H_

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


/** @defgroup SPI_PINSEL
 * @{
 */
/** SPI function pin selection group 0 */
#define SPI_PINSEL_SCK_P0_15	((uint32_t)(15))
#define SPI_PINFUNC_SCK_P0_15	((uint32_t)(3))

#define SPI_PINSEL_SSEL_P0_16	((uint32_t)(16))
#define SPI_PINFUNC_SSEL_P0_16	((uint32_t)(3))

#define SPI_PINSEL_MISO_P0_17	((uint32_t)(17))
#define SPI_PINFUNC_MISO_P0_17	((uint32_t)(3))

#define SPI_PINSEL_MOSI_P0_18	((uint32_t)(18))
#define SPI_PINFUNC_MOSI_P0_18	((uint32_t)(3))
/**
 * @}
 */


/*********************************************************************//**
 * Macro defines for SPI Control Register
 **********************************************************************/
/** @defgroup SPI_REGISTER_BIT_DEFINITION
 * @{
 */
/** Bit enable, the SPI controller sends and receives the number
 * of bits selected by bits 11:8 */
#define SPI_SPCR_BIT_EN			((uint32_t)(1<<2))
/** Clock phase control bit */
#define SPI_SPCR_CPHA_SECOND	((uint32_t)(1<<3))
/** Clock polarity control bit */
#define SPI_SPCR_CPOL_LOW 		((uint32_t)(1<<4))
/** SPI master mode enable */
#define SPI_SPCR_MSTR		 	((uint32_t)(1<<5))
/** LSB enable bit */
#define SPI_SPCR_LSBF			((uint32_t)(1<<6))
/** SPI interrupt enable bit */
#define SPI_SPCR_SPIE			((uint32_t)(1<<7))
/**  When bit 2 of this register is 1, this field controls the
number of bits per transfer */
#define SPI_SPCR_BITS(n)		((n==0) ? ((uint32_t)0) : ((uint32_t)((n&0x0F)<<8)))
/** SPI Control bit mask */
#define SPI_SPCR_BITMASK		((uint32_t)(0xFFC))


/*********************************************************************//**
 * Macro defines for  SPI Status Register
 **********************************************************************/
/** Slave abort */
#define SPI_SPSR_ABRT		((uint32_t)(1<<3))
/** Mode fault */
#define SPI_SPSR_MODF		((uint32_t)(1<<4))
/** Read overrun */
#define SPI_SPSR_ROVR		((uint32_t)(1<<5))
/** Write collision */
#define SPI_SPSR_WCOL		((uint32_t)(1<<6))
/** SPI transfer complete flag */
#define SPI_SPSR_SPIF 		((uint32_t)(1<<7))
/** SPI Status bit mask */
#define SPI_SPSR_BITMASK	((uint32_t)(0xF8))


/*********************************************************************//**
 * Macro defines for SPI Data Register
 **********************************************************************/
/** SPI Data low bit-mask */
#define SPI_SPDR_LO_MASK	((uint32_t)(0xFF))
/** SPI Data high bit-mask */
#define SPI_SPDR_HI_MASK	((uint32_t)(0xFF00))
/** SPI Data bit-mask */
#define SPI_SPDR_BITMASK	((uint32_t)(0xFFFF))


/*********************************************************************//**
 * Macro defines for SPI Clock Counter Register
 **********************************************************************/
/** SPI clock counter setting */
#define SPI_SPCCR_COUNTER(n) 	((uint32_t)(n&0xFF))
/** SPI clock counter bit-mask */
#define SPI_SPCCR_BITMASK		((uint32_t)(0xFF))


/***********************************************************************
 * Macro defines for SPI Test Control Register
 **********************************************************************/
/** SPI Test bit */
#define SPI_SPTCR_TEST_MASK	((uint32_t)(0xFE))
/** SPI Test register bit mask */
#define SPI_SPTCR_BITMASK	((uint32_t)(0xFE))



/*********************************************************************//**
 * Macro defines for SPI Test Status Register
 **********************************************************************/
/** Slave abort */
#define SPI_SPTSR_ABRT		((uint32_t)(1<<3))
/** Mode fault */
#define SPI_SPTSR_MODF		((uint32_t)(1<<4))
/** Read overrun */
#define SPI_SPTSR_ROVR		((uint32_t)(1<<5))
/** Write collision */
#define SPI_SPTSR_WCOL		((uint32_t)(1<<6))
/** SPI transfer complete flag */
#define SPI_SPTSR_SPIF 		((uint32_t)(1<<7))
/** SPI Status bit mask */
#define SPI_SPTSR_MASKBIT	((uint32_t)(0xF8))



/*********************************************************************//**
 * Macro defines for SPI Interrupt Register
 **********************************************************************/
/** SPI interrupt flag */
#define SPI_SPINT_INTFLAG 	((uint32_t)(1<<0))
/** SPI interrupt register bit mask */
#define SPI_SPINT_BITMASK 	((uint32_t)(0x01))

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

/** @defgroup SPI_TYPES
 * @{
 */
/** SPI configuration structure */
typedef struct {
	uint32_t Databit; 		/** Databit number, should be SPI_DATABIT_x,
							where x is in range from 8 - 16 */
	uint32_t CPHA;			/** Clock phase, should be:
							- SPI_CPHA_FIRST: first clock edge
							- SPI_CPHA_SECOND: second clock edge */
	uint32_t CPOL;			/** Clock polarity, should be:
							- SPI_CPOL_HI: high level
							- SPI_CPOL_LO: low level */
	uint32_t Mode;			/** SPI mode, should be:
							- SPI_MASTER_MODE: Master mode
							- SPI_SLAVE_MODE: Slave mode */
	uint32_t DataOrder;		/** Data order, should be:
							- SPI_DATA_MSB_FIRST: MSB first
							- SPI_DATA_LSB_FIRST: LSB first */
	uint32_t ClockRate;		/** Clock rate,in Hz, should not exceed
							(SPI peripheral clock)/8 */
} SPI_CFG_Type;

/** SPI pin configuration structure */
typedef struct {
	uint8_t SCK_Pin;	/** SCK Pin configuration, should be:
						- SPI_SCK_P0_15: SCK pin is on P0.15 */
	uint8_t SSEL_Pin;	/** SSEL Pin configuration, should be:
						- SPI_SSEL_P0_16: SSEL pin is on P0.16 */
	uint8_t MISO_Pin;	/** SCK Pin configuration, should be:
						- SPI_MISO_P0_17: MISO pin is on P0.17 */
	uint8_t MOSI_Pin;	/** SCK Pin configuration, should be:
						- SPI_MOSI_P0_18: MOSI pin is on P0.18 */
} SPI_PinCFG_Type;

/**
 * @}
 */

/**
 * @}
 */


/*************************** GLOBAL/PUBLIC MACROS ***************************/
/** @addtogroup PUBLIC_MACROS
 * @{
 */

/** @defgroup SPI_MACROS
 * @{
 */
/** Macro to determine if it is valid SPI port number */
#define PARAM_SPIx(n)	(((uint32_t *)n)==((uint32_t *)SPI))


/** SPI function pin selection defines */
#define SPI_SCK_P0_15			((uint8_t)(0))
#define SPI_SSEL_P0_16			((uint8_t)(0))
#define SPI_MISO_P0_17			((uint8_t)(0))
#define SPI_MOSI_P0_18			((uint8_t)(0))

/** Macro to check SPI pin configuration */
#define PARAM_SPI_SCK(n) ((n==SPI_SCK_P0_15))
#define PARAM_SPI_SSEL(n) ((n==SPI_SSEL_P0_16))
#define PARAM_SPI_MISO(n)	((n==SPI_MISO_P0_17))
#define PARAM_SPI_MOSI(n)	((n==SPI_MOSI_P0_18))


/*********************************************************************//**
 * SPI configuration parameter defines
 **********************************************************************/
/** Clock phase control bit */
#define SPI_CPHA_FIRST			((uint32_t)(0))
#define SPI_CPHA_SECOND			SPI_SPCR_CPHA_SECOND
#define PARAM_SPI_CPHA(n) 	((n==SPI_CPHA_FIRST) || (n==SPI_CPHA_SECOND))

/** Clock polarity control bit */
#define SPI_CPOL_HI				((uint32_t)(0))
#define SPI_CPOL_LO				SPI_SPCR_CPOL_LOW
#define PARAM_SPI_CPOL(n)	((n==SPI_CPOL_HI) || (n==SPI_CPOL_LO))

/** SPI master mode enable */
#define SPI_SLAVE_MODE			((uint32_t)(0))
#define SPI_MASTER_MODE			SPI_SPCR_MSTR
#define PARAM_SPI_MODE(n)	((n==SPI_SLAVE_MODE) || (n==SPI_MASTER_MODE))

/** LSB enable bit */
#define SPI_DATA_MSB_FIRST		((uint32_t)(0))
#define SPI_DATA_LSB_FIRST		SPI_SPCR_LSBF
#define PARAM_SPI_DATA_ORDER(n) ((n==SPI_DATA_MSB_FIRST) || (n==SPI_DATA_LSB_FIRST))

/** SPI data bit number defines */
#define SPI_DATABIT_16		SPI_SPCR_BITS(0)		/*!< Databit number = 16 */
#define SPI_DATABIT_8		SPI_SPCR_BITS(0x08) 	/*!< Databit number = 8 */
#define SPI_DATABIT_9		SPI_SPCR_BITS(0x09) 	/*!< Databit number = 9 */
#define SPI_DATABIT_10		SPI_SPCR_BITS(0x0A) 	/*!< Databit number = 10 */
#define SPI_DATABIT_11		SPI_SPCR_BITS(0x0B) 	/*!< Databit number = 11 */
#define SPI_DATABIT_12		SPI_SPCR_BITS(0x0C) 	/*!< Databit number = 12 */
#define SPI_DATABIT_13		SPI_SPCR_BITS(0x0D) 	/*!< Databit number = 13 */
#define SPI_DATABIT_14		SPI_SPCR_BITS(0x0E) 	/*!< Databit number = 14 */
#define SPI_DATABIT_15		SPI_SPCR_BITS(0x0F) 	/*!< Databit number = 15 */
#define PARAM_SPI_DATABIT(n)	((n==SPI_DATABIT_16) || (n==SPI_DATABIT_8) \
							|| (n==SPI_DATABIT_9) || (n==SPI_DATABIT_10) \
							|| (n==SPI_DATABIT_11) || (n==SPI_DATABIT_12) \
							|| (n==SPI_DATABIT_13) || (n==SPI_DATABIT_14) \
							|| (n==SPI_DATABIT_15))


/*********************************************************************//**
 * SPI Status Flag defines
 **********************************************************************/
/** Slave abort */
#define SPI_STAT_ABRT		SPI_SPSR_ABRT
/** Mode fault */
#define SPI_STAT_MODF		SPI_SPSR_MODF
/** Read overrun */
#define SPI_STAT_ROVR		SPI_SPSR_ROVR
/** Write collision */
#define SPI_STAT_WCOL		SPI_SPSR_WCOL
/** SPI transfer complete flag */
#define SPI_STAT_SPIF		SPI_SPSR_SPIF
#define PARAM_SPI_STAT(n)	((n==SPI_STAT_ABRT) || (n==SPI_STAT_MODF) \
						|| (n==SPI_STAT_ROVR) || (n==SPI_STAT_WCOL) \
						|| (n==SPI_STAT_SPIF))
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

/** @defgroup SPI_FUNCTIONS
 * @{
 */
Status SPI_SetClock (SPI_TypeDef *SPIx, uint32_t target_clock);
void SPI_PinConfig(SPI_TypeDef *SPIx, SPI_PinCFG_Type *SPIPinCfg, int32_t spiMode);
void SPI_PinConfigStructInit(SPI_PinCFG_Type *SPI_PinInitStruct);
void SPI_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef *SPIx, SPI_CFG_Type *SPI_ConfigStruct);
void SPI_ConfigStructInit(SPI_CFG_Type *SPI_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx);
void SPI_IntCmd(SPI_TypeDef *SPIx, FunctionalState NewState);
IntStatus SPI_GetIntStatus (SPI_TypeDef *SPIx);
void SPI_ClearIntPending(SPI_TypeDef *SPIx);
uint32_t SPI_GetStatus(SPI_TypeDef* SPIx);
FlagStatus SPI_CheckStatus (uint32_t inputSPIStatus,  uint8_t SPIStatus);
/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif /* LPC17XX_SPI_H_ */
