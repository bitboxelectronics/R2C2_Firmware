/***********************************************************************//**
 * @file	: lpc17xx_ssp.h
 * @purpose	: Contains all macro definitions and function prototypes
 * 				support for SSP firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 9. April. 2009
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

#ifndef LPC17XX_SSP_H_
#define LPC17XX_SSP_H_

#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/****************************** PRIVATE MACROS ******************************/

/* Pin Configuration selection must be defined in structure following:
 * - Port Number,
 * - Pin Number,
 * - Function Number,
 * - Pin Mode,
 * - Open Drain
 */

/** SSP0 function pin selection group 0 */
#define SSP0_PINSEL_SCK_P0_15	{0, 15, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP0_PINSEL_SSEL_P0_16	{0, 16, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP0_PINSEL_MISO_P0_17	{0, 17, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP0_PINSEL_MOSI_P0_18	{0, 18, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}


/** SSP0 function pin selection group 1 */
#define SSP0_PINSEL_SCK_P1_20	{1, 20, 3, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP0_PINSEL_SSEL_P1_21	{1, 21, 3, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP0_PINSEL_MISO_P1_23	{1, 23, 3, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP0_PINSEL_MOSI_P1_24	{1, 24, 3, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}


/** SSP1 function pin selection group 0 */
#define SSP1_PINSEL_SCK_P0_7	{0, 7, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP1_PINSEL_SSEL_P0_6	{0, 6, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP1_PINSEL_MISO_P0_8	{0, 8, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}

#define SSP1_PINSEL_MOSI_P0_9	{0, 9, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}


/** SSP1 function pin selection group 1 */
#define SSP1_PINSEL_SCK_P1_31	{1, 31, 2, \
								PINSEL_PINMODE_PULLUP, \
								PINSEL_PINMODE_NORMAL}


/* Max number of pin on each pin function */
#define SSP0_MAX_SCK_PIN	(2)
#define SSP0_MAX_SSEL_PIN	(2)
#define SSP0_MAX_MISO_PIN	(2)
#define SSP0_MAX_MOSI_PIN	(2)

#define SSP1_MAX_SCK_PIN	(2)
#define SSP1_MAX_SSEL_PIN	(1)
#define SSP1_MAX_MISO_PIN	(1)
#define SSP1_MAX_MOSI_PIN	(1)

/*********************************************************************//**
 * Macro defines for CR0 register
 **********************************************************************/
/** SSP data size select, must be 4 bits to 16 bits */
#define SSP_CR0_DSS(n)   		((uint32_t)((n-1)&0xF))
/** SSP control 0 Motorola SPI mode */
#define SSP_CR0_FRF_SPI  		((uint32_t)(0<<4))
/** SSP control 0 TI synchronous serial mode */
#define SSP_CR0_FRF_TI   		((uint32_t)(1<<4))
/** SSP control 0 National Micro-wire mode */
#define SSP_CR0_FRF_MICROWIRE  	((uint32_t)(2<<4))
/** SPI clock polarity bit (used in SPI mode only), (1) = maintains the
   bus clock high between frames, (0) = low */
#define SSP_CR0_CPOL_HI		((uint32_t)(1<<6))
/** SPI clock out phase bit (used in SPI mode only), (1) = captures data
   on the second clock transition of the frame, (0) = first */
#define SSP_CR0_CPHA_SECOND	((uint32_t)(1<<7))
/** SSP serial clock rate value load macro, divider rate is
   PERIPH_CLK / (cpsr * (SCR + 1)) */
#define SSP_CR0_SCR(n)   	((uint32_t)((n&0xFF)<<8))
/** SSP CR0 bit mask */
#define SSP_CR0_BITMASK		((uint32_t)(0xFFFF))


/*********************************************************************//**
 * Macro defines for CR1 register
 **********************************************************************/
/** SSP control 1 loopback mode enable bit */
#define SSP_CR1_LBM_EN		((uint32_t)(1<<0))
/** SSP control 1 enable bit */
#define SSP_CR1_SSP_EN		((uint32_t)(1<<1))
/** SSP control 1 slave enable */
#define SSP_CR1_SLAVE_EN	((uint32_t)(1<<2))
/** SSP control 1 slave out disable bit, disables transmit line in slave
   mode */
#define SSP_CR1_SO_DISABLE	((uint32_t)(1<<3))
/** SSP CR1 bit mask */
#define SSP_CR1_BITMASK		((uint32_t)(0x0F))


/*********************************************************************//**
 * Macro defines for DR register
 **********************************************************************/
/** SSP data bit mask */
#define SSP_DR_BITMASK(n)   ((n)&0xFFFF)

/*********************************************************************//**
 * Macro defines for SR register
 **********************************************************************/
/** SSP status TX FIFO Empty bit */
#define SSP_SR_TFE      ((uint32_t)(1<<0))
/** SSP status TX FIFO not full bit */
#define SSP_SR_TNF      ((uint32_t)(1<<1))
/** SSP status RX FIFO not empty bit */
#define SSP_SR_RNE      ((uint32_t)(1<<2))
/** SSP status RX FIFO full bit */
#define SSP_SR_RFF      ((uint32_t)(1<<3))
/** SSP status SSP Busy bit */
#define SSP_SR_BSY      ((uint32_t)(1<<4))
/** SSP SR bit mask */
#define SSP_SR_BITMASK	((uint32_t)(0x1F))


/*********************************************************************//**
 * Macro defines for CPSR register
 **********************************************************************/
/** SSP clock prescaler */
#define SSP_CPSR_CPDVSR(n) 	((uint32_t)(n&0xFF))
/** SSP CPSR bit mask */
#define SSP_CPSR_BITMASK	((uint32_t)(0xFF))


/*********************************************************************//**
 * Macro define for (IMSC) Interrupt Mask Set/Clear registers
 **********************************************************************/
/** Receive Overrun */
#define SSP_IMSC_ROR	((uint32_t)(1<<0))
/** Receive TimeOut */
#define SSP_IMSC_RT		((uint32_t)(1<<1))
/** Rx FIFO is at least half full */
#define SSP_IMSC_RX		((uint32_t)(1<<2))
/** Tx FIFO is at least half empty */
#define SSP_IMSC_TX		((uint32_t)(1<<3))
/** IMSC bit mask */
#define SSP_IMSC_BITMASK	((uint32_t)(0x0F))

/*********************************************************************//**
 * Macro define for (RIS) Raw Interrupt Status registers
 **********************************************************************/
/** Receive Overrun */
#define SSP_RIS_ROR		((uint32_t)(1<<0))
/** Receive TimeOut */
#define SSP_RIS_RT		((uint32_t)(1<<1))
/** Rx FIFO is at least half full */
#define SSP_RIS_RX		((uint32_t)(1<<2))
/** Tx FIFO is at least half empty */
#define SSP_RIS_TX		((uint32_t)(1<<3))
/** RIS bit mask */
#define SSP_RIS_BITMASK	((uint32_t)(0x0F))


/*********************************************************************//**
 * Macro define for (MIS) Masked Interrupt Status registers
 **********************************************************************/
/** Receive Overrun */
#define SSP_MIS_ROR		((uint32_t)(1<<0))
/** Receive TimeOut */
#define SSP_MIS_RT		((uint32_t)(1<<1))
/** Rx FIFO is at least half full */
#define SSP_MIS_RX		((uint32_t)(1<<2))
/** Tx FIFO is at least half empty */
#define SSP_MIS_TX		((uint32_t)(1<<3))
/** MIS bit mask */
#define SSP_MIS_BITMASK	((uint32_t)(0x0F))


/*********************************************************************//**
 * Macro define for (ICR) Interrupt Clear registers
 **********************************************************************/
/** Writing a 1 to this bit clears the "frame was received when
 * RxFIFO was full" interrupt */
#define SSP_ICR_ROR		((uint32_t)(1<<0))
/** Writing a 1 to this bit clears the "Rx FIFO was not empty and
 * has not been read for a timeout period" interrupt */
#define SSP_ICR_RT		((uint32_t)(1<<1))
/** ICR bit mask */
#define SSP_ICR_BITMASK	((uint32_t)(0x03))


/*********************************************************************//**
 * Macro defines for DMACR register
 **********************************************************************/
/** SSP bit for enabling RX DMA */
#define SSP_DMA_RXDMA_EN  	((uint32_t)(1<<0))
/** SSP bit for enabling TX DMA */
#define SSP_DMA_TXDMA_EN  	((uint32_t)(1<<1))
/** DMACR	bit mask */
#define SSP_DMA_BITMASK		((uint32_t)(0x03))



/**************************** GLOBAL/PUBLIC TYPES ***************************/
/** SSP configuration structure */
typedef struct {
	uint32_t Databit; 		/** Databit number, should be SSP_DATABIT_x,
							where x is in range from 4 - 16 */
	uint32_t CPHA;			/** Clock phase, should be:
							- SSP_CPHA_FIRST: first clock edge
							- SSP_CPHA_SECOND: second clock edge */
	uint32_t CPOL;			/** Clock polarity, should be:
							- SSP_CPOL_HI: high level
							- SSP_CPOL_LO: low level */
	uint32_t Mode;			/** SSP mode, should be:
							- SSP_MASTER_MODE: Master mode
							- SSP_SLAVE_MODE: Slave mode */
	uint32_t FrameFormat;	/** Frame Format:
							- SSP_FRAME_SPI: Motorola SPI frame format
							- SSP_FRAME_TI: TI frame format
							- SSP_FRAME_MICROWIRE: National Microwire frame format */
	uint32_t ClockRate;		/** Clock rate,in Hz */
} SSP_CFG_Type;

/** SSP pin configuration structure */
typedef struct {
	uint8_t	SCK_Pin;	/** SCK Pin configuration, should be:
						- SSP0_SCK_P0_15: SCK0 pin is on P0.15 (SSP0)
						- SSP0_SCK_P1_20: SCK0 pin is on P1.21 (SSP0)
						- SSP1_SCK_P0_7: SCK1 pin is on P0.7 (SSP1)
						- SSP1_SCK_P1_31: SCK1 pin is on P1.31 (SSP1) */
	uint8_t	SSEL_Pin;	/** SSEL Pin configuration, should be:
						- SSP0_SSEL_P0_16: SSEL0 pin is on P0.16 (SSP0)
						- SSP0_SSEL_P1_21: SSEL0 pin is on P1.21 (SSP0)
						- SSP1_SSEL_P0_6: SSEL1 pin is on P0.6 (SSP1) */
	uint8_t	MISO_Pin;	/** SCK Pin configuration, should be:
						- SSP0_MISO_P0_17: MISO0 pin is on P0.17 (SSP0)
						- SSP0_MISO_P1_23: MISO0 pin is on P1.23 (SSP0)
						- SSP1_MISO_P0_8: MISO1 pin is on P0.8 (SSP1) */
	uint8_t	MOSI_Pin;	/** SCK Pin configuration, should be:
						- SSP0_MOSI_P0_18: MOSI0 pin is on P0.18 (SSP0)
						- SSP0_MOSI_P1_24: MOSI0 pin is on P1.24 (SSP0)
						- SSP1_MOSI_P0_9: MOSI1 pin is on P0.9 (SSP1) */
	uint8_t SSPMode;	/** SSP mode, should be:
							- SSP_MASTER_MODE: Master mode
							- SSP_SLAVE_MODE: Slave mode
							*/
	uint8_t CSPinConfig;	/** CS Pin configuration in master mode,
							should be:
							- SSP_CS_DEFAULT: default CS function for SSP peripheral.
							- SSP_CS_GPIO: GPIO function.
							*/
	uint8_t Reserved[2];	/** Reserved */
} SSP_PinCFG_Type;


/*************************** GLOBAL/PUBLIC MACROS ***************************/

/** Macro to determine if it is valid SSP port number */
#define PARAM_SSPx(n)	((((uint32_t *)n)==((uint32_t *)SSP0)) \
						|| (((uint32_t *)n)==((uint32_t *)SSP1)))


/** SSP0 function pin selection defines */
#define SSP0_SCK_P0_15	((uint8_t)(0))
#define SSP0_SSEL_P0_16	((uint8_t)(0))
#define SSP0_MISO_P0_17	((uint8_t)(0))
#define SSP0_MOSI_P0_18	((uint8_t)(0))



#define SSP0_SCK_P1_20	((uint8_t)(1))
#define SSP0_SSEL_P1_21	((uint8_t)(1))
#define SSP0_MISO_P1_23	((uint8_t)(1))
#define SSP0_MOSI_P1_24	((uint8_t)(1))


/** SSP1 function pin selection defines */
#define SSP1_SCK_P0_7	((uint8_t)(0))
#define SSP1_SSEL_P0_6	((uint8_t)(0))
#define SSP1_MISO_P0_8	((uint8_t)(0))
#define SSP1_MOSI_P0_9	((uint8_t)(0))


#define SSP1_SCK_P1_31	((uint8_t)(1))


/** CS function configuration on master mode */
#define SSP_CS_DEFAULT	((uint8_t)(0))
#define SSP_CS_GPIO		((uint8_t)(1))

#define PARAM_SSP_CS_FUNC(n)	((n==SSP_CS_DEFAULT) || (n==SSP_CS_GPIO))


/** Macro to check SSP0 pin configuration */
#define PARAM_SSP0_SCK(n) 	((n==SSP0_SCK_P0_15) || (n==SSP0_SCK_P1_20))
#define PARAM_SSP0_SSEL(n) 	((n==SSP0_SSEL_P0_16) || (n==SSP0_SSEL_P1_21))
#define PARAM_SSP0_MISO(n)	((n==SSP0_MISO_P0_17) || (n==SSP0_MISO_P1_23))
#define PARAM_SSP0_MOSI(n)	((n==SSP0_MOSI_P0_18) || (n==SSP0_MOSI_P1_24))

/** Macro to check SSP1 pin configuration */
#define PARAM_SSP1_SCK(n) ((n==SSP1_SCK_P0_7) || (n==SSP1_SCK_P1_31))
#define PARAM_SSP1_SSEL(n) ((n==SSP1_SSEL_P0_6))
#define PARAM_SSP1_MISO(n)	((n==SSP1_MISO_P0_8))
#define PARAM_SSP1_MOSI(n)	((n==SSP1_MOSI_P0_9))


/*********************************************************************//**
 * SSP configuration parameter defines
 **********************************************************************/
/** Clock phase control bit */
#define SSP_CPHA_FIRST			((uint32_t)(0))
#define SSP_CPHA_SECOND			SSP_CR0_CPHA_SECOND
#define PARAM_SSP_CPHA(n) 		((n==SSP_CPHA_FIRST) || (n==SSP_CPHA_SECOND))

/** Clock polarity control bit */
#define SSP_CPOL_HI				((uint32_t)(0))
#define SSP_CPOL_LO				SSP_CR0_CPOL_LOW
#define PARAM_SSP_CPOL(n)		((n==SSP_CPOL_HI) || (n==SSP_CPOL_LO))

/** SSP master mode enable */
#define SSP_SLAVE_MODE			SSP_CR1_SLAVE_EN
#define SSP_MASTER_MODE			((uint32_t)(0))
#define PARAM_SSP_MODE(n)		((n==SSP_SLAVE_MODE) || (n==SSP_MASTER_MODE))

/** SSP data bit number defines */
#define SSP_DATABIT_4		SSP_CR0_DSS(4) 			/*!< Databit number = 4 */
#define SSP_DATABIT_5		SSP_CR0_DSS(5) 			/*!< Databit number = 5 */
#define SSP_DATABIT_6		SSP_CR0_DSS(6) 			/*!< Databit number = 6 */
#define SSP_DATABIT_7		SSP_CR0_DSS(7) 			/*!< Databit number = 7 */
#define SSP_DATABIT_8		SSP_CR0_DSS(8) 			/*!< Databit number = 8 */
#define SSP_DATABIT_9		SSP_CR0_DSS(9) 			/*!< Databit number = 9 */
#define SSP_DATABIT_10		SSP_CR0_DSS(10) 		/*!< Databit number = 10 */
#define SSP_DATABIT_11		SSP_CR0_DSS(11) 		/*!< Databit number = 11 */
#define SSP_DATABIT_12		SSP_CR0_DSS(12) 		/*!< Databit number = 12 */
#define SSP_DATABIT_13		SSP_CR0_DSS(13) 		/*!< Databit number = 13 */
#define SSP_DATABIT_14		SSP_CR0_DSS(14) 		/*!< Databit number = 14 */
#define SSP_DATABIT_15		SSP_CR0_DSS(15) 		/*!< Databit number = 15 */
#define SSP_DATABIT_16		SSP_CR0_DSS(16) 		/*!< Databit number = 16 */
#define PARAM_SSP_DATABIT(n) 	((n==SSP_DATABIT_4) || (n==SSP_DATABIT_5) \
							|| (n==SSP_DATABIT_6) || (n==SSP_DATABIT_16) \
							|| (n==SSP_DATABIT_7) || (n==SSP_DATABIT_8) \
							|| (n==SSP_DATABIT_9) || (n==SSP_DATABIT_10) \
							|| (n==SSP_DATABIT_11) || (n==SSP_DATABIT_12) \
							|| (n==SSP_DATABIT_13) || (n==SSP_DATABIT_14) \
							|| (n==SSP_DATABIT_15))

/** SSP Frame Format definition */
/** Motorola SPI mode */
#define SSP_FRAME_SPI		SSP_CR0_FRF_SPI
/** TI synchronous serial mode */
#define SSP_FRAME_TI		SSP_CR0_FRF_TI
/** National Micro-wire mode */
#define SSP_FRAME_MICROWIRE	SSP_CR0_FRF_MICROWIRE

#define PARAM_SSP_FRAME(n) ((n==SSP_FRAME_SPI) || (n==SSP_FRAME_TI) || (n==SSP_FRAME_MICROWIRE))


/*********************************************************************//**
 * SSP Status defines
 **********************************************************************/
/** SSP status TX FIFO Empty bit */
#define SSP_STAT_TXFIFO_EMPTY		SSP_SR_TFE
/** SSP status TX FIFO not full bit */
#define SSP_STAT_TXFIFO_NOTFULL		SSP_SR_TNF
/** SSP status RX FIFO not empty bit */
#define SSP_STAT_RXFIFO_NOTEMPTY	SSP_SR_RNE
/** SSP status RX FIFO full bit */
#define SSP_STAT_RXFIFO_FULL		SSP_SR_RFF
/** SSP status SSP Busy bit */
#define SSP_STAT_BUSY				SSP_SR_BSY

#define PARAM_SSP_STAT(n) ((n==SSP_STAT_TXFIFO_EMPTY) || (n==SSP_STAT_TXFIFO_NOTFULL) \
						|| (n==SSP_STAT_RXFIFO_NOTEMPTY) || (n==SSP_STAT_RXFIFO_FULL) \
						|| (n==SSP_STAT_BUSY))


/*********************************************************************//**
 * SSP Interrupt Configuration defines
 **********************************************************************/
/** Receive Overrun */
#define SSP_INTCFG_ROR		SSP_IMSC_ROR
/** Receive TimeOut */
#define SSP_INTCFG_RT		SSP_IMSC_RT
/** Rx FIFO is at least half full */
#define SSP_INTCFG_RX		SSP_IMSC_RX
/** Tx FIFO is at least half empty */
#define SSP_INTCFG_TX		SSP_IMSC_TX

#define PARAM_SSP_INTCFG(n)	((n==SSP_INTCFG_ROR) || (n==SSP_INTCFG_RT) \
						|| (n==SSP_INTCFG_RX) || (n==SSP_INTCFG_TX))


/*********************************************************************//**
 * SSP Configured Interrupt Status defines
 **********************************************************************/
/** Receive Overrun */
#define SSP_INTSTAT_ROR		SSP_MIS_ROR
/** Receive TimeOut */
#define SSP_INTSTAT_RT		SSP_MIS_RT
/** Rx FIFO is at least half full */
#define SSP_INTSTAT_RX		SSP_MIS_RX
/** Tx FIFO is at least half empty */
#define SSP_INTSTAT_TX		SSP_MIS_TX

#define PARAM_SSP_INTSTAT(n) ((n==SSP_INTSTAT_ROR) || (n==SSP_INTSTAT_RT) \
							|| (n==SSP_INTSTAT_RX) || (n==SSP_INTSTAT_TX))


/*********************************************************************//**
 * SSP Raw Interrupt Status defines
 **********************************************************************/
/** Receive Overrun */
#define SSP_INTSTAT_RAW_ROR		SSP_RIS_ROR
/** Receive TimeOut */
#define SSP_INTSTAT_RAW_RT		SSP_RIS_RT
/** Rx FIFO is at least half full */
#define SSP_INTSTAT_RAW_RX		SSP_RIS_RX
/** Tx FIFO is at least half empty */
#define SSP_INTSTAT_RAW_TX		SSP_RIS_TX

#define PARAM_SSP_INTSTAT_RAW(n)	((n==SSP_INTSTAT_RAW_ROR) || (n==SSP_INTSTAT_RAW_RT) \
								|| (n==SSP_INTSTAT_RAW_RX) || (n==SSP_INTSTAT_RAW_TX))


/*********************************************************************//**
 * SSP Interrupt Clear defines
 **********************************************************************/
/** Writing a 1 to this bit clears the "frame was received when
 * RxFIFO was full" interrupt */
#define SSP_INTCLR_ROR		SSP_ICR_ROR
/** Writing a 1 to this bit clears the "Rx FIFO was not empty and
 * has not been read for a timeout period" interrupt */
#define SSP_INTCLR_RT		SSP_ICR_RT

#define PARAM_SSP_INTCLR(n)	((n==SSP_INTCLR_ROR) || (n==SSP_INTCLR_RT))


/*********************************************************************//**
 * SSP DMA defines
 **********************************************************************/
/** SSP bit for enabling RX DMA */
#define SSP_DMA_TX		SSP_DMA_RXDMA_EN
/** SSP bit for enabling TX DMA */
#define SSP_DMA_RX		SSP_DMA_TXDMA_EN

#define PARAM_SSP_DMA(n)	((n==SSP_DMA_TX) || (n==SSP_DMA_RX))


/************************** GLOBAL/PUBLIC FUNCTIONS *************************/
Status SSP_SetClock (SSP_TypeDef *SSPx, uint32_t target_clock);
void SSP_PinConfig(SSP_TypeDef *SSPx, SSP_PinCFG_Type *SSPPinCfg);
void SSP_PinConfigStructInit(SSP_TypeDef *SSPx, SSP_PinCFG_Type *SSP_PinInitStruct);
void SSP_DeInit(SSP_TypeDef* SSPx);
void SSP_Init(SSP_TypeDef *SSPx, SSP_CFG_Type *SSP_ConfigStruct);
void SSP_ConfigStructInit(SSP_CFG_Type *SSP_InitStruct);
void SSP_Cmd(SSP_TypeDef* SSPx, FunctionalState NewState);
void SSP_LoopBackCmd(SSP_TypeDef* SSPx, FunctionalState NewState);
void SSP_SlaveOutputCmd(SSP_TypeDef* SSPx, FunctionalState NewState);
void SSP_SendData(SSP_TypeDef* SSPx, uint16_t Data);
uint16_t SSP_ReceiveData(SSP_TypeDef* SSPx);
FlagStatus SSP_GetStatus(SSP_TypeDef* SSPx, uint32_t FlagType);
void SSP_IntConfig(SSP_TypeDef *SSPx, uint32_t IntType, FunctionalState NewState);
IntStatus SSP_GetRawIntStatus(SSP_TypeDef *SSPx, uint32_t RawIntType);
IntStatus SSP_GetIntStatus (SSP_TypeDef *SSPx, uint32_t IntType);
void SSP_ClearIntPending(SSP_TypeDef *SSPx, uint32_t IntType);
void SSP_DMACmd(SSP_TypeDef *SSPx, uint32_t DMAMode, FunctionalState NewState);


#ifdef __cplusplus
}
#endif


#endif /* LPC17XX_SSP_H_ */
