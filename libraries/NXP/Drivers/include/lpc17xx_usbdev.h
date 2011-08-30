/***********************************************************************//**
 * @file	: lpc17xx_usbdev.h
 * @brief	: Contains all macro definitions and function prototypes
 * 				support for USB device firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 15. Aug. 2008
 * @author	: LinhNguyen
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

#ifndef LPC17XX_USBDEV_H_
#define LPC17XX_USBDEV_H_


#include "lpc_types.h"
#include "LPC17xx.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */


#ifdef __cplusplus
extern "C" {
#endif


/****************************** PRIVATE MACROS ******************************/
/** @addtogroup PRIVATE_MACROS
 * @{
 */


/* ------------------ USB Device Controller Register Bit definition --------------- */
/** @defgroup USBDEV_REGISTER_BIT_DEFINITIONS
 * @{
 */

/* USB Clock control registers ---------------------------------------------------- */
/* USB Clock Control register */
#define USBDEV_USBClkCtrl_DEV_CLK_EN				((uint32_t)(1<<1))			/**< Device clock enable */
#define USBDEV_USBClkCtrl_PORTSEL_CLK_EN			((uint32_t)(1<<3))			/**< Port select register clock enable */
#define USBDEV_USBClkCtrl_AHB_CLK_EN				((uint32_t)(1<<4))			/**< AHB clock enable */

/* USB Clock Status register */
#define USBDEV_USBClkSt_DEV_CLK_ON					((uint32_t)(1<<1))			/**< Device clock on */
#define USBDEV_USBClkSt_PORTSEL_CLK_ON				((uint32_t)(1<<3))			/**< Port select register clock on */
#define USBDEV_USBClkSt_AHB_CLK_ON					((uint32_t)(1<<4))			/**< AHB clock on */


/* USB Device interrupt registers ---------------------------------------------------- */
/* USB Interrupt Status register */
#define USBDEV_USBIntSt_USB_INT_REQ_LP				((uint32_t)(1<<0))			/**< Low priority interrupt line status */
#define USBDEV_USBIntSt_USB_INT_REQ_HP				((uint32_t)(1<<1)) 			/**< High priority interrupt line status */
#define USBDEV_USBIntSt_USB_INT_REQ_DMA				((uint32_t)(1<<2)) 			/**< DMA interrupt line status */
#define USBDEV_USBIntSt_USB_NEED_CLOCK				((uint32_t)(1<<8)) 			/**< USB need clock indicator */
#define USBDEV_USBIntSt_EN_USB_INTS					((uint32_t)(1<<31)) 		/**< Enable all USB interrupts */

/* USB Device Interrupt Status register */
#define USBDEV_USBDevIntSt_FRAME					((uint32_t)(1<<0)) 			/**< The frame interrupt occurs every 1 ms */
#define USBDEV_USBDevIntSt_EP_FAST					((uint32_t)(1<<1)) 			/**< Fast endpoint interrupt */
#define USBDEV_USBDevIntSt_EP_SLOW					((uint32_t)(1<<2)) 			/**< Slow endpoints interrupt */
#define USBDEV_USBDevIntSt_DEV_STAT					((uint32_t)(1<<3)) 			/**< Set when USB Bus reset, USB suspend change or Connect change event occurs */
#define USBDEV_USBDevIntSt_CCEMTY					((uint32_t)(1<<4)) 			/**< The command code register (USBCmdCode) is empty (New command can be written) */
#define USBDEV_USBDevIntSt_CDFULL					((uint32_t)(1<<5)) 			/**< Command data register (USBCmdData) is full (Data can be read now) */
#define USBDEV_USBDevIntSt_RxENDPKT					((uint32_t)(1<<6)) 			/**< The current packet in the endpoint buffer is transferred to the CPU */
#define USBDEV_USBDevIntSt_TxENDPKT					((uint32_t)(1<<7)) 			/**< The number of data bytes transferred to the endpoint buffer equals the number of
																					bytes programmed in the TxPacket length register (USBTxPLen) */
#define USBDEV_USBDevIntSt_EP_RLZED					((uint32_t)(1<<8)) 			/**< Endpoints realized */
#define USBDEV_USBDevIntSt_ERR_INT					((uint32_t)(1<<9)) 			/**< Error Interrupt */

/* USB Device Interrupt Enable register */
#define USBDEV_USBDevIntEn_FRAME					((uint32_t)(1<<0)) 			/**< The frame interrupt occurs every 1 ms */
#define USBDEV_USBDevIntEn_EP_FAST					((uint32_t)(1<<1)) 			/**< Fast endpoint interrupt */
#define USBDEV_USBDevIntEn_EP_SLOW					((uint32_t)(1<<2)) 			/**< Slow endpoints interrupt */
#define USBDEV_USBDevIntEn_DEV_STAT					((uint32_t)(1<<3)) 			/**< Set when USB Bus reset, USB suspend change or Connect change event occurs */
#define USBDEV_USBDevIntEn_CCEMTY					((uint32_t)(1<<4)) 			/**< The command code register (USBCmdCode) is empty (New command can be written) */
#define USBDEV_USBDevIntEn_CDFULL					((uint32_t)(1<<5)) 			/**< Command data register (USBCmdData) is full (Data can be read now) */
#define USBDEV_USBDevIntEn_RxENDPKT					((uint32_t)(1<<6)) 			/**< The current packet in the endpoint buffer is transferred to the CPU */
#define USBDEV_USBDevIntEn_TxENDPKT					((uint32_t)(1<<7)) 			/**< The number of data bytes transferred to the endpoint buffer equals the number of
																					bytes programmed in the TxPacket length register (USBTxPLen) */
#define USBDEV_USBDevIntEn_EP_RLZED					((uint32_t)(1<<8)) 			/**< Endpoints realized */
#define USBDEV_USBDevIntEn_ERR_INT					((uint32_t)(1<<9)) 			/**< Error Interrupt */
#define USBDEV_USBDevIntEn_BITMASK					((uint32_t)(0x3FF))			/**< USB Device interrupt bit mask */

/* USB Device Interrupt Clear register */
#define USBDEV_USBDevIntClr_FRAME					((uint32_t)(1<<0)) 			/**< The frame interrupt occurs every 1 ms */
#define USBDEV_USBDevIntClr_EP_FAST					((uint32_t)(1<<1)) 			/**< Fast endpoint interrupt */
#define USBDEV_USBDevIntClr_EP_SLOW					((uint32_t)(1<<2)) 			/**< Slow endpoints interrupt */
#define USBDEV_USBDevIntClr_DEV_STAT				((uint32_t)(1<<3)) 			/**< Set when USB Bus reset, USB suspend change or Connect change event occurs */
#define USBDEV_USBDevIntClr_CCEMTY					((uint32_t)(1<<4)) 			/**< The command code register (USBCmdCode) is empty (New command can be written) */
#define USBDEV_USBDevIntClr_CDFULL					((uint32_t)(1<<5)) 			/**< Command data register (USBCmdData) is full (Data can be read now) */
#define USBDEV_USBDevIntClr_RxENDPKT				((uint32_t)(1<<6)) 			/**< The current packet in the endpoint buffer is transferred to the CPU */
#define USBDEV_USBDevIntClr_TxENDPKT				((uint32_t)(1<<7)) 			/**< The number of data bytes transferred to the endpoint buffer equals the number of
																					bytes programmed in the TxPacket length register (USBTxPLen) */
#define USBDEV_USBDevIntClr_EP_RLZED				((uint32_t)(1<<8)) 			/**< Endpoints realized */
#define USBDEV_USBDevIntClr_ERR_INT					((uint32_t)(1<<9)) 			/**< Error Interrupt */

/* USB Device Interrupt Set register */
#define USBDEV_USBDevIntSet_FRAME					((uint32_t)(1<<0)) 			/**< The frame interrupt occurs every 1 ms */
#define USBDEV_USBDevIntSet_EP_FAST					((uint32_t)(1<<1)) 			/**< Fast endpoint interrupt */
#define USBDEV_USBDevIntSet_EP_SLOW					((uint32_t)(1<<2)) 			/**< Slow endpoints interrupt */
#define USBDEV_USBDevIntSet_DEV_STAT				((uint32_t)(1<<3)) 			/**< Set when USB Bus reset, USB suspend change or Connect change event occurs */
#define USBDEV_USBDevIntSet_CCEMTY					((uint32_t)(1<<4)) 			/**< The command code register (USBCmdCode) is empty (New command can be written) */
#define USBDEV_USBDevIntSet_CDFULL					((uint32_t)(1<<5)) 			/**< Command data register (USBCmdData) is full (Data can be read now) */
#define USBDEV_USBDevIntSet_RxENDPKT				((uint32_t)(1<<6)) 			/**< The current packet in the endpoint buffer is transferred to the CPU */
#define USBDEV_USBDevIntSet_TxENDPKT				((uint32_t)(1<<7)) 			/**< The number of data bytes transferred to the endpoint buffer equals the number of
																					bytes programmed in the TxPacket length register (USBTxPLen) */
#define USBDEV_USBDevIntSet_EP_RLZED				((uint32_t)(1<<8)) 			/**< Endpoints realized */
#define USBDEV_USBDevIntSet_ERR_INT					((uint32_t)(1<<9)) 			/**< Error Interrupt */

/* USB Device Interrupt Priority register */
#define USBDEV_USBDevIntPri_FRAME					((uint32_t)(1<<0))			/**< FRAME interrupt is routed to USB_INT_REQ_HP */
#define USBDEV_USBDevIntPri_EP_FAST					((uint32_t)(1<<1))			/**< EP_FAST interrupt is routed to USB_INT_REQ_HP */


/* USB Device Endpoint interrupt registers --------------------------------------------------- */
/* USB Endpoint Interrupt Status register */
#define USBDEV_USBEpIntSt_RX(n)						((uint32_t)(n<<1))			/**< Data Received Interrupt bit for EPn */
#define USBDEV_USBEpIntSt_TX(n)						((uint32_t)((n<<1)+1))		/**< Data Transmitted Interrupt bit for EPn */

/* USB Endpoint Interrupt Enable register */
#define USBDEV_USBEpIntEn_RX(n)						((uint32_t)(n<<1))			/**< Data Received Interrupt bit for EPn */
#define USBDEV_USBEpIntEn_TX(n)						((uint32_t)((n<<1)+1))		/**< Data Transmitted Interrupt bit for EPn */

/* USB Endpoint Interrupt Clear register */
#define USBDEV_USBEpIntClr_RX(n)					((uint32_t)(n<<1))			/**< Data Received Interrupt bit for EPn */
#define USBDEV_USBEpIntClr_TX(n)					((uint32_t)((n<<1)+1))		/**< Data Transmitted Interrupt bit for EPn */

/* USB Endpoint Interrupt Set register */
#define USBDEV_USBEpIntSet_RX(n)					((uint32_t)(n<<1))			/**< Data Received Interrupt bit for EPn */
#define USBDEV_USBEpIntSet_TX(n)					((uint32_t)((n<<1)+1))		/**< Data Transmitted Interrupt bit for EPn */

/* USB Endpoint Priority register
 * 0: The corresponding interrupt is routed to the EP_SLOW bit of USBDevIntSt
 * 1: The corresponding interrupt is routed to the EP_FAST bit of USBDevIntSt */
#define USBDEV_USBEpIntPri_RX(n)					((uint32_t)(n<<1))			/**< Data Received Interrupt bit for EPn */
#define USBDEV_USBEpIntPri_TX(n)					((uint32_t)((n<<1)+1))		/**< Data Transmitted Interrupt bit for EPn */


/* Endpoint realization registers ------------------------------------------------------- */
/* USB Realize Endpoint register
 * Note:
 * On reset, only the control endpoints (EP0 and EP1) are realized */
#define USBDEV_USBReEp_EPn(n)						((uint32_t)(1<<n))			/**< Endpoint EPn is realized */

/* USB Endpoint Index register */
#define USBDEV_USBEpIn_PHY_EP(n)					((uint32_t)(n&0x1F))		/**< Physical endpoint number */

/* USB MaxPacketSize register */
#define USBDEV_USBMaxPSize_MPS(n)					((uint32_t)(n&0x3FF))		/**< The maximum packet size value */


/* USB transfer registers -------------------------------------------------------------- */
/* USB Receive Data register */
/* USB Receive Packet Length register */
#define USBDEV_USBRxPLen_PKT_LNGTH_MASK				((uint32_t)(0x3FF))			/**< The remaining number of bytes to be read from the
																				currently selected endpoint’s buffer */
#define USBDEV_USBRxPLen_DV							((uint32_t)(1<<10))			/**< Data Valid */
#define USBDEV_USBRxPLen_PKT_RDY					((uint32_t)(1<<11))			/**< The PKT_LNGTH field is valid and the packet is
																				ready for reading */
/* USB Transmit Data register */
/* USB Transmit Packet Length register */
#define USBDEV_USBTxPLen_PKT_LNGTH_MASK				((uint32_t)(0x3FF))			/**< The remaining number of bytes to be written to
																				the selected endpoint buffer */
/* USB Control register */
#define USBDEV_USBCtrl_RD_EN						((uint32_t)(1<<0))			/**< Read mode control */
#define USBDEV_USBCtrl_WR_EN						((uint32_t)(1<<1))			/**< Write mode control */
#define USBDEV_USBCtrl_LOG_ENDPOINT(n)				((uint32_t)((n&0F)<<2))		/**< Logical Endpoint number */


/* SIE (Serial Interface Engine) command code registers ----------------------------------------------------------- */
/* USB Command Code register */
#define USBDEV_USBCmdCode_CMD_PHASE(n)				((uint32_t)((n&0xFF)<<8))			/**< The command phase */
#define USBDEV_USBCmdCode_CMD_PHASE_READ			USBDEV_USBCmdCode_CMD_PHASE(0x02)	/**< Read */
#define USBDEV_USBCmdCode_CMD_PHASE_WRITE			USBDEV_USBCmdCode_CMD_PHASE(0x01)	/**< Write */
#define USBDEV_USBCmdCode_CMD_PHASE_COMMAND			USBDEV_USBCmdCode_CMD_PHASE(0x05)	/**< Command */

#define USBDEV_USBCmdCode_CMD_CODE(n)				((uint32_t)((n&0xFF)<<16))			/**< Command Code field */
#define USBDEV_USBCmdCode_CMD_WDATA(n)				((uint32_t)((n&0xFF)<<16))			/**< Write Data field */


/* USB Command Data register */
#define USBDEV_USBCmdData_CMD_RDATA(n)				((uint32_t)(n&0xFF))				/**< Command Read Data */


/* SIE (Serial Interface Engine) command code definitions --------------------------------------------- */
/* Device command */
#define SIECMD_DEV_SET_ADDR							(0xD0) 		/**< Set address command */
#define SIECMD_DEV_CFG_DEV							(0xD8) 		/**< Configure device command */
#define SIECMD_DEV_SET_MODE							(0xF3) 		/**< Set mode command */
#define SIECMD_DEV_RD_FRAME							(0xF5) 		/**< Read current frame number command */
#define SIECMD_DEV_RD_TEST							(0xFD)		/**< Read test register command */
#define SIECMD_DEV_SET_STAT							(0xFE) 		/**< Set device status command */
#define SIECMD_DEV_GET_STAT							(0xFE) 		/**< Get device status command */
#define SIECMD_DEV_GET_ERR_CODE						(0xFF) 		/**< Get error code command */
#define SIECMD_DEV_RD_ERR_STAT						(0xFB) 		/**< Read error status command */
/* EndPoint commands */
#define SIECMD_EP_SEL(n)							(n) 		/**< Select endpoint command */
#define SIECMD_EP_CLRI(n)							(0x40+n) 	/**< Select endpoint/clear interrupt command */
#define SIECMD_EP_SET_STAT(n)						(0x40+n)	/**< Set endpoint status command */
#define SIECMD_EP_CLR_BUF							(0xF2) 		/**< Clear the buffer command */
#define SIECMD_EP_VALID_BUF							(0xFA) 		/**< Validate the buffer command */


/* SIE Command implementation -------------------------------------------------------------- */
/* Set address command */
#define DEV_ADDR(n)									(n&0x7F) 	/**< Device address set by software */
#define DEV_EN										(1<<7) 		/**< Device enable */

/* Configure device command */
#define CONF_DEVICE									(1<<0) 		/**< Device is configured */

/* Set mode command */
#define AP_CLK										(1<<0) 		/**< Always PLL clock */
#define INAK_CI										(1<<1) 		/**< Interrupt on NAK for Control IN endpoint */
#define INAK_CO										(1<<2) 		/**< Interrupt on NAK for Control OUT endpoint */
#define INAK_II										(1<<3) 		/**< Interrupt on NAK for Interrupt IN endpoint */
#define INAK_IO										(1<<4) 		/**< Interrupt on NAK for Interrupt OUT endpoint */
#define INAK_BI										(1<<5) 		/**< Interrupt on NAK for Bulk IN endpoint */
#define INAK_BO										(1<<6) 		/**< Interrupt on NAK for Bulk OUT endpoint */

/* Read current frame number command */

/* Read test register command */

/* Set/Get device status command */
#define CON											(1<<0) 		/**< Indicate the current connect status of the device */
#define CON_CH										(1<<1) 		/**< Connect change */
#define SUS											(1<<2) 		/**< Suspend */
#define SUS_CH										(1<<3) 		/**< Suspend bit change indicator */
#define RST											(1<<4) 		/**< Bus reset bit */

/*
 * Get error code command
 * Error Code descriptions:
 *  - 0000 No Error.
 *  - 0001 PID Encoding Error.
 *  - 0010 Unknown PID.
 *  - 0011 Unexpected Packet - any packet sequence violation from the
 * 		specification.
 *  - 0100 Error in Token CRC.
 *  - 0101 Error in Data CRC.
 *  - 0110 Time Out Error.
 *  - 0111 Babble.
 *  - 1000 Error in End of Packet.
 *  - 1001 Sent/Received NAK.
 *  - 1010 Sent Stall.
 *  - 1011 Buffer Overrun Error.
 *  - 1100 Sent Empty Packet (ISO Endpoints only).
 *  - 1101 Bitstuff Error.
 *  - 1110 Error in Sync.
 *  - 1111 Wrong Toggle Bit in Data PID, ignored data.
 *  - Bit 4 : EA - The Error Active bit will be reset once this register is read.
 */

/* Read error status command */
#define ERR_PID										(1<<0) 		/**< PID encoding error or Unknown PID or Token CRC */
#define ERR_UEPKT									(1<<1) 		/**< Unexpected packet */
#define ERR_DCRC									(1<<2) 		/**< Data CRC error */
#define ERR_TIMEOUT									(1<<3) 		/**< Timeout error */
#define ERR_EOP										(1<<4) 		/**< End of packet error */
#define ERR_B_OVRN									(1<<5) 		/**< Buffer overrun */
#define ERR_BTSTF									(1<<6) 		/**< Bit stuff error */
#define ERR_TGL_ERR									(1<<7) 		/**< Wrong toggle bit in data PID, ignored data */


/* Select Endpoint command */
#define EPSTAT_FE									(1<<0) 		/**< For the F/E bit give the ORed result
																	of B_1_FULL and B_2_FULL bits*/
#define EPSTAT_ST									(1<<1) 		/**< Stalled endpoint indicator*/
#define EPSTAT_STP									(1<<2) 		/**< Setup bit */
#define EPSTAT_PO									(1<<3) 		/**< Packet over-written bit */
#define EPSTAT_EPN									(1<<4) 		/**< EP NAKed bit indicates sending of a NAK */
#define EPSTAT_B1FULL								(1<<5) 		/**< Buffer 1 status: Full */
#define EPSTAT_B2FULL								(1<<6) 		/**< Buffer 2 status: Full */

/* Select Endpoint/Clear Interrupt command */

/* Set Endpoint Status command */
#define EP_ST										(1<<0) 		/**< Stalled endpoint bit */
#define EP_DA										(1<<5) 		/**< Disable endpoint bit */
#define EP_RF_MO									(1<<6) 		/**< Rate feedback mode */
#define EP_CND_ST									(1<<7) 		/**< Conditional stall bit */

/* Clear buffer command */
#define CLR_BUF_PO									(1<<0)		/**< Packet over-written bit */


/* DMA registers --------------------------------------------------------------------------------------- */
/* USB DMA Request Status register */
#define USBDEV_USBDMARSt_EPn(n)						((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))	/**< DMA requested by endpoint n */

/* USB DMA Request Clear register */
#define USBDEV_USBDMARClr_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))	/**< Clear the endpoint n DMA request */

/* USB DMA Request Set register */
#define USBDEV_USBDMARSet_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))	/**< Set the endpoint n DMA request */

/* USB UDCA (USB Device Communication Area) Head register */
#define USBDEV_USBUDCAH_UDCA_ADDR(n)				((uint32_t)(n&0xFFFFFF00))						/**< Start address of the UDCA */

/* USB Endpoint DMA Status register */
#define USBDEV_USBEpDMASt_EPn_DMA_ENABLE(n)			((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))	/**< The DMA for endpoint EPn is enabled */

/* USB Endpoint DMA Enable register */
#define USBDEV_USBEpDMAEn_EPn_DMA_ENABLE(n)			((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))	/**< Enable the DMA operation for endpoint EPn */

/* USB Endpoint DMA Disable register */
#define USBDEV_USBEpDMADis_EPn_DMA_DISABLE(n)		((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))	/**< Disable the DMA operation for endpoint EPn */

/* USB DMA Interrupt Status register */
#define USBDEV_USBDMAIntSt_EOT						((uint32_t)(1<<0))			/**< End of Transfer Interrupt bit */
#define USBDEV_USBDMAIntSt_NDDR						((uint32_t)(1<<1))			/**< New DD Request Interrupt bit */
#define USBDEV_USBDMAIntSt_ERR						((uint32_t)(1<<2))			/**< System Error Interrupt bit */

/* USB DMA Interrupt Enable register */
#define USBDEV_USBDMAIntEn_EOT						((uint32_t)(1<<0))			/**< End of Transfer Interrupt Enable bit */
#define USBDEV_USBDMAIntEn_NDDR						((uint32_t)(1<<1))			/**< New DD Request Interrupt Enable bit */
#define USBDEV_USBDMAIntEn_ERR						((uint32_t)(1<<2))			/**< System Error Interrupt Enable bit */

/* USB End of Transfer Interrupt Status register */
#define USBDEV_USBEoTIntSt_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Endpoint n End of Transfer Interrupt request */

/* USB End of Transfer Interrupt Clear register */
#define USBDEV_USBEoTIntClr_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Clear endpoint n End of Transfer Interrupt request */

/* USB End of Transfer Interrupt Set register */
#define USBDEV_USBEoTIntSet_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Set endpoint n End of Transfer Interrupt request */

/* USB New DD Request Interrupt Status register */
#define USBDEV_USBNDDRIntSt_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Endpoint n new DD interrupt request */

/* USB New DD Request Interrupt Clear register */
#define USBDEV_USBNDDRIntClr_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Clear endpoint n new DD interrupt request */

/* USB New DD Request Interrupt Set register */
#define USBDEV_USBNDDRIntSet_EPn(n)					((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Set endpoint n new DD interrupt request */

/* USB System Error Interrupt Status register */
#define USBDEV_USBSysErrIntSt_EPn(n)				((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Endpoint n System Error Interrupt request */

/* USB System Error Interrupt Clear register */
#define USBDEV_USBSysErrIntClr_EPn(n)				((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Clear endpoint n System Error Interrupt request */

/* USB System Error Interrupt Set register */
#define USBDEV_USBSysErrIntSet_EPn(n)				((uint32_t)(((n>=2)&&(n<=31)) ? (1<<n) : (0)))		/**< Set endpoint n System Error Interrupt request */


/* USB DMA Implementation -------------------------------------------------------------------- */
/* USB RAM Definitions */
#define USB_RAM_ADR     (0x20080000)  						/**< USB RAM Start Address */
#define USB_RAM_SZ      (0x00004000)  						/**< USB RAM Size (16kB) */


/* DMA Endpoint Descriptors */
#define DD_NISO_CNT      	(16)  							/**< Non-Iso EP DMA Descriptor Count (max. 32) */
#define DD_ISO_CNT        	(8)  								/**< Iso EP DMA Descriptor Count (max. 32) */
#define DD_NISO_SZ    		(DD_NISO_CNT * 16)    			/**< Non-Iso DMA Descriptor Size */
#define DD_ISO_SZ     		(DD_ISO_CNT  * 20)    			/**< Iso DMA Descriptor Size */
#define DD_NISO_ADR   		(USB_RAM_ADR + 128)   			/**< Non-Iso DMA Descriptor Address */
#define DD_ISO_ADR    		(DD_NISO_ADR + DD_NISO_SZ) 		/**< Iso DMA Descriptor Address */
#define DD_SZ           	(128 + DD_NISO_SZ + DD_ISO_SZ) 	/**< Descriptor Size */

/* DMA Buffer Memory Definitions */
#define DMA_BUF_ADR   		(USB_RAM_ADR + DD_SZ) 			/**< DMA Buffer Start Address */
#define DMA_BUF_SZ    		(USB_RAM_SZ  - DD_SZ) 			/**< DMA Buffer Size */


/* Other USB Device Hardware Configurations --------------------------------------------------------- */
/* USB CONFIGURATION */
#define USB_EP_NUM          (32)		/**< USB EP number */
#define USB_MAX_PACKET0     (64)		/**< Maximum packet size for EP 0 */
#define MAX_PACKET_SIZE0	(64)		/**< Maximum packet size for EP 0 */

/* Some other useful macros */
#define MIN(x,y)	((x)<(y)?(x):(y))	/**< MIN macro */
#define MAX(x,y)	((x)>(y)?(x):(y))	/**< MAX macro */

/* Extern macro defines for debugging */
#define nextline	/**< Extern macro defines for debugging: Next line debug command */
#define DBG(x)		/**< Extern macro defines for debugging: String debug command */
#define DBC(x)		/**< Extern macro defines for debugging: Number debug command */
#define ASSERT(x)	/**< Extern macro defines for debugging: Assert command */

/**
 * @}
 */


/**
 * @}
 */


/**************************** GLOBAL/PUBLIC TYPES ***************************/

/**
 * @addtogroup PUBLIC_TYPES
 * @{
 */

/** @defgroup USBDEV_TYPES
 * @{
 */


/**
 * @brief Setup Packet Definitions
 */
typedef struct __attribute__ ((__packed__)) {
	uint8_t	bmRequestType;			/**< characteristics of the specific request */
	uint8_t	bRequest;				/**< specific request */
	uint16_t wValue;				/**< request specific parameter */
	uint16_t wIndex;				/**< request specific parameter */
	uint16_t wLength;				/**< length of data transferred in data phase */
} USB_SETUP_PACKET;


/**
 * @brief USB Descriptor Header
 */
typedef struct __attribute__ ((__packed__)) {
	uint8_t	bLength;				/**< descriptor length */
	uint8_t	bDescriptorType;		/**< descriptor type */
} USB_DESC_HEADER;


#if _USB_DMA
/**
 * @brief DMA Descriptor Data Structure
 */
typedef struct __attribute__ ((__packed__)) {
	__IO uint32_t UDCA[USB_EP_NUM];						/**< USB Device Communication Area memory */
	__IO uint32_t DD_NISO_Mem[4*DD_NISO_CNT];           /**< Non-Iso DMA Descriptor Memory */
	__IO uint32_t DD_ISO_Mem [5*DD_ISO_CNT];            /**< Iso DMA Descriptor Memory */
	__IO uint32_t BUF_ADR; 								/**< The start address to buffer the data */
} USB_DMA_T;


/**
 * @brief  USB DMA Descriptor Option Structure
 */
typedef struct __attribute__ ((__packed__)) _USB_DMA_OPTS {
	uint32_t Link   : 1;             /**< Link to existing Descriptors */
	uint32_t IsoEP  : 1;             /**< Isonchronous Endpoint */
	uint32_t ATLE   : 1;             /**< ATLE (Auto Transfer Length Extract) */
	uint32_t Rsrvd  : 5;             /**< Reserved */
	uint32_t LenPos : 8;             /**< Length Position (ATLE) */
} USB_DMA_OPTS;

/**
 * @brief  USB DMA Descriptor Structure
 */

typedef struct __attribute__ ((__packed__)) _USB_DMA_DESCRIPTOR {
  uint32_t 	BufAdr;                     	/**< DMA Buffer Address */
  uint16_t  BufLen;                     	/**< DMA Buffer Length */
  uint16_t  MaxSize;                    	/**< Maximum Packet Size */
  uint32_t 	InfoAdr;                    	/**< Packet Info Memory Address */
  union __attribute__ ((__packed__)) {
	USB_DMA_OPTS Type;						/**< DMA Option in bit-field type value */
	uint32_t Val;							/**< DMA Option value */
  } Cfg;									/**< DMA Configuration */
} USB_DMA_DESCRIPTOR;
#endif /* _USB_DMA */

extern uint8_t ep_configured;

/* USB EndPoint Call Back Function type */
typedef void (USB_EPIntHandler)(uint8_t bEP, uint8_t bEPStatus);

/* USB Device Status Event Call Back Function type */
typedef void (USB_DevIntHandler)(uint8_t bDevStatus);

/* USB Frame Event Call Back Function type */
typedef void (USB_FrameHandler)(uint16_t wFrame);

/* USB DMA EP Event Call Back Function type */
typedef void (USB_DMA_EPIntHandler)(uint8_t bEP, uint32_t wStatus);

/* USB Device Request Handler type */
typedef Bool (USB_HandleRequest)(USB_SETUP_PACKET *pSetup, int32_t *piLen, uint8_t **ppbData);

/**
 * @}
 */

/**
 * @}
 */


/*************************** GLOBAL/PUBLIC MACROS ***************************/

/**
 * @addtogroup PUBLIC_MACROS
 * @{
 */

/** @defgroup USBDEV_PUBLIC_MACROS
 * @{
 */


/* USB Endpoint CallBack Events Type ---------------------------------------------- */
#define USB_EVT_SETUP       (1)   /**< Setup Packet */
#define USB_EVT_OUT         (2)   /**< OUT Packet */
#define USB_EVT_IN          (3)   /**< IN Packet */
#define USB_EVT_OUT_NAK     (4)   /**< OUT Packet - Not Acknowledged */
#define USB_EVT_IN_NAK      (5)   /**< IN Packet - Not Acknowledged */
#define USB_EVT_OUT_STALL   (6)   /**< OUT Packet - Stalled */
#define USB_EVT_IN_STALL    (7)   /**< IN Packet - Stalled */
#define USB_EVT_OUT_DMA_EOT (8)   /**< DMA OUT EP - End of Transfer */
#define USB_EVT_IN_DMA_EOT  (9)   /**< DMA IN EP - End of Transfer */
#define USB_EVT_OUT_DMA_NDR (10)  /**< DMA OUT EP - New Descriptor Request */
#define USB_EVT_IN_DMA_NDR  (11)  /**< DMA IN EP - New Descriptor Request */
#define USB_EVT_OUT_DMA_ERR (12)  /**< DMA OUT EP - Error */
#define USB_EVT_IN_DMA_ERR  (13)  /**< DMA IN EP - Error */

/* USB Endpoint Status (sent through CallBack) ------------------------------------ */
#define EP_STATUS_DATA		(1<<0)		/**< EP has data */
#define EP_STATUS_STALLED	(1<<1)		/**< EP is stalled */
#define EP_STATUS_SETUP		(1<<2)		/**< EP received setup packet */
#define EP_STATUS_ERROR		(1<<3)		/**< EP data was overwritten by setup packet */
#define EP_STATUS_NACKED	(1<<4)		/**< EP sent NAK */

/* USB Device Status (Sent through CallBack) ------------------------------------- */
#define DEV_STATUS_CONNECT		(1<<0)	/**< Device just got connected */
#define DEV_STATUS_SUSPEND		(1<<2)	/**< Device entered suspend state */
#define DEV_STATUS_RESET		(1<<4)	/**< Device just got reset */

/* USB Interrupt Type for NAK events --------------------------------------------- */
#define INACK_CI			(1<<1)			/**< Interrupt on NACK for control in */
#define INACK_CO			(1<<2)			/**< Interrupt on NACK for control out */
#define INACK_II			(1<<3)			/**< Interrupt on NACK for Interrupt in */
#define INACK_IO			(1<<4)			/**< Interrupt on NACK for Interrupt out */
#define INACK_BI			(1<<5)			/**< Interrupt on NACK for bulk in */
#define INACK_BO			(1<<6)			/**< Interrupt on NACK for bulk out */

/* USB Error Codes -------------------------------------------------------------- */
#define USB_ERR_PID         (0x0001)  /**< PID Error */
#define USB_ERR_UEPKT       (0x0002)  /**< Unexpected Packet */
#define USB_ERR_DCRC        (0x0004)  /**< Data CRC Error */
#define USB_ERR_TIMOUT      (0x0008)  /**< Bus Time-out Error */
#define USB_ERR_EOP         (0x0010)  /**< End of Packet Error */
#define USB_ERR_B_OVRN      (0x0020)  /**< Buffer Overrun */
#define USB_ERR_BTSTF       (0x0040)  /**< Bit Stuff Error */
#define USB_ERR_TGL         (0x0080)  /**< Toggle Bit Error */

/* USB DMA Status Codes -------------------------------------------------------- */
#define USB_DMA_INVALID     (0x0000)  /**< DMA Invalid - Not Configured */
#define USB_DMA_IDLE        (0x0001)  /**< DMA Idle - Waiting for Trigger */
#define USB_DMA_BUSY        (0x0002)  /**< DMA Busy - Transfer in progress */
#define USB_DMA_DONE        (0x0003)  /**< DMA Transfer Done (no Errors)*/
#define USB_DMA_OVER_RUN    (0x0004)  /**< Data Over Run */
#define USB_DMA_UNDER_RUN   (0x0005)  /**< Data Under Run (Short Packet) */
#define USB_DMA_ERROR       (0x0006)  /**< Error */
#define USB_DMA_UNKNOWN     (0xFFFF)  /**< Unknown State */

/* Macro used in USB Request Handler ------------------------------------------- */
#define REQTYPE_GET_DIR(x)		(((x)>>7)&0x01)		/**< Get Request Direction: To Host or To Device */
#define REQTYPE_GET_TYPE(x)		(((x)>>5)&0x03)		/**< Get Request Type */
#define REQTYPE_GET_RECIP(x)	((x)&0x1F)			/**< Get Request Recipient */

#define REQTYPE_DIR_TO_DEVICE	(0)					/**< Request Direction is To Device */
#define REQTYPE_DIR_TO_HOST		(1)					/**< Request Direction is To Host */

#define REQTYPE_TYPE_STANDARD	(0)					/**< Standard Request Type */
#define REQTYPE_TYPE_CLASS		(1)					/**< Class Request Type */
#define REQTYPE_TYPE_VENDOR		(2)					/**< Vendor Request Type */
#define REQTYPE_TYPE_RESERVED	(3)					/**< Reserved Request Type */

#define REQTYPE_RECIP_DEVICE	(0)					/**< Request Recipient type: Device */
#define REQTYPE_RECIP_INTERFACE	(1)					/**< Request Recipient type: Interface */
#define REQTYPE_RECIP_ENDPOINT	(2)					/**< Request Recipient type: EndPoint */
#define REQTYPE_RECIP_OTHER		(3)					/**< Request Recipient type: Other */

/* Standard requests definitions ------------------------------------------------ */
#define	REQ_GET_STATUS			(0x00)				/**< Get Status Request */
#define REQ_CLEAR_FEATURE		(0x01)				/**< Clear Feature Request */
#define REQ_SET_FEATURE			(0x03)				/**< Set Feature Request */
#define REQ_SET_ADDRESS			(0x05)				/**< Set Address Request */
#define REQ_GET_DESCRIPTOR		(0x06)				/**< Get Descriptor Request */
#define REQ_SET_DESCRIPTOR		(0x07)				/**< Set Descriptor Request */
#define REQ_GET_CONFIGURATION	(0x08)				/**< Set Configuration Request */
#define REQ_SET_CONFIGURATION	(0x09)				/**< Set Configuration Request */
#define REQ_GET_INTERFACE		(0x0A)				/**< Get Interface Request */
#define REQ_SET_INTERFACE		(0x0B)				/**< Set Interface Request */
#define REQ_SYNCH_FRAME			(0x0C)				/**< Synchronous Frame Request */

/* Class requests HID definitions ---------------------------------------------- */
#define HID_GET_REPORT			(0x01)				/**< HID Get Report Request */
#define HID_GET_IDLE			(0x02)				/**< HID Get Idle Request */
#define HID_GET_PROTOCOL	 	(0x03)				/**< HID Get Protocol Request */
#define HID_SET_REPORT			(0x09)				/**< HID Set Protocol Request */
#define HID_SET_IDLE			(0x0A)				/**< HID Set Idle Request */
#define HID_SET_PROTOCOL		(0x0B)				/**< HID Set Protocol Request */

/* Feature selectors definitions ---------------------------------------------- */
#define FEA_ENDPOINT_HALT		(0x00)				/**< Halt EP feature */
#define FEA_REMOTE_WAKEUP		(0x01)				/**< Remote WakeUp feature */
#define FEA_TEST_MODE			(0x02)				/**< Test mode feature */


/* USB Descriptor definitions -------------------------------------------------- */
#define DESC_DEVICE				(1)					/**< Device Descriptor */
#define DESC_CONFIGURATION		(2)					/**< Configuration Descriptor */
#define DESC_STRING				(3)					/**< String Descriptor */
#define DESC_INTERFACE			(4)					/**< Interface Descriptor */
#define DESC_ENDPOINT			(5)					/**< EndPoint Descriptor */
#define DESC_DEVICE_QUALIFIER	(6)					/**< Device Qualifier Descriptor */
#define DESC_OTHER_SPEED		(7)					/**< Other Speed Descriptor */
#define DESC_INTERFACE_POWER	(8)					/**< Interface Power Descriptor */

#define DESC_HID_HID			(0x21)				/**< HID Descriptor */
#define DESC_HID_REPORT			(0x22)				/**< HID Report Descriptor */
#define DESC_HID_PHYSICAL		(0x23)				/**< HID Physical Descriptor */

#define GET_DESC_TYPE(x)		(((x)>>8)&0xFF)		/**< Get Descriptor Type */
#define GET_DESC_INDEX(x)		((x)&0xFF)			/**< Get Descriptor Index */


/**
 * @}
 */

/**
 * @}
 */



/************************** GLOBAL/PUBLIC FUNCTIONS *************************/

/**
 * @addtogroup PUBLIC_FUNCTION_PROTOTYPES
 * @{
 */

/** @defgroup USBDEV_FUNCTIONS
 * @{
 */


/* USB Hardware Function Section --------------------------------------------- */
void USBDEV_Init(void);
void USBDEV_DeInit(void);
void USBDEV_SetAddress(uint8_t bAddr);
void USBDEV_Connect(FunctionalState NewState);


/* USB EndPoint Function Section --------------------------------------------- */
void USBDEV_EPRealize(uint8_t bEP, uint16_t wMaxPSize);
void USBDEV_EPCmd(uint8_t bEP, FunctionalState NewState);
void USBDEV_EPConfig(uint8_t bEP, uint16_t wMaxPacketSize);
void USBDEV_EPIntCmd(uint8_t bEP, FunctionalState NewState);
uint8_t USBDEV_EPGetStatus(uint8_t bEP);
void   USBDEV_EPStallCmd(uint8_t bEP, FunctionalState NewState);
int32_t USBDEV_EPWrite(uint8_t bEP, uint8_t *pbBuf, int32_t iLen);
int32_t USBDEV_EPRead(uint8_t bEP, uint8_t *pbBuf, int32_t iMaxLen);
uint32_t USBDEV_EPIsoWrite(uint32_t EPNum, uint8_t *pData, uint32_t cnt);
uint32_t USBDEV_EPIsoRead(uint32_t EPNum, uint8_t *pData);


/* USB Call Back Function Configuration Section ----------------------------- */
/* USB EndPoint Call Back Function */
void USBDEV_EPSetupCBS(uint8_t bEP, USB_EPIntHandler *pfnHandler);

/* USB Device Event Call Back Function */
void USBDEV_DeviceSetupCBS(USB_DevIntHandler *pfnHandler);

/* USB Frame Event Call Back Function */
void USBDEV_FrameSetupCBS(USB_FrameHandler *pfnHandler);

/* Default USB Device CallBack Function Initialization */
void USBDEV_InitDefaultCBS(void);

/* Standard USB IRQ Handler */
void USBDEV_StandardIRQHandler(void);

/* USB Device Reset handler */
void USBDEV_ResetHandler(uint8_t bDevStatus);


/* Other USB Device Configurations Section --------------------------------- */
void USBDEV_NAKIntConfig(uint8_t bIntBits);
void USBDEV_DeviceConfigCmd(FunctionalState NewState);


/* USB DMA functionalities section -------------------------------------------- */
#if _USB_DMA
void USBDEV_DMAEPSetupCBS(uint8_t bEP, USB_DMA_EPIntHandler *pfnHandler);
Bool USBDEV_DMAEPSetup(uint8_t bEP, USB_DMA_DESCRIPTOR *pDD) ;
void USBDEV_DMAEPCmd (uint8_t bEP, FunctionalState NewState) ;
uint32_t USBDEV_DMAEPGetStatus (uint8_t bEP) ;
uint32_t USBDEV_DMAEPGetBufAdr (uint8_t bEP) ;
uint32_t USBDEV_DMAEPGetBufCnt (uint8_t bEP) ;
#endif /* _USB_DMA */


/* USB Device Application Interface Function Declarations (Middle Ware Layer)---------------------------- */
/* USB Device EP0 (Control) handler */
void USBDEV_ControlHandler(uint8_t bEP, uint8_t bEPStat);

/* USB Device Request Handler */
void USBDEV_RequestHandlerSetupCBS(int32_t iType, USB_HandleRequest *pfnHandler, uint8_t *pbDataStore);
Bool USBDEV_StandardRequestHandler(USB_SETUP_PACKET *pSetup, int32_t *piLen, uint8_t **ppbData);
void USBDEV_CustomRequestHandlerSetupCBS(USB_HandleRequest *pfnHandler);

/* USB Device Descriptor Handler  */
void USBDEV_RegisterDescriptor(const uint8_t *pabDescriptors);
uint8_t USBDEV_GetCurrentConfiguration(void);
uint8_t USBDEV_GetCurrentInterface(void);
uint8_t USBDEV_GetCurrentAlternateSetting(void);
Bool USBDEV_GetDescriptor(uint16_t wTypeIndex, uint16_t wLangID, int32_t *piLen, uint8_t **ppbData);


/**
 * @}
 */

/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_USBDEV_H_ */
