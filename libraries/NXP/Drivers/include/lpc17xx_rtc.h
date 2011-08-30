/***********************************************************************//**
 * @file	: lpc17xx_rtc.h
 * @brief	: Contains all macro definitions and function prototypes
 * 				support for RTC firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 23. Apr. 2009
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

#ifndef LPC17XX_RTC_H_
#define LPC17XX_RTC_H_

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

/** @defgroup RTC_REGISTER_BIT_DEFINITIONS
 * @{
 */
/* Miscellaneous register group --------------------------------------------- */

/**********************************************************************
* ILR register definitions
**********************************************************************/
/** ILR register mask */
#define RTC_ILR_BITMASK			((uint32_t )(0x00000003))
/** Bit inform the source interrupt is counter increment*/
#define RTC_IRL_RTCCIF			((uint32_t)(1<<0))
/** Bit inform the source interrupt is alarm match*/
#define RTC_IRL_RTCALF			((uint32_t)(1<<1))
///**IRL set field bits */
//#define RTC_IRL_SET(bit, n)		((uint32_t)(n<<bit))

/**********************************************************************
* CCR register definitions
**********************************************************************/
/** CCR register mask */
#define RTC_CCR_BITMASK			((uint32_t )(0x00000013))
/** Clock enable */
#define RTC_CCR_CLKEN			((uint32_t)(1<<0))
/** Clock reset */
#define RTC_CCR_CTCRST			((uint32_t)(1<<1))
/** Calibration counter enable */
#define RTC_CCR_CCALEN			((uint32_t)(1<<4))
///**CCR set field bits */
//#define RTC_CCR_SET(bit, n)		((uint32_t)(n<<bit))

/**********************************************************************
* CIIR register definitions
**********************************************************************/
/** Counter Increment Interrupt bit for second */
#define RTC_CIIR_IMSEC			((uint32_t)(1<<0))
/** Counter Increment Interrupt bit for minute */
#define RTC_CIIR_IMMIN			((uint32_t)(1<<1))
/** Counter Increment Interrupt bit for hour */
#define RTC_CIIR_IMHOUR			((uint32_t)(1<<2))
/** Counter Increment Interrupt bit for day of month */
#define RTC_CIIR_IMDOM			((uint32_t)(1<<3))
/** Counter Increment Interrupt bit for day of week */
#define RTC_CIIR_IMDOW			((uint32_t)(1<<4))
/** Counter Increment Interrupt bit for day of year */
#define RTC_CIIR_IMDOY			((uint32_t)(1<<5))
/** Counter Increment Interrupt bit for month */
#define RTC_CIIR_IMMON			((uint32_t)(1<<6))
/** Counter Increment Interrupt bit for year */
#define RTC_CIIR_IMYEAR			((uint32_t)(1<<7))
/** CIIR bit mask */
#define RTC_CIIR_BITMASK		((uint32_t)(0xFF))

/**********************************************************************
* AMR register definitions
**********************************************************************/
/** Counter Increment Select Mask bit for second */
#define RTC_AMR_AMRSEC			((uint32_t)(1<<0))
/** Counter Increment Select Mask bit for minute */
#define RTC_AMR_AMRMIN			((uint32_t)(1<<1))
/** Counter Increment Select Mask bit for hour */
#define RTC_AMR_AMRHOUR			((uint32_t)(1<<2))
/** Counter Increment Select Mask bit for day of month */
#define RTC_AMR_AMRDOM			((uint32_t)(1<<3))
/** Counter Increment Select Mask bit for day of week */
#define RTC_AMR_AMRDOW			((uint32_t)(1<<4))
/** Counter Increment Select Mask bit for day of year */
#define RTC_AMR_AMRDOY			((uint32_t)(1<<5))
/** Counter Increment Select Mask bit for month */
#define RTC_AMR_AMRMON			((uint32_t)(1<<6))
/** Counter Increment Select Mask bit for year */
#define RTC_AMR_AMRYEAR			((uint32_t)(1<<7))
/** AMR bit mask */
#define RTC_AMR_BITMASK			((uint32_t)(0xFF))

/**********************************************************************
* RTC_AUX register definitions
**********************************************************************/
/** RTC Oscillator Fail detect flag */
#define RTC_AUX_RTC_OSCF		((uint32_t)(1<<4))

/**********************************************************************
* RTC_AUXEN register definitions
**********************************************************************/
/** Oscillator Fail Detect interrupt enable*/
#define RTC_AUXEN_RTC_OSCFEN	((uint32_t)(1<<4))


/* Consolidated time register group ----------------------------------- */
/** Consolidated Time Register 0 */
#define RTC_CTIME0_SECONDS_MASK		((uint32_t)(0x3F))
#define RTC_CTIME0_MINUTES_MASK		((uint32_t)(0x3F00))
#define RTC_CTIME0_HOURS_MASK		((uint32_t)(0x1F0000))
#define RTC_CTIME0_DOW_MASK			((uint32_t)(0x7000000))
/** Consolidated Time Register 1 */
#define RTC_CTIME1_DOM_MASK			((uint32_t)(0x1F))
#define RTC_CTIME1_MONTH_MASK		((uint32_t)(0xF00))
#define RTC_CTIME1_YEAR_MASK		((uint32_t)(0xFFF0000))
/** Consolidated Time Register 2 */
#define RTC_CTIME2_DOY_MASK			((uint32_t)(0xFFF))


/* Time Counter Group and Alarm register group ----------------------------- */
/** SEC register mask */
#define RTC_SEC_MASK			((uint32_t )0x0000003F)
/** MIN register mask */
#define RTC_MIN_MASK			((uint32_t )0x0000003F)
/** HOUR register mask */
#define RTC_HOUR_MASK			((uint32_t )0x0000001F)
/** DOM register mask */
#define RTC_DOM_MASK			((uint32_t )0x0000001F)
/** DOW register mask */
#define RTC_DOW_MASK			((uint32_t )0x00000007)
/** DOY register mask */
#define RTC_DOY_MASK			((uint32_t )0x000001FF)
/** MONTH register mask */
#define RTC_MONTH_MASK			((uint32_t )0x0000000F)
/** YEAR register mask */
#define RTC_YEAR_MASK			((uint32_t )0x00000FFF)

#define RTC_SECOND_MAX		59 /*!< Maximum value of second */
#define RTC_MINUTE_MAX		59 /*!< Maximum value of minute*/
#define RTC_HOUR_MAX		23 /*!< Maximum value of hour*/
#define RTC_MONTH_MIN		1 /*!< Minimum value of month*/
#define RTC_MONTH_MAX		12 /*!< Maximum value of month*/
#define RTC_DAYOFMONTH_MIN 	1 /*!< Minimum value of day of month*/
#define RTC_DAYOFMONTH_MAX 	31 /*!< Maximum value of day of month*/
#define RTC_DAYOFWEEK_MAX	6 /*!< Maximum value of day of week*/
#define RTC_DAYOFYEAR_MIN	1 /*!< Minimum value of day of year*/
#define RTC_DAYOFYEAR_MAX	366 /*!< Maximum value of day of year*/
#define RTC_YEAR_MAX		4095 /*!< Maximum value of year*/

/* Calibration register */
/** Calibration value */
#define RTC_CALIBRATION_CALVAL_MASK		((uint32_t)(0x1FFFF))
/** Calibration direction */
#define RTC_CALIBRATION_LIBDIR			((uint32_t)(1<<17))
/** Calibration max value */
#define RTC_CALIBRATION_MAX				((uint32_t)(0x20000))

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

/** @defgroup RTC_TYPES
 * @{
 */
/** Time structure definitions for easy manipulate the data */
typedef struct {
	uint32_t SEC; 		/*!< Seconds Register */
	uint32_t MIN; 		/*!< Minutes Register */
	uint32_t HOUR; 		/*!< Hours Register */
	uint32_t DOM;		/*!< Day of Month Register */
	uint32_t DOW; 		/*!< Day of Week Register */
	uint32_t DOY; 		/*!< Day of Year Register */
	uint32_t MONTH; 	/*!< Months Register */
	uint32_t YEAR; 		/*!< Years Register */
} RTC_TIME_Type;

/** RTC interrupt source */
typedef enum {
	RTC_INT_COUNTER_INCREASE = RTC_IRL_RTCCIF, 	/*!<  Counter Increment Interrupt */
	RTC_INT_ALARM = RTC_IRL_RTCALF, 				/*!< The alarm interrupt */
} RTC_INT_OPT;

#define PARAM_RTC_INT(n)	((n==RTC_INT_COUNTER_INCREASE) || (n==RTC_INT_ALARM))


/** RTC time type option */
typedef enum {
	RTC_TIMETYPE_SECOND = 0, 		/*!< Second */
	RTC_TIMETYPE_MINUTE = 1, 		/*!< Month */
	RTC_TIMETYPE_HOUR = 2, 			/*!< Hour */
	RTC_TIMETYPE_DAYOFWEEK = 3, 	/*!< Day of week */
	RTC_TIMETYPE_DAYOFMONTH = 4, 	/*!< Day of month */
	RTC_TIMETYPE_DAYOFYEAR = 5, 	/*!< Day of year */
	RTC_TIMETYPE_MONTH = 6, 		/*!< Month */
	RTC_TIMETYPE_YEAR = 7, 			/*!< Year */
} RTC_TIMETYPE_Num;

#define PARAM_RTC_TIMETYPE(n)	((n==RTC_TIMETYPE_SECOND) || (n==RTC_TIMETYPE_MINUTE) \
							|| (n==RTC_TIMETYPE_HOUR) || (n==RTC_TIMETYPE_DAYOFWEEK) \
							|| (n==RTC_TIMETYPE_DAYOFMONTH) || (n==RTC_TIMETYPE_DAYOFYEAR) \
							|| (n==RTC_TIMETYPE_MONTH) || (n==RTC_TIMETYPE_YEAR))

typedef struct
{
	uint8_t CLKEN;	/* 1: Timer, counter are enable
						   0: Counter are disable */
	uint8_t	CTCRST;	/* 1: Counter reset
						   0: No effect */
	uint8_t	CCALEN; /*   1: Calibration counter is disabled and reset to zero
						   0: Calibration is enable */
	uint8_t	RTCCIF;	/* 1: Counter increment interrupt block generated an interrupt
						   0" Counter increment interrupt block NO generated an interrupt          */
	uint8_t	RTALF;	/* 1: alarm register generated an interrupt
						   0: alarm register no generated an interrupt          */
	uint8_t	RESERVE[3];
}RTC_CFG_TYPE;

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

/** @defgroup RTC_MACROS
 * @{
 */
/** Macro to determine if it is valid RTC peripheral */
#define PARAM_RTCx(x)	(((uint32_t *)x)==((uint32_t *)RTC))

/** Calibration definitions */
#define RTC_CALIB_DIR_FORWARD	((uint8_t)(0))
#define RTC_CALIB_DIR_BACKWARD	((uint8_t)(1))

#define PARAM_RTC_CALIB_DIR(n)	((n==RTC_CALIB_DIR_FORWARD) || (n==RTC_CALIB_DIR_BACKWARD))
#define PARAM_RTC_GPREG_CH(n)	((n>=0) && (n<=4))

#define PARAM_RTC_CALIBRATION_DIR(n)

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

/** @defgroup RTC_FUNCTIONS
 * @{
 */
void RTC_Init (RTC_TypeDef *RTCx);
void RTC_DeInit(RTC_TypeDef *RTCx);
void RTC_ResetClockTickCounter(RTC_TypeDef *RTCx);
void RTC_Cmd (RTC_TypeDef *RTCx, FunctionalState NewState);
void RTC_CntIncrIntConfig (RTC_TypeDef *RTCx, uint32_t CntIncrIntType, \
								FunctionalState NewState);
void RTC_AlarmIntConfig (RTC_TypeDef *RTCx, uint32_t AlarmTimeType, \
								FunctionalState NewState);
void RTC_SetTime (RTC_TypeDef *RTCx, uint32_t Timetype, uint32_t TimeValue);
uint32_t RTC_GetTime(RTC_TypeDef *RTCx, uint32_t Timetype);
void RTC_SetFullTime (RTC_TypeDef *RTCx, RTC_TIME_Type *pFullTime);
void RTC_GetFullTime (RTC_TypeDef *RTCx, RTC_TIME_Type *pFullTime);
void RTC_SetAlarmTime (RTC_TypeDef *RTCx, uint32_t Timetype, uint32_t ALValue);
uint32_t RTC_GetAlarmTime (RTC_TypeDef *RTCx, uint32_t Timetype);
void RTC_SetFullAlarmTime (RTC_TypeDef *RTCx, RTC_TIME_Type *pFullTime);
void RTC_GetFullAlarmTime (RTC_TypeDef *RTCx, RTC_TIME_Type *pFullTime);
IntStatus RTC_GetIntPending (RTC_TypeDef *RTCx, uint32_t IntType);
void RTC_ClearIntPending (RTC_TypeDef *RTCx, uint32_t IntType);
void RTC_CalibCounterCmd(RTC_TypeDef *RTCx, FunctionalState NewState);
void RTC_CalibConfig(RTC_TypeDef *RTCx, uint32_t CalibValue, uint8_t CalibDir);
void RTC_WriteGPREG (RTC_TypeDef *RTCx, uint8_t Channel, uint32_t Value);
uint32_t RTC_ReadGPREG (RTC_TypeDef *RTCx, uint8_t Channel);

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif


#endif /* LPC17XX_RTC_H_ */
