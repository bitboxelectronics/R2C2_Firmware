/**
 * @file	: lpc17xx_can.c
 * @brief	: Contains all functions support for CAN firmware library on LPC17xx
 * @version	: 1.0
 * @date	: 1.June.2009
 * @author	: NguyenCao
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

#include "lpc17xx_can.h"
#include "lpc17xx_clkpwr.h"
//#include "lpc17xx_pinsel.h"


/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

FunctionalState FULLCAN_ENABLE;

//use for debugging
CAN_TypeDef *CAN1x = CAN1;
CAN_TypeDef *CAN2x = CAN2;
CANAF_RAM_TypeDef *CANAFRAMx = CANAF_RAM;
CANAF_TypeDef *CANAFx = CANAF;

/************************** PRIVATE VARIABLES *************************/
#ifdef _CAN

/** @addtogroup Private_Variables
 * @{
 */

/** @defgroup CAN_Private_Variables
 * @{
 */

/* Values of bit time register for different baudrates
   NT = Nominal bit time = TSEG1 + TSEG2 + 3
   SP = Sample point     = ((TSEG2 +1) / (TSEG1 + TSEG2 + 3)) * 100%
                                            SAM,  SJW, TSEG1, TSEG2, NT,  SP */
const uint32_t CAN_BIT_TIME[] = {       0, /*             not used             */
                                      0, /*             not used             */
                                      0, /*             not used             */
                                      0, /*             not used             */
                             0x0001C000, /* 0+1,  3+1,   1+1,   0+1,  4, 75% */
                                      0, /*             not used             */
                             0x0012C000, /* 0+1,  3+1,   2+1,   1+1,  6, 67% */
                                      0, /*             not used             */
                             0x0023C000, /* 0+1,  3+1,   3+1,   2+1,  8, 63% */
                                      0, /*             not used             */
                             0x0025C000, /* 0+1,  3+1,   5+1,   2+1, 10, 70% */
                                      0, /*             not used             */
                             0x0036C000, /* 0+1,  3+1,   6+1,   3+1, 12, 67% */
                                      0, /*             not used             */
                                      0, /*             not used             */
                             0x0048C000, /* 0+1,  3+1,   8+1,   4+1, 15, 67% */
                             0x0049C000, /* 0+1,  3+1,   9+1,   4+1, 16, 69% */
                           };
/* Counts number of filters (CAN message objects) used */
uint16_t CANAF_FullCAN_cnt = 0;
uint16_t CANAF_std_cnt = 0;
uint16_t CANAF_gstd_cnt = 0;
uint16_t CANAF_ext_cnt = 0;
uint16_t CANAF_gext_cnt = 0;

static fnCANCbs_Type* _apfnCANCbs[12]={
		NULL, 	//CAN Recieve  Call-back funtion pointer
		NULL,	//CAN Transmit1 Call-back funtion pointer
		NULL,	//CAN Error Warning Call-back function pointer
		NULL,	//CAN Data Overrun Call-back function pointer
		NULL,	//CAN Wake-up Call-back funtion pointer
		NULL,	//CAN Error Passive Call-back function pointer
		NULL,	//CAN Arbitration Lost Call-back function pointer
		NULL,	//CAN Bus Error Call-back function pointer
		NULL,	//CAN ID Ready Call-back function pointer
		NULL,	//CAN Transmit2 Call-back function pointer
		NULL,	//CAN Transmit3 Call-back function pointer
		NULL	//FullCAN Receive Call-back function pointer
};

/**
 * @}
 */

/**
 * @}
 */
/***************************** PRIVATE FUNCTION *****************************/
/** @addtogroup  Private_Functions
 * @{
 */

/** @defgroup 	CAN_Private_Functions
 * @{
 */
/*********************************************************************//**
 * @brief 		Setting CAN baud rate
 * @param[in] 	CANx point to CAN_TypeDef object, should be:
 * 				- CAN1
 * 				- CAN2
 * @param[in]	baudrate is the baud rate value will be set
 * @return 		None
 ***********************************************************************/
void CAN_SetBaudRate (CAN_TypeDef *CANx, uint32_t baudrate)
{
	uint32_t nominal_time;
	uint32_t result = 0;
	uint32_t CANPclk = 0;

	CHECK_PARAM(PARAM_CANx(CANx));

	if (CANx == CAN1)
	{
		CANPclk = CLKPWR_GetPCLK (CLKPWR_PCONP_PCAN1);
	}
	else
	{
		CANPclk = CLKPWR_GetPCLK (CLKPWR_PCONP_PCAN2);
	}
	/* Determine which nominal time to use for PCLK and baudrate */
	if (baudrate <= 500000)
	{
		nominal_time = 12;
	}
	else if (((CANPclk / 1000000) % 15) == 0)
	{
		nominal_time = 15;
	}
	else if (((CANPclk / 1000000) % 16) == 0)
	{
		nominal_time = 16;
	}
	else
	{
		nominal_time = 10;
	}

	/* Prepare value appropriate for bit time register                         */
	result  = (CANPclk / nominal_time) / baudrate - 1;
	result &= 0x000003FF;
	result |= CAN_BIT_TIME[nominal_time];

	/* Enter reset mode */
	CANx->MOD = 0x01;
	/* Set bit timing */
	CANx->BTR  = result;

	/* Return to normal operating */
	CANx->MOD = 0;
}

/********************************************************************//**
 * @brief		Initialize CAN peripheral with given baudrate
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	baudrate: the value of CAN baudrate will be set
 * @return 		void
 *********************************************************************/
void CAN_Init(CAN_TypeDef *CANx, uint32_t baudrate)
{
	uint32_t temp;
	CHECK_PARAM(PARAM_CANx(CANx));

	if(CANx == CAN1)
	{
		/* Turn on power and clock for CAN1 */
		CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCAN1, ENABLE);
		/* Set clock divide for CAN1 */
		CLKPWR_SetPCLKDiv (CLKPWR_PCONP_PCAN1, CLKPWR_PCLKSEL_CCLK_DIV_4);
	}
	else
	{
		/* Turn on power and clock for CAN1 */
		CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCAN2, ENABLE);
		/* Set clock divide for CAN2 */
		CLKPWR_SetPCLKDiv (CLKPWR_PCONP_PCAN2, CLKPWR_PCLKSEL_CCLK_DIV_4);
	}

	CANx->MOD = 1; // Enter Reset Mode
	CANx->IER = 0; // Disable All CAN Interrupts
	CANx->GSR = 0;
	/* Request command to release Rx, Tx buffer and clear data overrun */
	//CANx->CMR = CAN_CMR_AT | CAN_CMR_RRB | CAN_CMR_CDO;
	CANx->CMR = (1<<1)|(1<<2)|(1<<3);
	/* Read to clear interrupt pending in interrupt capture register */
	temp = CANx->ICR;
	CANx->MOD = 0;// Return Normal operating

	//Reset CANAF value
	CANAF->AFMR = 0x01;

	CANAF->SFF_sa = 0x00;
	CANAF->SFF_GRP_sa = 0x00;
	CANAF->EFF_sa = 0x00;
	CANAF->EFF_GRP_sa = 0x00;
	CANAF->ENDofTable = 0x00;

	CANAF->AFMR = 0x00;
	/* Set baudrate */
	CAN_SetBaudRate (CANx, baudrate);
}
/********************************************************************//**
 * @brief		Add Explicit ID into AF Look-Up Table dynamically.
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	id: The ID of entry will be added
 * @param[in]	format: is the type of ID Frame Format, should be:
 * 				- STD_ID_FORMAT: 11-bit ID value
 * 				- EXT_ID_FORMAT: 29-bit ID value
 * @return 		CAN Error, could be:
 * 				- CAN_OBJECTS_FULL_ERROR: No more rx or tx objects available
				- CAN_OK: ID is added into table successfully
 *********************************************************************/
CAN_ERROR CAN_LoadID (CAN_TypeDef *CANx,CANAF_RAM_TypeDef *CANAFRAM, uint32_t id, CAN_ID_FORMAT_Type format)
{
	uint32_t ctrl0 = 0;
	uint32_t buf0, buf1;
	int16_t cnt1, cnt2, bound1, total;

	CHECK_PARAM(PARAM_CANx(CANx));
	CHECK_PARAM(PARAM_ID_FORMAT(format));
	CHECK_PARAM(PARAM_CANAFRAMx(CANAFRAM));

	if (CANx == CAN1)
	{
		ctrl0 = 0;
	}
	else if (CANx == CAN2)
	{
		ctrl0 = 1;
	}

	/* Acceptance Filter Memory full - return */
	total = ((CANAF_std_cnt + 1) >> 1)+ CANAF_gstd_cnt + CANAF_ext_cnt + \
			(CANAF_ext_cnt<<1);
	if(total >= 512 )
	{
		return CAN_OBJECTS_FULL_ERROR;
	}

	/* Setup Acceptance Filter Configuration
    Acceptance Filter Mode Register = Off */
 	CANAF->AFMR = 0x00000001;

 	if (format == STD_ID_FORMAT)
 	{
 		/* Add mask for standard identifiers   */
 		id &= 0x07FF;
 		id |= (ctrl0 << 13) | (1 << 11); /* Add controller number */

 		/* Move all remaining extended mask entries one place up
		if new entry will increase standard ID filters list */
 		if ((CANAF_std_cnt & 0x0001) == 0 && CANAF_ext_cnt != 0)
 		{
 			cnt1   = (CANAF_std_cnt >> 1);
 			bound1 = CANAF_ext_cnt;
 			buf0   = CANAFRAM->mask[cnt1];
 			while (bound1--)
 			{
 				cnt1++;
 				buf1 = CANAF_RAM->mask[cnt1];
 				CANAFRAM->mask[cnt1] = buf0;
 				buf0 = buf1;
 			}
 		}
 		if (CANAF_std_cnt == 0)
 		{
 			/* For entering first ID */
 			CANAFRAM->mask[0] = 0x0000FFFF | (id << 16);
 		}
 		else if (CANAF_std_cnt == 1)
 		{
 			/* For entering second ID */
 			if ((CANAFRAM->mask[0] >> 16) > id)
 			{
 				CANAFRAM->mask[0] = (CANAFRAM->mask[0] >> 16) | (id << 16);
 			}
 			else
 			{
 				CANAFRAM->mask[0] = (CANAFRAM->mask[0] & 0xFFFF0000) | id;
 			}
 		}
 		else
 		{
 			/* Find where to insert new ID */
 			cnt1 = 0;
 			cnt2 = CANAF_std_cnt;
 			bound1 = (CANAF_std_cnt - 1) >> 1;
 			while (cnt1 <= bound1)
 			{
 				/* Loop through standard existing IDs */
 				if ((CANAFRAM->mask[cnt1] >> 16) > id)
 				{
 					cnt2 = cnt1 * 2;
 					break;
 				}

 				if ((CANAFRAM->mask[cnt1] & 0x0000FFFF) > id)
 				{
 					cnt2 = cnt1 * 2 + 1;
 					break;
 				}

 				cnt1++;
 			}
 			/* cnt1 = U32 where to insert new ID */
 			/* cnt2 = U16 where to insert new ID */

 			if (cnt1 > bound1)
 			{
 				/* Adding ID as last entry */
 				/* Even number of IDs exists */
 				if ((CANAF_std_cnt & 0x0001) == 0)
 				{
 					CANAFRAM->mask[cnt1]  = 0x0000FFFF | (id << 16);
 				}
 				/* Odd  number of IDs exists */
 				else
 				{
 					CANAFRAM->mask[cnt1]  = (CANAFRAM->mask[cnt1] & 0xFFFF0000) | id;
 				}
 			}
 			else
 			{
 				buf0 = CANAFRAM->mask[cnt1]; /* Remember current entry */
 				if ((cnt2 & 0x0001) == 0)
 				{
 					/* Insert new mask to even address*/
 					buf1 = (id << 16) | (buf0 >> 16);
 				}
 				else
 				{
 					/* Insert new mask to odd  address */
 					buf1 = (buf0 & 0xFFFF0000) | id;
 				}
 				CANAFRAM->mask[cnt1] = buf1;/* Insert mask */
 				bound1 = CANAF_std_cnt >> 1;
 				/* Move all remaining standard mask entries one place up */
 				while (cnt1 < bound1)
 				{
 					cnt1++;
 					buf1  = CANAFRAM->mask[cnt1];
 					CANAFRAM->mask[cnt1] = (buf1 >> 16) | (buf0 << 16);
 					buf0  = buf1;
 				}

 				if ((CANAF_std_cnt & 0x0001) == 0)
 				{
 					/* Even number of IDs exists */
 					CANAFRAM->mask[cnt1] = (CANAFRAM->mask[cnt1] & 0xFFFF0000)
 					 							| (0x0000FFFF);
 				}
 			}
 		}
 		CANAF_std_cnt++;
 		buf0 = ((CANAF_std_cnt + 1) >> 1) << 2;
 		if(buf0 != CANAF->SFF_GRP_sa) // add new entry
 		{
 			CANAF->SFF_GRP_sa +=0x04 ;
 			CANAF->EFF_sa     +=0x04 ;
 			CANAF->EFF_GRP_sa +=0x04;
 			CANAF->ENDofTable +=0x04;
 		}
 	}

 	/* Add mask for extended identifiers */
 	else
 	{
 		/* Add controller number */
 		id |= (ctrl0) << 29;

 		cnt1 = (((CANAF_std_cnt + 1) >> 1) + CANAF_gstd_cnt);
 		cnt2 = 0;
 		while (cnt2 < CANAF_ext_cnt)
 		{
 			/* Loop through extended existing masks*/
 			if (CANAFRAM->mask[cnt1] > id)
 			{
 				break;
 			}
 			cnt1++;/* cnt1 = U32 where to insert new mask */
			cnt2++;
 		}

 		buf0 = CANAFRAM->mask[cnt1];  /* Remember current entry */
 		CANAFRAM->mask[cnt1] = id;    /* Insert mask */

 		CANAF_ext_cnt++;

 		bound1 = CANAF_ext_cnt + CANAF_gext_cnt - 1;
 		/* Move all remaining extended mask entries one place up*/
 		while (cnt2 < bound1)
 		{
 			cnt1++;
 			cnt2++;
 			buf1 = CANAFRAM->mask[cnt1];
 			CANAFRAM->mask[cnt1] = buf0;
 			buf0 = buf1;
 		}
 		/* update address values */
		CANAF->EFF_GRP_sa += 4;
		CANAF->ENDofTable += 4;
 	}
 	if(CANAF_FullCAN_cnt == 0) //not use FullCAN mode
 	{
 		CANAF->AFMR = 0x00;//not use FullCAN mode
 	}
 	else
 	{
 		CANAF->AFMR = 0x04;
 	}

 	return CAN_OK;
}
/********************************************************************//**
 * @brief		Setup Acceptance Filter Look-Up Table
 * @param[in]	CANx pointer to CANAF_TypeDef, should be: CANAF
 * @param[in]	AFSection is the pointer to AF_SectionDef struct
 * 				It contain information about 5 sections will be install in AFLUT
 * @return 		CAN Error, could be:
 * 				- CAN_OBJECTS_FULL_ERROR: No more rx or tx objects available
 * 				- CAN_AF_ENTRY_ERROR: table error-violation of ascending numerical order
 * 				- CAN_OK: ID is added into table successfully
 *********************************************************************/
CAN_ERROR CAN_SetupAFLUT(CANAF_TypeDef* CANAFx,CANAF_RAM_TypeDef* CANAFRAM, AF_SectionDef* AFSection)
{
	uint8_t ctrl1,ctrl2;
	uint8_t dis1, dis2;
	uint16_t SID, SID_temp, count = 0;
	uint32_t EID, EID_temp, entry, buf;
	uint16_t lowerSID, upperSID;
	uint32_t lowerEID, upperEID;

	CHECK_PARAM(PARAM_CANAFx(CANAFx));
	CHECK_PARAM(PARAM_CANAFRAMx(CANAFRAM));
	CANAFx->AFMR = 0x01;

/***** setup FullCAN Table *****/
	if(AFSection->FullCAN_Sec == NULL)
	{
		FULLCAN_ENABLE = DISABLE;
	}
	else
	{
		FULLCAN_ENABLE = ENABLE;
		while(1)
		{
			if(count + 1 > 64)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->FullCAN_Sec->controller;
			SID = AFSection->FullCAN_Sec->id_11;
			dis1 = AFSection->FullCAN_Sec->disable;

			CHECK_PARAM(PARAM_CTRL(ctrl1));
			CHECK_PARAM(PARAM_ID_11(SID));
			CHECK_PARAM(PARAM_MSG_DISABLE(dis1));
			entry = 0x00; //reset entry value
			if((CANAF_FullCAN_cnt & 0x00000001)==0)
			{
				if(count!=0x00)
				{
					buf = CANAFRAM->mask[count-1];
					SID_temp = (buf & 0x000003FF);
					if(SID_temp > SID)
					{
						return CAN_AF_ENTRY_ERROR;
					}
				}
				entry = (ctrl1<<29)|(dis1<<28)|(SID<<16)|(1<<27);
				CANAFRAM->mask[count] &= 0x0000FFFF;
				CANAFRAM->mask[count] |= entry;
				CANAF_FullCAN_cnt++;
			}
			else
			{
				buf = CANAFRAM->mask[count];
				SID_temp = (buf & 0x03FF0000)>>16;
				if(SID_temp > SID)
				{
					return CAN_AF_ENTRY_ERROR;
				}
				entry = (ctrl1<<13)|(dis1<<12)|(SID<<0)|(1<<11);
				CANAF_RAM->mask[count] &= 0xFFFF0000;
				CANAF_RAM->mask[count]|= entry;
				count++;
				CANAF_FullCAN_cnt++;
			}
			AFSection->FullCAN_Sec =(FullCAN_Entry*)(AFSection->FullCAN_Sec->next);
			if(AFSection->FullCAN_Sec == NULL) break;
		}
	}

/***** Setup Explicit Standard Frame Format Section *****/
	if(AFSection->SFF_Sec != NULL)
	{
		while(1)
		{
			if(count + 1 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->SFF_Sec->controller;
			SID = AFSection->SFF_Sec->id_11;
			dis1 = AFSection->SFF_Sec->disable;

			//check parameter
			CHECK_PARAM(PARAM_CTRL(ctrl1));
			CHECK_PARAM(PARAM_ID_11(SID));
			CHECK_PARAM(PARAM_MSG_DISABLE(dis1));

			entry = 0x00; //reset entry value
			if((CANAF_std_cnt & 0x00000001)==0)
			{
				if(CANAF_std_cnt !=0 )
				{
					buf = CANAFRAM->mask[count-1];
					SID_temp = (buf & 0x00000FFF);
					if(SID_temp > SID)
					{
						return CAN_AF_ENTRY_ERROR;
					}
				}
				entry = (ctrl1<<29)|(dis1<<28)|(SID<<16);
				CANAFRAM->mask[count] &= 0x0000FFFF;
				CANAFRAM->mask[count] |= entry;
				CANAF_std_cnt++;
			}
			else
			{
				buf = CANAFRAM->mask[count];
				SID_temp = (buf & 0x0FFF0000)>>16;
				if(SID_temp > SID)
				{
					return CAN_AF_ENTRY_ERROR;
				}
				entry = (ctrl1<<13)|(dis1<<12)|(SID<<0);
				CANAFRAM->mask[count] &= 0xFFFF0000;
				CANAFRAM->mask[count] |= entry;
				count++;
				CANAF_std_cnt++;
			}
			AFSection->SFF_Sec = (SFF_Entry*)(AFSection->SFF_Sec->next);
			if(AFSection->SFF_Sec == NULL) break;
		}
	}
/***** Setup Group of Standard Frame Format Identifier Section *****/
	if(AFSection->SFF_GPR_Sec != NULL)
	{
		while(1)
		{
			if(count + 1 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->SFF_GPR_Sec->controller1;
			ctrl2 = AFSection->SFF_GPR_Sec->controller2;
			dis1 = AFSection->SFF_GPR_Sec->disable1;
			dis2 = AFSection->SFF_GPR_Sec->disable2;
			lowerSID = AFSection->SFF_GPR_Sec->lowerID;
			upperSID = AFSection->SFF_GPR_Sec->upperID;

			/* check parameter */
			CHECK_PARAM(PARAM_CTRL(ctrl1));
			CHECK_PARAM(PARAM_CTRL(ctrl2));
			CHECK_PARAM(PARAM_MSG_DISABLE(dis1));
			CHECK_PARAM(PARAM_MSG_DISABLE(dis2));
			CHECK_PARAM(PARAM_ID_11(lowerSID));
			CHECK_PARAM(PARAM_ID_11(upperSID));

			entry = 0x00;
			if(CANAF_gstd_cnt!=0)
			{
				buf = CANAFRAM->mask[count-1];
				SID_temp = buf & 0x00000FFF;
				if(SID_temp > lowerSID)
				{
					return CAN_AF_ENTRY_ERROR;
				}
			}
			entry = (ctrl1 << 29)|(dis1 << 28)|(lowerSID << 16)|  \
					(ctrl2 << 13)|(dis2 << 12)|(upperSID << 0);
			CANAFRAM->mask[count] = entry;
			CANAF_gstd_cnt++;
			count++;
			AFSection->SFF_GPR_Sec = (SFF_GPR_Entry*)(AFSection->SFF_GPR_Sec->next);
			if(AFSection->SFF_GPR_Sec == NULL) break;
		}
	}

/***** Setup Explicit Extend Frame Format Identifier Section *****/
	if(AFSection->EFF_Sec != NULL)
	{
		while(1)
		{
			if(count + 1 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			EID = AFSection->EFF_Sec->ID_29;
			ctrl1 = AFSection->EFF_Sec->controller;

			// check parameter
			CHECK_PARAM(PARAM_ID_29(EID));
			CHECK_PARAM(PARAM_CTRL(ctrl1));

			entry = 0x00; //reset entry value
			if(CANAF_ext_cnt != 0)
			{
				buf = CANAFRAM->mask[count-1];
				EID_temp = buf & 0x0FFFFFFF;
				if(EID_temp > EID)
				{
					return CAN_AF_ENTRY_ERROR;
				}
			}
			entry = (ctrl1 << 29)|(EID << 0);
			CANAFRAM->mask[count] = entry;
			CANAF_ext_cnt ++;
			count++;
			AFSection->EFF_Sec = (EFF_Entry*)(AFSection->EFF_Sec->next);
			if(AFSection->EFF_Sec == NULL) break;
		}
	}
/***** Setup Group of Extended Frame Format Identifier Section *****/
	if(AFSection->EFF_GPR_Sec != NULL)
	{
		while(1)
		{
			if(count + 2 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->EFF_GPR_Sec->controller1;
			ctrl2 = AFSection->EFF_GPR_Sec->controller2;
			lowerEID = AFSection->EFF_GPR_Sec->lowerEID;
			upperEID = AFSection->EFF_GPR_Sec->upperEID;

			//check parameter
			CHECK_PARAM(PARAM_CTRL(ctrl1));
			CHECK_PARAM(PARAM_CTRL(ctrl2));
			CHECK_PARAM(PARAM_ID_29(lowerEID));
			CHECK_PARAM(PARAM_ID_29(upperEID));

			entry = 0x00;
			if(CANAF_gext_cnt != 0)
			{
				buf = CANAFRAM->mask[count-1];
				EID_temp = buf & 0x0FFFFFFF;
				if(EID_temp > lowerEID)
				{
					return CAN_AF_ENTRY_ERROR;
				}
			}
			entry = (ctrl1 << 29)|(lowerEID << 0);
			CANAFRAM->mask[count++] = entry;
			entry = (ctrl2 << 29)|(upperEID << 0);
			CANAFRAM->mask[count++] = entry;
			CANAF_gext_cnt++;
			AFSection->EFF_GPR_Sec = (EFF_GPR_Entry*)(AFSection->EFF_GPR_Sec->next);
			if(AFSection->EFF_GPR_Sec == NULL) break;
		}
	}
	//update address values
	CANAF->SFF_sa = ((CANAF_FullCAN_cnt + 1)>>1)<<2;
	CANAF->SFF_GRP_sa = CANAF->SFF_sa + (((CANAF_std_cnt+1)>>1)<< 2);
	CANAF->EFF_sa = CANAF->SFF_GRP_sa + (CANAF_gstd_cnt << 2);
	CANAF->EFF_GRP_sa = CANAF->EFF_sa + (CANAF_ext_cnt << 2);
	CANAF->ENDofTable = CANAF->EFF_GRP_sa + (CANAF_ext_cnt << 3);

	if(FULLCAN_ENABLE == DISABLE)
	{
		CANAF->AFMR = 0x00; // Normal mode
	}
	else
	{
		//Enable FullCAN Interrupt
//		CANAF->FCANIE = 0x01;
		CANAF->AFMR = 0x04;
	}
	return CAN_OK;
}
/********************************************************************//**
 * @brief		Send message data
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	CAN_Msg point to the CAN_MSG_Type Structure, it contains message
 * 				information such as: ID, DLC, RTR, ID Format
 * @return 		Status:
 * 				- SUCCESS: send message successfully
 * 				- ERROR: send message unsuccessfully
 *********************************************************************/
Status CAN_SendMsg (CAN_TypeDef *CANx, CAN_MSG_Type *CAN_Msg)
{
	CHECK_PARAM(PARAM_CANx(CANx));
	CHECK_PARAM(PARAM_ID_FORMAT(CAN_Msg->format));
	if(CAN_Msg->format==STD_ID_FORMAT)
	{
		CHECK_PARAM(PARAM_ID_11(CAN_Msg->id));
	}
	else
	{
		CHECK_PARAM(PARAM_ID_29(CAN_Msg->id));
	}
	CHECK_PARAM(PARAM_DLC(CAN_Msg->len));
	CHECK_PARAM(PARAM_FRAME_TYPE(CAN_Msg->type));

	//Check status of Transmit Buffer 1
	if ((CANx->SR & 0x00000004)>>2)
	{
		/* Transmit Channel 1 is available */
		/* Write frame informations and frame data into its CANxTFI1,
		 * CANxTID1, CANxTDA1, CANxTDB1 register */
		CANx->TFI1 &= ~0x000F000;
		CANx->TFI1 |= (CAN_Msg->len)<<16;
		if(CAN_Msg->type == REMOTE_FRAME)
		{
			CANx->TFI1 |= (1<<30); //set bit RTR
		}
		else
		{
			CANx->TFI1 &= ~(1<<30);
		}
		if(CAN_Msg->format == EXT_ID_FORMAT)
		{
			CANx->TFI1 |= (1<<31); //set bit FF
		}
		else
		{
			CANx->TFI1 &= ~(1<<31);
		}

		/* Write CAN ID*/
		CANx->TID1 = CAN_Msg->id;

		/*Write first 4 data bytes*/
		CANx->TDA1 = *((uint32_t *) &(CAN_Msg->dataA));

		/*Write second 4 data bytes*/
		 CANx->TDB1 = *((uint32_t *) &(CAN_Msg->dataB));

		 /*Write transmission request*/
		 CANx->CMR = 0x21;
		 return SUCCESS;
	}
	//check status of Transmit Buffer 2
	else if((CANx->SR & 0x00000004)>>10)
	{
		/* Transmit Channel 2 is available */
		/* Write frame informations and frame data into its CANxTFI2,
		 * CANxTID2, CANxTDA2, CANxTDB2 register */
		CANx->TFI2 &= ~0x000F000;
		CANx->TFI2 |= (CAN_Msg->len)<<16;
		if(CAN_Msg->type == REMOTE_FRAME)
		{
			CANx->TFI2 |= (1<<30); //set bit RTR
		}
		else
		{
			CANx->TFI2 &= ~(1<<30);
		}
		if(CAN_Msg->format == EXT_ID_FORMAT)
		{
			CANx->TFI2 |= (1<<31); //set bit FF
		}
		else
		{
			CANx->TFI2 &= ~(1<<31);
		}

		/* Write CAN ID*/
		CANx->TID2 = CAN_Msg->id;

		/*Write first 4 data bytes*/
		CANx->TDA2 = *((uint32_t *) &(CAN_Msg->dataA));

		/*Write second 4 data bytes*/
		CANx->TDB2 = *((uint32_t *) &(CAN_Msg->dataB));

		/*Write transmission request*/
		CANx->CMR = 0x41;
		return SUCCESS;
	}
	//check status of Transmit Buffer 3
	else if ((CANx->SR & 0x00000004)>>18)
	{
		/* Transmit Channel 3 is available */
		/* Write frame informations and frame data into its CANxTFI3,
		 * CANxTID3, CANxTDA3, CANxTDB3 register */
		CANx->TFI3 &= ~0x000F000;
		CANx->TFI3 |= (CAN_Msg->len)<<16;
		if(CAN_Msg->type == REMOTE_FRAME)
		{
			CANx->TFI3 |= (1<<30); //set bit RTR
		}
		else
		{
			CANx->TFI3 &= ~(1<<30);
		}
		if(CAN_Msg->format == EXT_ID_FORMAT)
		{
			CANx->TFI3 |= (1<<31); //set bit FF
		}
		else
		{
			CANx->TFI3 &= ~(1<<31);
		}

		/* Write CAN ID*/
		CANx->TID3 = CAN_Msg->id;

		/*Write first 4 data bytes*/
		CANx->TDA3 = *((uint32_t *) &(CAN_Msg->dataA));

		/*Write second 4 data bytes*/
		CANx->TDB3 = *((uint32_t *) &(CAN_Msg->dataB));

		/*Write transmission request*/
		CANx->CMR = 0x81;
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

/********************************************************************//**
 * @brief		Receive message data
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	CAN_Msg point to the CAN_MSG_Type Struct, it will contain received
 *  			message information such as: ID, DLC, RTR, ID Format
 * @return 		Status:
 * 				- SUCCESS: receive message successfully
 * 				- ERROR: receive message unsuccessfully
 *********************************************************************/
Status CAN_ReceiveMsg (CAN_TypeDef *CANx, CAN_MSG_Type *CAN_Msg)
{
	CHECK_PARAM(PARAM_CANx(CANx));

	//check status of Receive Buffer
	if((CANx->SR &0x00000001))
	{
		/* Receive message is available */
		/* Read frame informations */
		CAN_Msg->format   = (uint8_t)(((CANx->RFS) & 0x80000000)>>31);
		CAN_Msg->type     = (uint8_t)(((CANx->RFS) & 0x40000000)>>30);
		CAN_Msg->len      = (uint8_t)(((CANx->RFS) & 0x000F0000)>>16);


		/* Read CAN message identifier */
		CAN_Msg->id = CANx->RID;

		/* Read the data if received message was DATA FRAME */
		if (CAN_Msg->type == DATA_FRAME)
		{
			/* Read first 4 data bytes */
			*((uint32_t *) &CAN_Msg->dataA) = CANx->RDA;

			/* Read second 4 data bytes */
			*((uint32_t *) &CAN_Msg->dataB) = CANx->RDB;

		/*release receive buffer*/
		CANx->CMR = 0x04;
		}
		else
		{
			/* Received Frame is a Remote Frame, not have data, we just receive
			 * message information only */
			return SUCCESS;
		}
	}
	else
	{
		// no receive message available
		return ERROR;
	}
	return SUCCESS;
}

/********************************************************************//**
 * @brief		Receive FullCAN Object
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	CAN_Msg point to the CAN_MSG_Type Struct, it will contain received
 *  			message information such as: ID, DLC, RTR, ID Format
 * @return 		CAN_ERROR, could be:
 * 				- CAN_FULL_OBJ_NOT_RCV: FullCAN Object is not be received
 * 				- CAN_OK: Received FullCAN Object successful
 *
 *********************************************************************/
CAN_ERROR FCAN_ReadObj (CANAF_TypeDef* CANAFx, CAN_MSG_Type *CAN_Msg)
{
	uint32_t *pSrc;
	uint32_t interrut_word, msg_idx, test_bit, head_idx, tail_idx;

	CHECK_PARAM(PARAM_CANAFx(CANAFx));

	interrut_word = 0;

	if (CANAF->FCANIC0 != 0)
	{
		interrut_word = CANAF->FCANIC0;
		head_idx = 0;
		tail_idx = 31;
	}
	else if (CANAF->FCANIC1 != 0)
	{
		interrut_word = CANAF->FCANIC1;
		head_idx = 32;
		tail_idx = 63;
	}

	if (interrut_word != 0)
	{
		/* Detect for interrupt pending */
		msg_idx = 0;
		for (msg_idx = head_idx; msg_idx <= tail_idx; msg_idx++)
		{
			test_bit = interrut_word & 0x1;
			interrut_word = interrut_word >> 1;

			if (test_bit)
			{
				pSrc = (uint32_t *) (CANAF->ENDofTable + CANAF_RAM_BASE + msg_idx * 12);

	    	 	/* Has been finished updating the content */
	    	 	if ((*pSrc & 0x03000000L) == 0x03000000L)
	    	 	{
	    	 		/*clear semaphore*/
	    	 		*pSrc &= 0xFCFFFFFF;

	    	 		/*Set to DatA*/
	    	 		pSrc++;
	    	 		/* Copy to dest buf */
	    	 		*((uint32_t *) &CAN_Msg->dataA) = *pSrc;

	    	 		/*Set to DatB*/
	    	 		pSrc++;
	    	 		/* Copy to dest buf */
	    	 		*((uint32_t *) &CAN_Msg->dataB) = *pSrc;

	    	 		/*Back to Dat1*/
	    	 		pSrc -= 2;

	    	 		CAN_Msg->id = *pSrc & 0x7FF;
	    	 		CAN_Msg->len = (uint8_t) (*pSrc >> 16) & 0x0F;
					CAN_Msg->format = 0; //FullCAN Object ID always is 11-bit value
					CAN_Msg->type = (uint8_t)(*pSrc >> 30) &0x01;
	    	 		/*Re-read semaphore*/
	    	 		if ((*pSrc & 0x03000000L) == 0)
	    	 		{
	    	 			return CAN_OK;
	    	 		}
	    	 	}
			}
		}
	}
	return CAN_FULL_OBJ_NOT_RCV;
}
/********************************************************************//**
 * @brief		Get CAN Control Status
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	arg: type of CAN status to get from CAN status register
 * 				Should be:
 * 				- CANCTRL_GLOBAL_STS: CAN Global Status
 * 				- CANCTRL_INT_CAP: CAN Interrupt and Capture
 * 				- CANCTRL_ERR_WRN: CAN Error Warning Limit
 * 				- CANCTRL_STS: CAN Control Status
 * @return 		Current Control Status that you want to get value
 *********************************************************************/
uint32_t CAN_GetCTRLStatus (CAN_TypeDef* CANx, CAN_CTRL_STS_Type arg)
{
	CHECK_PARAM(PARAM_CANx(CANx));
	CHECK_PARAM(PARAM_CTRL_STS_TYPE(arg));

	switch (arg)
	{
	case CANCTRL_GLOBAL_STS:
		return CANx->GSR;
		break;

	case CANCTRL_INT_CAP:
		return CANx->ICR;
		break;

	case CANCTRL_ERR_WRN:
		return CANx->EWL;
		break;

	default: // CANCTRL_STS
		return CANx->SR;
		break;
	}
}
/********************************************************************//**
 * @brief		Get CAN Central Status
 * @param[in]	CANCRx point to CANCR_TypeDef
 * @param[in]	arg: type of CAN status to get from CAN Central status register
 * 				Should be:
 * 				- CANCR_TX_STS: Central CAN Tx Status
 * 				- CANCR_RX_STS: Central CAN Rx Status
 * 				- CANCR_MS: Central CAN Miscellaneous Status
 * @return 		Current Central Status that you want to get value
 *********************************************************************/
uint32_t CAN_GetCRStatus (CANCR_TypeDef* CANCRx, CAN_CR_STS_Type arg)
{
	CHECK_PARAM(PARAM_CANCRx(CANCRx));
	CHECK_PARAM(PARAM_CR_STS_TYPE(arg));

	switch (arg)
	{
	case CANCR_TX_STS:
		return CANCRx->CANTxSR;
		break;

	case CANCR_RX_STS:
		return CANCRx->CANRxSR;
		break;

	default:	// CANCR_MS
		return CANCRx->CANMSR;
		break;
	}
}
/********************************************************************//**
 * @brief		Enable/Disable CAN Interrupt
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	arg: type of CAN interrupt that you want to enable/disable
 * 				Should be:
 * 				- CANINT_RIE: CAN Receiver Interrupt Enable
 * 				- CANINT_TIE1: CAN Transmit Interrupt Enable
 * 				- CANINT_EIE: CAN Error Warning Interrupt Enable
 * 				- CANINT_DOIE: CAN Data Overrun Interrupt Enable
 * 				- CANINT_WUIE: CAN Wake-Up Interrupt Enable
 * 				- CANINT_EPIE: CAN Error Passive Interrupt Enable
 * 				- CANINT_ALIE: CAN Arbitration Lost Interrupt Enable
 * 				- CANINT_BEIE: CAN Bus Error Interrupt Enable
 * 				- CANINT_IDIE: CAN ID Ready Interrupt Enable
 * 				- CANINT_TIE2: CAN Transmit Interrupt Enable for Buffer2
 * 				- CANINT_TIE3: CAN Transmit Interrupt Enable for Buffer3
 * 				- CANINT_FCE: FullCAN Interrupt Enable
 * @param[in]	NewState: New state of this function, should be:
 * 				- ENABLE
 * 				- DISABLE
 * @return 		none
 *********************************************************************/
void CAN_IRQCmd (CAN_TypeDef* CANx, CAN_INT_EN_Type arg, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_CANx(CANx));
	CHECK_PARAM(PARAM_INT_EN_TYPE(arg));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if(NewState == ENABLE)
	{
		if(arg==CANINT_FCE)
		{
			CANAF->AFMR = 0x01;
			CANAF->FCANIE = 0x01;
			CANAF->AFMR = 0x04;
		}
		else
			CANx->IER |= (1 << arg);
	}
	else
	{
		if(arg==CANINT_FCE){
			CANAF->AFMR = 0x01;
			CANAF->FCANIE = 0x01;
			CANAF->AFMR = 0x00;
		}
		else
			CANx->IER &= ~(1 << arg);
	}
}
/*********************************************************************//**
 * @brief		Install interrupt call-back function
 * @param[in]	arg: CAN interrupt type, should be:
 * 	  			- CANINT_RIE: CAN Receiver Interrupt Enable
 * 				- CANINT_TIE1: CAN Transmit Interrupt Enable
 * 				- CANINT_EIE: CAN Error Warning Interrupt Enable
 * 				- CANINT_DOIE: CAN Data Overrun Interrupt Enable
 * 				- CANINT_WUIE: CAN Wake-Up Interrupt Enable
 * 				- CANINT_EPIE: CAN Error Passive Interrupt Enable
 * 				- CANINT_ALIE: CAN Arbitration Lost Interrupt Enable
 * 				- CANINT_BEIE: CAN Bus Error Interrupt Enable
 * 				- CANINT_IDIE: CAN ID Ready Interrupt Enable
 * 				- CANINT_TIE2: CAN Transmit Interrupt Enable for Buffer2
 * 				- CANINT_TIE3: CAN Transmit Interrupt Enable for Buffer3
 * 				- CANINT_FCE: FullCAN Interrupt Enable
 * @param[in]	pnCANCbs: pointer point to call-back function
 * @return		None
 **********************************************************************/
void CAN_SetupCBS(CAN_INT_EN_Type arg,fnCANCbs_Type* pnCANCbs)
{
	CHECK_PARAM(PARAM_INT_EN_TYPE(arg));
	_apfnCANCbs[arg] = pnCANCbs;
}
/********************************************************************//**
 * @brief		Setting Acceptance Filter mode
 * @param[in]	CANAFx point to CANAF_TypeDef object, should be: CANAF
 * @param[in]	AFMode: type of AF mode that you want to set, should be:
 * 				- CAN_Normal: Normal mode
 * 				- CAN_AccOff: Acceptance Filter Off Mode
 * 				- CAN_AccBP: Acceptance Fileter Bypass Mode
 * 				- CAN_eFCAN: FullCAN Mode Enhancement
 * @return 		none
 *********************************************************************/
void CAN_SetAFMode (CANAF_TypeDef* CANAFx, CAN_AFMODE_Type AFMode)
{
	CHECK_PARAM(PARAM_CANAFx(CANAFx));
	CHECK_PARAM(PARAM_AFMODE_TYPE(AFMode));

	switch(AFMode)
	{
	case CAN_Normal:
		CANAFx->AFMR = 0x00;
		break;
	case CAN_AccOff:
		CANAFx->AFMR = 0x01;
		break;
	case CAN_AccBP:
		CANAFx->AFMR = 0x02;
		break;
	case CAN_eFCAN:
		CANAFx->AFMR = 0x04;
		break;
	}
}

/********************************************************************//**
 * @brief		Enable/Disable CAN Mode
 * @param[in]	CANx pointer to CAN_TypeDef, should be:
 * 				- CAN1: CAN 1
 * 				- CAN2: CAN 2
 * @param[in]	mode: type of CAN mode that you want to enable/disable, should be:
 * 				- CAN_OPERATING_MODE: Normal Operating Mode
 * 				- CAN_RESET_MODE: Reset Mode
 * 				- CAN_LISTENONLY_MODE: Listen Only Mode
 * 				- CAN_SELFTEST_MODE: Self Test Mode
 * 				- CAN_TXPRIORITY_MODE: Transmit Priority Mode
 * 				- CAN_SLEEP_MODE: Sleep Mode
 * 				- CAN_RXPOLARITY_MODE: Receive Polarity Mode
 * 				- CAN_TEST_MODE: Test Mode
 * @param[in]	NewState: New State of this function, should be:
 * 				- ENABLE
 * 				- DISABLE
 * @return 		none
 *********************************************************************/
void CAN_ModeConfig(CAN_TypeDef* CANx, CAN_MODE_Type mode, FunctionalState NewState)
{
	CHECK_PARAM(PARAM_CANx(CANx));
	CHECK_PARAM(PARAM_MODE_TYPE(mode));
	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	switch(mode)
	{
	case CAN_OPERATING_MODE:
		CANx->MOD = 0x00;
		break;
	case CAN_RESET_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=CAN_MOD_RM;
		else
			CANx->MOD &= ~CAN_MOD_RM;
		break;
	case CAN_LISTENONLY_MODE:
		CANx->MOD |=CAN_MOD_RM;
		if(NewState == ENABLE)
			CANx->MOD |=CAN_MOD_LOM;
		else
			CANx->MOD &=~CAN_MOD_LOM;
		break;
	case CAN_SELFTEST_MODE:
		CANx->MOD |=CAN_MOD_RM;
		if(NewState == ENABLE)
			CANx->MOD |=CAN_MOD_STM;
		else
			CANx->MOD &=~CAN_MOD_STM;
		break;
	case CAN_TXPRIORITY_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=CAN_MOD_TPM;
		else
			CANx->MOD &=~CAN_MOD_TPM;
		break;
	case CAN_SLEEP_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=CAN_MOD_SM;
		else
			CANx->MOD &=~CAN_MOD_SM;
		break;
	case CAN_RXPOLARITY_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=CAN_MOD_RPM;
		else
			CANx->MOD &=~CAN_MOD_RPM;
		break;
	case CAN_TEST_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=CAN_MOD_TM;
		else
			CANx->MOD &=~CAN_MOD_TM;
		break;
	}
}
/*********************************************************************//**
 * @brief		Standard CAN interrupt handler, this function will check
 * 				all interrupt status of CAN channels, then execute the call
 * 				back function if they're already installed
 * @param[in]	CANx point to CAN peripheral selected, should be: CAN1 or CAN2
 * @return		None
 **********************************************************************/
void CAN_IntHandler(CAN_TypeDef* CANx)
{
	uint8_t t;
	//scan interrupt pending
	if(CANAF->FCANIE)
	{
		_apfnCANCbs[11]();
		//clear FCANIE
		CANAF->AFMR = 0x01;
		CANAF->FCANIE = 0x00;
		CANAF->AFMR = 0x04;
	}
	else{
		for(t=0;t<11;t++)
		{
			if(((CANx->ICR)>>t)&0x01)
			{
				_apfnCANCbs[t]();
			}
		}
	}
}
/**
 * @}
 */
/**
 * @}
 */
#endif /* _I2S */

