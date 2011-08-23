/*
 * fattime.c
 *
 *  Created on: 09.07.2010
 *      Author: mthomas
 */

#define RTC_CAL_AVAILABLE     0

#include "integer.h"

#if RTC_CAL_AVAILABLE

#include "rtc_cal.h"

DWORD get_fattime (void)
{
	RTC_TIME_Type time_curr;
	DWORD ret;

	rtc_cal_gettime(&time_curr);

	/* Pack date and time into a DWORD variable */
	ret =
		((DWORD)(time_curr.YEAR - 1980) << 25)
		| ((DWORD)time_curr.MONTH << 21)
		| ((DWORD)time_curr.DOM << 16)
		| ((DWORD)time_curr.HOUR << 11)
		| ((DWORD)time_curr.MIN << 5)
		| ((DWORD)time_curr.SEC >> 1);

	return ret;
}

#else
/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */
/* This is not required in read-only configuration.        */

DWORD get_fattime (void)
{
	/* If no RTC support. Return a fixed value 2011/8/23 12:34:56 */
	return	  ((DWORD)(2011 - 1980) << 25)	/* Y */
			| ((DWORD)8  << 21)				/* M */
			| ((DWORD)23 << 16)				/* D */
			| ((DWORD)12  << 11)				/* H */
			| ((DWORD)34  << 5)				/* M */
			| ((DWORD)56  >> 1);				/* S */
}

#endif

