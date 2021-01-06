#ifndef  __REMOTE_H__
#define __REMOTE_H__
#include "main.h"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
		struct 
		{
			uint16_t ch0;
			uint16_t ch1;
			uint16_t ch2;
			uint16_t ch3;
			uint8_t s1;
			uint8_t s2;
		}rc;
		
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
		}mouse;
		
		struct
		{
			uint16_t vl;
			uint16_t vh;  
		}key;
		
		uint16_t SW;
		
		uint8_t update;
		
}RC_Ctl_t;

void RemoteDataProcess(uint8_t *pData);
	
#endif

