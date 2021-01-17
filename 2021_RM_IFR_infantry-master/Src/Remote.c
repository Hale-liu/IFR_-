#include "Remote.h"
#include "robo_base.h"

RC_Ctl_t RC_CtrlData=
{1024,1024,1024,1024,2,2};


void RemoteDataProcess(uint8_t *pData) 
{     	
	if(pData == 0)     
	{         
		return;     
	}          
	RC_CtrlData.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;      
	RC_CtrlData.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;    
	RC_CtrlData.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;     
	RC_CtrlData.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;          
	RC_CtrlData.s1 = ((pData[5] >> 4) & 0x000C) >> 2;     
	RC_CtrlData.s2 = ((pData[5] >> 4) & 0x0003); 
}




	
