#ifndef __YUNTAB_H__
#define __YUNTAB_H__
//---------头文件包含部分----------//
#include "main.h"
#include "EMC.h"
#include "imu_analysis.h"
//---------------------------------//

//---------#define部分-------------//

//---------------------------------//

//---------外部变量声明部分-------//

//--------------------------------//

//---------云台结构体部分----------//
typedef struct ROBO_Yuntab			//底盘结构体
{
	Pos_System Pos_MotorHorizontal;			//位置环--水平转动
	Pos_System Pos_MotorVertical;		//位置环--竖直转动

	uint8_t Tx_CAN1[8];				//CAN1通信发送数据
	uint8_t Rx_CAN1[8];				//CAN1通信接收数据
}ROBO_Yuntab;

void Yuntab_Init(ROBO_Yuntab *Robo_yun) ;
void Motor_Pos_Analysis(ROBO_Yuntab* Robo_yun,uint8_t* RX_Data,uint32_t Motor_Num);
void PID_Send_Yun(ROBO_Yuntab* Robo_yun);
void Send_To_Yun(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);
void Yuntab_contral(ROBO_Yuntab* Robo_yun,IMU_T* IMU);

#endif

