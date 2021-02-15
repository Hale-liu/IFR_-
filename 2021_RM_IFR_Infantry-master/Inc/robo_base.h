#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------头文件包含部分----------//
#include "main.h"
#include "EMC.h"
#include "math.h"
#include "Remote.h"
#include "imu_analysis.h"
//---------------------------------//

//---------#define部分-------------//
#define WORKING 1
#define MISSING 0
//---------------------------------//

//---------底盘结构体部分----------//
typedef struct Balance_System		//平衡系统
{
	PID Balance_PID;		//平衡系统pid
	float Tar_Angle;		//平衡状态目标角度
}Balance_System;

typedef struct Robo_Base			//底盘结构体
{
	Speed_System Speed_MotorLeft;			//速度环--左轮
	Speed_System Speed_MotorRight;		//速度环--右轮
	Balance_System Balance_system;		//平衡系统

	int Speed_X;					//底盘X方向上目标速度
	int Speed_Y;					//底盘Y方向上目标速度

	uint8_t Tx_CAN2[8];				//CAN2通信发送数据
	uint8_t Rx_CAN2[8];				//CAN2通信接收数据
}ROBO_BASE;

typedef struct System_state
{
	uint8_t State;
	uint16_t count_time;
}System_state;

//---------------------------------//

//-------------函数声明------------//
void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//速度环电机数据分析的接口函数
void BASE_Init(ROBO_BASE *Robo);																									//底盘PID参数初始化的接口函数
void PID_Send_Base(ROBO_BASE* Robo);																//PID发送函数
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);								//CAN通信发送函数

void Base_Contral(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU);
void speed_distribution(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU);

void Feed_dog(System_state *system_state);
void System_check(System_state *system_state);
//---------------------------------//
#endif


