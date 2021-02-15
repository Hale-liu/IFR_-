//文件名称:		robo_base.c
//对应头文件:	robo_base.h
//主要功能:
//		基于大疆C620的电调与M3508电机封装起来的底盘函数库.
//		能够实现底盘信息的初始化, 电机反馈信息的分析, 与PID控制
//
//时间:
//		2020/11/13
//
//版本:	1.0V

//---------头文件引用部分---------//
#include "robo_base.h"
#include "can.h"
#include "Remote.h"
#include "imu_analysis.h"
//--------------------------------//

//---------变量声明部分-----------//
System_state system_state={WORKING,0};
//--------------------------------//

//---------外部变量声明部分-------//
extern RC_Ctl_t RC_CtrlData;
extern IMU_T IMU;
ROBO_BASE Robo;
//--------------------------------//
/**********************************************************底盘pid控制系统****************************************************************************************************/
//--------------------------------------------------------------------------------------------------//
//1.初始化底盘
//
//函数名称:
//		底盘参数初始化
//
//函数功能:
//		初始化底盘所有的信息
//
//参数类型:
//		ROBO_BASE 指针, 底盘结构体的指针
//
//移植建议:
//		有什么状态, 电机, 电机状态都先把数据封装进ROBO_BASE结构体里, 然后直接初始化就好了
//
//--------------------------------------------------------------------------------------------------//
void BASE_Init(ROBO_BASE *Robo)       
{
  Speed_System* P_Speed=NULL;      //速度环信息和pid
  P_Speed=&Robo->Speed_MotorLeft;  PID_Init(&P_Speed->Speed_PID,	5,	0,	0,	5000,	0,	5000,	5000); P_Speed->Motor_Num=0;
  P_Speed=&Robo->Speed_MotorRight; PID_Init(&P_Speed->Speed_PID,	5,	0,	0,	5000,	0,	5000,	5000); P_Speed->Motor_Num=1;
	Balance_System* P_Balance=NULL;
	P_Balance=&Robo->Balance_system; PID_Init(&P_Balance->Balance_PID,	5,	0,	0,	5000,	0,	5000,	5000);
}
//--------------------------------------------------------------------------------------------------//
//2.获取当前速度环速度数据（speed入口）
//
//函数名称:
//		速度环电机数据分析的接口函数
//
//函数功能:
//		读取Robo_Base对应的CAN口储存的数据, 根据电机号码来分辨是哪一个轮子的信息, 然后储存电机数据.
//
//参数类型:
//		ROBO_BASE 指针, 底盘结构体的指针
//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
//		uint32_t 电机号码
//
//移植建议:
//		直接对case的数据进行修改, 有几个速度环的轮子就加几个, 然后让指针指向对应的轮子就行.
//
//--------------------------------------------------------------------------------------------------//
void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num)
{
  Speed_System* S_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x201:S_Motor=&Robo->Speed_MotorLeft;break;
    case 0x202:S_Motor=&Robo->Speed_MotorRight;break;
		default:break;
  }if(S_Motor!=NULL) Speed_Info_Analysis(&S_Motor->Info,RX_Data);
}
//--------------------------------------------------------------------------------------------------//
//3.发送结果给电机（出口）
//
//函数名称:
//		PID发送函数
//
//函数功能:
//		发送电机PID
//
//参数类型:
//		ROBO_BASE* 底盘结构体指针
//
//移植建议:
//		有需要啥环的控制就让指针指向这个系统, 然后调用对应的PID计算函数进行处理
//
//--------------------------------------------------------------------------------------------------//
void PID_Send_Base(ROBO_BASE* Robo)
{
  Speed_System* P_Speed=NULL;
  P_Speed=&Robo->Speed_MotorLeft; PID_Speed_Cal(P_Speed,Robo->Tx_CAN2);
  P_Speed=&Robo->Speed_MotorRight; PID_Speed_Cal(P_Speed,Robo->Tx_CAN2);
  Send_To_Motor(&hcan2,Robo->Tx_CAN2);
}
//--------------------------------------------------------------------------------------------------//
//函数名称:
//		//CAN通信发送函数
//
//函数功能:
//		发送数据
//
//参数类型:
//		CAN_HandleTypeDef* CAN的句柄
//		uint8_t* 发送数据的数组
//
//移植建议:
//		根据要求修改标识符就行
//--------------------------------------------------------------------------------------------------//
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox; 

  TxHeader.RTR = 0;
  TxHeader.IDE = 0;            
  TxHeader.StdId=0x200;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
        
  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, Tx_Data, &TxMailbox) != HAL_OK)
  {
   /* Transmission request Error */
     Error_Handler();
  }
}

void speed_distribution(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU)//正常模式
{
	mpu_get_data();
	
	Robo->Speed_X=(RC_CtrlData->ch0-1024)*4000.0f/660.0f;
	Robo->Speed_Y=(RC_CtrlData->ch1-1024)*4000.0f/660.0f;
	
	Robo->Balance_system.Tar_Angle = 0;

	Robo->Speed_MotorLeft.Tar_Speed  = +(PID_General_Cal(&Robo->Balance_system.Balance_PID, IMU->quaternion.pitch, Robo->Balance_system.Tar_Angle) + Robo->Speed_X + Robo->Speed_Y);
	Robo->Speed_MotorRight.Tar_Speed = -(PID_General_Cal(&Robo->Balance_system.Balance_PID, IMU->quaternion.pitch, Robo->Balance_system.Tar_Angle) + Robo->Speed_X - Robo->Speed_Y);
}

void Base_Contral(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU)//计算pid的输出和发送
{
	switch(RC_CtrlData->s1)
	{
		case 2:speed_distribution(Robo,RC_CtrlData,IMU);break;
	}
	PID_Send_Base(Robo);
}

void System_check(System_state *system_state)
{
	if(system_state->count_time<1000) system_state->count_time++;
	else system_state->State=MISSING;
}

void Feed_dog(System_state *system_state)
{
	system_state->count_time=0;
}

