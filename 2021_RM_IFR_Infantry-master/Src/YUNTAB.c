//---------头文件包含部分---------//
#include "YUNTAB.h"
#include "imu_analysis.h"
#include "can.h"
//--------------------------------//

//----------变量声明部分----------//
ROBO_Yuntab Robo_yun;
extern IMU_T IMU;
//--------------------------------//
/**********************************************************云台pid控制系统****************************************************************************************************/
//--------------------------------------------------------------------------------------------------//
//1.初始化云台
//
//函数名称:
//		云台参数初始化
//
//函数功能:
//		初始化云台所有的信息
//
//参数类型:
//		ROBO_Yuntab 指针, 云台结构体的指针
//
//移植建议:
//		有什么状态, 电机, 电机状态都先把数据封装进ROBO_Yuntab结构体里, 然后直接初始化就好了
//
//--------------------------------------------------------------------------------------------------//
void Yuntab_Init(ROBO_Yuntab *Robo_yun)       
{
  Pos_System* P_Pos=NULL;      //速度环信息和pid
  P_Pos=&Robo_yun->Pos_MotorHorizontal; PID_Init(&P_Pos->Speed_PID,	0.5,0,	0,	5000,	0,	5000,	5000);
																				PID_Init(&P_Pos->Angle_PID,	5,	0,	0,	5000,	0,	5000,	5000);
																				P_Pos->Motor_Num = 0;
  P_Pos=&Robo_yun->Pos_MotorVertical;		PID_Init(&P_Pos->Speed_PID,	0.5,0,	0,	5000,	0,	5000,	5000);
																				PID_Init(&P_Pos->Angle_PID,	5,	0,	0,	5000,	0,	5000,	5000);
																				P_Pos->Motor_Num = 1 ;
}
//--------------------------------------------------------------------------------------------------//
//2.获取当前速度环速度数据（pos入口）
//
//函数名称:
//		速度环电机数据分析的接口函数
//
//函数功能:
//		读取ROBO_Yuntab对应的CAN口储存的数据, 根据电机号码来分辨是哪一个电机的信息, 然后储存电机数据.
//
//参数类型:
//		ROBO_Yuntab 指针, 云台结构体的指针
//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
//		uint32_t 电机号码
//
//移植建议:
//		直接对case的数据进行修改, 有几个方向的电机就加几个, 然后让指针指向对应的电机就行.
//
//--------------------------------------------------------------------------------------------------//
void Motor_Pos_Analysis(ROBO_Yuntab* Robo_yun,uint8_t* RX_Data,uint32_t Motor_Num)
{
  Pos_System* P_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x205:P_Motor=&Robo_yun->Pos_MotorHorizontal;break;
    case 0x206:P_Motor=&Robo_yun->Pos_MotorVertical;break;
		default:break;
  }if(P_Motor!=NULL) Pos_Info_Analysis(&P_Motor->Info,RX_Data);
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
//		ROBO_Yuntab* 底盘结构体指针
//
//移植建议:
//		有需要啥环的控制就让指针指向这个系统, 然后调用对应的PID计算函数进行处理
//
//--------------------------------------------------------------------------------------------------//
void PID_Send_Yun(ROBO_Yuntab* Robo_yun)
{
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo_yun->Pos_MotorHorizontal; PID_Pos_Cal(P_Pos,Robo_yun->Tx_CAN1,IMU.quaternion.pitch);
  P_Pos=&Robo_yun->Pos_MotorVertical;   PID_Pos_Cal(P_Pos,Robo_yun->Tx_CAN1,IMU.quaternion.yaw);
  Send_To_Yun(&hcan1,Robo_yun->Tx_CAN1);
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
void Send_To_Yun(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox; 

  TxHeader.RTR = 0;
  TxHeader.IDE = 0;            
  TxHeader.StdId=0x1FF;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
        
  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, Tx_Data, &TxMailbox) != HAL_OK)
  {
   /* Transmission request Error */
     Error_Handler();
  }
	else HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
}

void Yuntab_contral(ROBO_Yuntab* Robo_yun,IMU_T* IMU)
{
	Robo_yun->Pos_MotorHorizontal.Tar_Pos = 0;
	Robo_yun->Pos_MotorVertical.Tar_Pos = 0;
	
	PID_Send_Yun(Robo_yun);
}
