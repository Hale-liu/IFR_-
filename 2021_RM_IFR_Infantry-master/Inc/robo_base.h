#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------ͷ�ļ���������----------//
#include "main.h"
#include "EMC.h"
#include "math.h"
#include "Remote.h"
#include "imu_analysis.h"
//---------------------------------//

//---------#define����-------------//
#define WORKING 1
#define MISSING 0
//---------------------------------//

//---------���̽ṹ�岿��----------//
typedef struct Balance_System		//ƽ��ϵͳ
{
	PID Balance_PID;		//ƽ��ϵͳpid
	float Tar_Angle;		//ƽ��״̬Ŀ��Ƕ�
}Balance_System;

typedef struct Robo_Base			//���̽ṹ��
{
	Speed_System Speed_MotorLeft;			//�ٶȻ�--����
	Speed_System Speed_MotorRight;		//�ٶȻ�--����
	Balance_System Balance_system;		//ƽ��ϵͳ

	int Speed_X;					//����X������Ŀ���ٶ�
	int Speed_Y;					//����Y������Ŀ���ٶ�

	uint8_t Tx_CAN2[8];				//CAN2ͨ�ŷ�������
	uint8_t Rx_CAN2[8];				//CAN2ͨ�Ž�������
}ROBO_BASE;

typedef struct System_state
{
	uint8_t State;
	uint16_t count_time;
}System_state;

//---------------------------------//

//-------------��������------------//
void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//�ٶȻ�������ݷ����Ľӿں���
void BASE_Init(ROBO_BASE *Robo);																									//����PID������ʼ���Ľӿں���
void PID_Send_Base(ROBO_BASE* Robo);																//PID���ͺ���
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);								//CANͨ�ŷ��ͺ���

void Base_Contral(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU);
void speed_distribution(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU);

void Feed_dog(System_state *system_state);
void System_check(System_state *system_state);
//---------------------------------//
#endif


