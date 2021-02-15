//�ļ�����:		robo_base.c
//��Ӧͷ�ļ�:	robo_base.h
//��Ҫ����:
//		���ڴ�C620�ĵ����M3508�����װ�����ĵ��̺�����.
//		�ܹ�ʵ�ֵ�����Ϣ�ĳ�ʼ��, ���������Ϣ�ķ���, ��PID����
//
//ʱ��:
//		2020/11/13
//
//�汾:	1.0V

//---------ͷ�ļ����ò���---------//
#include "robo_base.h"
#include "can.h"
#include "Remote.h"
#include "imu_analysis.h"
//--------------------------------//

//---------������������-----------//
System_state system_state={WORKING,0};
//--------------------------------//

//---------�ⲿ������������-------//
extern RC_Ctl_t RC_CtrlData;
extern IMU_T IMU;
ROBO_BASE Robo;
//--------------------------------//
/**********************************************************����pid����ϵͳ****************************************************************************************************/
//--------------------------------------------------------------------------------------------------//
//1.��ʼ������
//
//��������:
//		���̲�����ʼ��
//
//��������:
//		��ʼ���������е���Ϣ
//
//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//
//��ֲ����:
//		��ʲô״̬, ���, ���״̬���Ȱ����ݷ�װ��ROBO_BASE�ṹ����, Ȼ��ֱ�ӳ�ʼ���ͺ���
//
//--------------------------------------------------------------------------------------------------//
void BASE_Init(ROBO_BASE *Robo)       
{
  Speed_System* P_Speed=NULL;      //�ٶȻ���Ϣ��pid
  P_Speed=&Robo->Speed_MotorLeft;  PID_Init(&P_Speed->Speed_PID,	5,	0,	0,	5000,	0,	5000,	5000); P_Speed->Motor_Num=0;
  P_Speed=&Robo->Speed_MotorRight; PID_Init(&P_Speed->Speed_PID,	5,	0,	0,	5000,	0,	5000,	5000); P_Speed->Motor_Num=1;
	Balance_System* P_Balance=NULL;
	P_Balance=&Robo->Balance_system; PID_Init(&P_Balance->Balance_PID,	5,	0,	0,	5000,	0,	5000,	5000);
}
//--------------------------------------------------------------------------------------------------//
//2.��ȡ��ǰ�ٶȻ��ٶ����ݣ�speed��ڣ�
//
//��������:
//		�ٶȻ�������ݷ����Ľӿں���
//
//��������:
//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
//
//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
//		uint32_t �������
//
//��ֲ����:
//		ֱ�Ӷ�case�����ݽ����޸�, �м����ٶȻ������ӾͼӼ���, Ȼ����ָ��ָ���Ӧ�����Ӿ���.
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
//3.���ͽ������������ڣ�
//
//��������:
//		PID���ͺ���
//
//��������:
//		���͵��PID
//
//��������:
//		ROBO_BASE* ���̽ṹ��ָ��
//
//��ֲ����:
//		����Ҫɶ���Ŀ��ƾ���ָ��ָ�����ϵͳ, Ȼ����ö�Ӧ��PID���㺯�����д���
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
//��������:
//		//CANͨ�ŷ��ͺ���
//
//��������:
//		��������
//
//��������:
//		CAN_HandleTypeDef* CAN�ľ��
//		uint8_t* �������ݵ�����
//
//��ֲ����:
//		����Ҫ���޸ı�ʶ������
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

void speed_distribution(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU)//����ģʽ
{
	mpu_get_data();
	
	Robo->Speed_X=(RC_CtrlData->ch0-1024)*4000.0f/660.0f;
	Robo->Speed_Y=(RC_CtrlData->ch1-1024)*4000.0f/660.0f;
	
	Robo->Balance_system.Tar_Angle = 0;

	Robo->Speed_MotorLeft.Tar_Speed  = +(PID_General_Cal(&Robo->Balance_system.Balance_PID, IMU->quaternion.pitch, Robo->Balance_system.Tar_Angle) + Robo->Speed_X + Robo->Speed_Y);
	Robo->Speed_MotorRight.Tar_Speed = -(PID_General_Cal(&Robo->Balance_system.Balance_PID, IMU->quaternion.pitch, Robo->Balance_system.Tar_Angle) + Robo->Speed_X - Robo->Speed_Y);
}

void Base_Contral(ROBO_BASE* Robo,RC_Ctl_t* RC_CtrlData,IMU_T* IMU)//����pid������ͷ���
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

