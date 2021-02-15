//---------ͷ�ļ���������---------//
#include "YUNTAB.h"
#include "imu_analysis.h"
#include "can.h"
//--------------------------------//

//----------������������----------//
ROBO_Yuntab Robo_yun;
extern IMU_T IMU;
//--------------------------------//
/**********************************************************��̨pid����ϵͳ****************************************************************************************************/
//--------------------------------------------------------------------------------------------------//
//1.��ʼ����̨
//
//��������:
//		��̨������ʼ��
//
//��������:
//		��ʼ����̨���е���Ϣ
//
//��������:
//		ROBO_Yuntab ָ��, ��̨�ṹ���ָ��
//
//��ֲ����:
//		��ʲô״̬, ���, ���״̬���Ȱ����ݷ�װ��ROBO_Yuntab�ṹ����, Ȼ��ֱ�ӳ�ʼ���ͺ���
//
//--------------------------------------------------------------------------------------------------//
void Yuntab_Init(ROBO_Yuntab *Robo_yun)       
{
  Pos_System* P_Pos=NULL;      //�ٶȻ���Ϣ��pid
  P_Pos=&Robo_yun->Pos_MotorHorizontal; PID_Init(&P_Pos->Speed_PID,	0.5,0,	0,	5000,	0,	5000,	5000);
																				PID_Init(&P_Pos->Angle_PID,	5,	0,	0,	5000,	0,	5000,	5000);
																				P_Pos->Motor_Num = 0;
  P_Pos=&Robo_yun->Pos_MotorVertical;		PID_Init(&P_Pos->Speed_PID,	0.5,0,	0,	5000,	0,	5000,	5000);
																				PID_Init(&P_Pos->Angle_PID,	5,	0,	0,	5000,	0,	5000,	5000);
																				P_Pos->Motor_Num = 1 ;
}
//--------------------------------------------------------------------------------------------------//
//2.��ȡ��ǰ�ٶȻ��ٶ����ݣ�pos��ڣ�
//
//��������:
//		�ٶȻ�������ݷ����Ľӿں���
//
//��������:
//		��ȡROBO_Yuntab��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ���������Ϣ, Ȼ�󴢴�������.
//
//��������:
//		ROBO_Yuntab ָ��, ��̨�ṹ���ָ��
//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
//		uint32_t �������
//
//��ֲ����:
//		ֱ�Ӷ�case�����ݽ����޸�, �м�������ĵ���ͼӼ���, Ȼ����ָ��ָ���Ӧ�ĵ������.
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
//3.���ͽ������������ڣ�
//
//��������:
//		PID���ͺ���
//
//��������:
//		���͵��PID
//
//��������:
//		ROBO_Yuntab* ���̽ṹ��ָ��
//
//��ֲ����:
//		����Ҫɶ���Ŀ��ƾ���ָ��ָ�����ϵͳ, Ȼ����ö�Ӧ��PID���㺯�����д���
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
