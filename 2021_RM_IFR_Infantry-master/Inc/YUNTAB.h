#ifndef __YUNTAB_H__
#define __YUNTAB_H__
//---------ͷ�ļ���������----------//
#include "main.h"
#include "EMC.h"
#include "imu_analysis.h"
//---------------------------------//

//---------#define����-------------//

//---------------------------------//

//---------�ⲿ������������-------//

//--------------------------------//

//---------��̨�ṹ�岿��----------//
typedef struct ROBO_Yuntab			//���̽ṹ��
{
	Pos_System Pos_MotorHorizontal;			//λ�û�--ˮƽת��
	Pos_System Pos_MotorVertical;		//λ�û�--��ֱת��

	uint8_t Tx_CAN1[8];				//CAN1ͨ�ŷ�������
	uint8_t Rx_CAN1[8];				//CAN1ͨ�Ž�������
}ROBO_Yuntab;

void Yuntab_Init(ROBO_Yuntab *Robo_yun) ;
void Motor_Pos_Analysis(ROBO_Yuntab* Robo_yun,uint8_t* RX_Data,uint32_t Motor_Num);
void PID_Send_Yun(ROBO_Yuntab* Robo_yun);
void Send_To_Yun(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);
void Yuntab_contral(ROBO_Yuntab* Robo_yun,IMU_T* IMU);

#endif

