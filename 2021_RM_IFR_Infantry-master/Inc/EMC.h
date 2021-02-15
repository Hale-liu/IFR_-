#ifndef __EMC_H__
#define __EMC_H__
//---------ͷ�ļ���������----------//
#include "main.h"
#include "imu_analysis.h"
//---------------------------------//

//---------#define����-------------//
#define PI (2*acos(0))							//PIԲ���ʵĺ궨��
#define ToDegree(a) (a/PI*180)					//����ת���ɽǶȵĺ궨��
#define ToRadian(a) (a/180*PI)					//�Ƕ�ת���ɻ��ȵĺ궨��
#define TX_LENGTH 20							//����λ��ͨ���ַ��ܳ���
#define ROTOR_ANGLE 8192						//ת�ӻ�е�Ƕ�
#define GEAR_RATIO 19							//������ٱ�
#define ONE_CIRCLE (ROTOR_ANGLE*GEAR_RATIO)		//���ת��һȦ���ܻ�е�Ƕ�
//---------------------------------//

//---------�ⲿ������������-------//
extern IMU_T IMU;
//--------------------------------//
typedef struct pid_init_val{		//���PID�����ṹ��
	
	float Kp;
	float Ki;
	float Kd;
	
	float error;					//���
	float error_last;				//��һ�����
	float error_max;				//������
	float dead_line;				//����
	
	float intergral;				//������
	float intergral_max;			//���������ֵ
	
	float derivative;				//���΢��
	
	float output;					//���
	float output_max;				//������ֵ
	
}PID;

typedef struct Motor_Pos_Info		//����λ�û����Ƶĵ����Ϣ
{
  int16_t Speed;					//����ٶ�				��λ(rad/min ת/ÿ����)
  uint16_t Angle;					//ת�ӻ�е�Ƕ�
  int32_t Abs_Angle;				//ת�Ӿ��Ի�е�Ƕ�
  float Relative_Angle;				//����������Ƕ�		��λ(�� ��)
  uint8_t Temperature;				//����¶�				��λ(�� ���϶�)
  int16_t Electric;					//����					��λ(mA ����)
  uint16_t Last_Angle;				//��һ�ε�ת�Ӿ��ԽǶ�
}Motor_Pos_Info;

typedef struct Motor_Speed_Info		//�����ٶȻ����Ƶĵ����Ϣ
{
  int16_t Speed;					//����ٶ�				��λ(rad/min ת/ÿ����)
  uint8_t Temperature;				//����¶�				��λ(�� ���϶�)
  int16_t Electric;					//����					��λ(mA ����)
}Motor_Speed_Info;

typedef struct Speed_System			//�ٶȻ�ϵͳ
{
  Motor_Speed_Info Info;			//�ٶȻ������Ϣ
  PID Speed_PID;					//�ٶȻ�PID����
  float Tar_Speed;					//Ŀ���ٶ�
  uint8_t Motor_Num;				//�������
}Speed_System;


typedef struct Pos_System			//λ�û�ϵͳ
{
  Motor_Pos_Info Info;				//λ�û������Ϣ
	PID Angle_PID;		//λ�û�PID����
  PID Speed_PID;					//�ٶȻ�PID����
  float Tar_Pos;					//Ŀ��λ��
  uint8_t Motor_Num;				//�������
}Pos_System;

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);           //������ֵ���ʼ��
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data);
float PID_General_Cal(PID *pid, float fdbV, float tarV);
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg);
void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg,uint8_t quaternion);
#endif

