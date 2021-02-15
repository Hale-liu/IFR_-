#ifndef __EMC_H__
#define __EMC_H__
//---------头文件包含部分----------//
#include "main.h"
#include "imu_analysis.h"
//---------------------------------//

//---------#define部分-------------//
#define PI (2*acos(0))							//PI圆周率的宏定义
#define ToDegree(a) (a/PI*180)					//弧度转化成角度的宏定义
#define ToRadian(a) (a/180*PI)					//角度转化成弧度的宏定义
#define TX_LENGTH 20							//上下位机通信字符总长度
#define ROTOR_ANGLE 8192						//转子机械角度
#define GEAR_RATIO 19							//电机减速比
#define ONE_CIRCLE (ROTOR_ANGLE*GEAR_RATIO)		//电机转动一圈的总机械角度
//---------------------------------//

//---------外部变量声明部分-------//
extern IMU_T IMU;
//--------------------------------//
typedef struct pid_init_val{		//电机PID参数结构体
	
	float Kp;
	float Ki;
	float Kd;
	
	float error;					//误差
	float error_last;				//上一次误差
	float error_max;				//最大误差
	float dead_line;				//死区
	
	float intergral;				//误差积分
	float intergral_max;			//误差积分最大值
	
	float derivative;				//误差微分
	
	float output;					//输出
	float output_max;				//输出最大值
	
}PID;

typedef struct Motor_Pos_Info		//进行位置环控制的电机信息
{
  int16_t Speed;					//电机速度				单位(rad/min 转/每分钟)
  uint16_t Angle;					//转子机械角度
  int32_t Abs_Angle;				//转子绝对机械角度
  float Relative_Angle;				//电机相对坐标角度		单位(° 度)
  uint8_t Temperature;				//电机温度				单位(℃ 摄氏度)
  int16_t Electric;					//电流					单位(mA 毫安)
  uint16_t Last_Angle;				//上一次的转子绝对角度
}Motor_Pos_Info;

typedef struct Motor_Speed_Info		//进行速度环控制的电机信息
{
  int16_t Speed;					//电机速度				单位(rad/min 转/每分钟)
  uint8_t Temperature;				//电机温度				单位(℃ 摄氏度)
  int16_t Electric;					//电流					单位(mA 毫安)
}Motor_Speed_Info;

typedef struct Speed_System			//速度环系统
{
  Motor_Speed_Info Info;			//速度环电机信息
  PID Speed_PID;					//速度环PID参数
  float Tar_Speed;					//目标速度
  uint8_t Motor_Num;				//电机号码
}Speed_System;


typedef struct Pos_System			//位置环系统
{
  Motor_Pos_Info Info;				//位置环电机信息
	PID Angle_PID;		//位置环PID参数
  PID Speed_PID;					//速度环PID参数
  float Tar_Pos;					//目标位置
  uint8_t Motor_Num;				//电机号码
}Pos_System;

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);           //参数赋值与初始化
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data);
float PID_General_Cal(PID *pid, float fdbV, float tarV);
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg);
void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg,uint8_t quaternion);
#endif

