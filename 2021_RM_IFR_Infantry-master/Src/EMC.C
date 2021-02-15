//---------头文件包含部分---------//
#include "EMC.h"
//--------------------------------//

//----------变量声明部分----------//

//--------------------------------//

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		PID参数初始化函数
//
//函数功能:
//		初始化系统PID参数
//
//参数类型:
//		PID* PID指针
//		float PID的Kp
//		float PID的Ki
//		float PID的Kd
//		float 误差最大值
//		float 死区
//		float 误差累积最大值
//		float 输出最大值
//
//--------------------------------------------------------------------------------------------------//
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max)           //参数赋值与初始化
{
	pid->Kp = Kp;                      
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->error_max = error_max;       //最大误差
	pid->output_max = output_max;         //最大输出
	pid->dead_line = dead_line;          //四死线
	
	pid->intergral_max = intergral_max;         //积分最大    
	
	pid->error = 0;                  //误差
	pid->error_last = 0;               //上一次误差
	pid->intergral = 0;                //积分
	pid->derivative = 0;              //求导
	pid->output = 0;                  //输出
}
//--------------------------------------------------------------------------------------------------//
//函数名称:
//		速度环电机数据分析的操作函数
//
//函数功能:
//		根据数据传进来的数据进行解析.
//		默认通信协议内容:  	Data[0]----电机速度高8位
//							Data[1]----电机速度低8位
//							Data[2]----转子角度高8位
//							Data[3]----转子角度低8位
//							Data[4]----电流大小高8位
//							Data[5]----电流大小低8位
//							Data[6]----温度
//							Data[7]----NULL
//
//参数类型:
//		Motor_Speed_Info* 速度环电机信息指针
//		uint8_t* 电机信息的数组
//
//移植建议:
//		大框架不需要改, 要改的话, 信息解析的地方根据通信协议来改就行.
//--------------------------------------------------------------------------------------------------//
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data)
{
  Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
  Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
  Motor->Temperature=RX_Data[6];
}
//--------------------------------------------------------------------------------------------------//
//函数名称:
//		位置环电机数据分析的操作函数
//
//函数功能:
//		根据数据传进来的数据进行解析, 并且计算出绝对角度和相对角度.
//		默认通信协议内容:  	Data[0]----电机速度高8位
//							Data[1]----电机速度低8位
//							Data[2]----转子角度高8位
//							Data[3]----转子角度低8位
//							Data[4]----电流大小高8位
//							Data[5]----电流大小低8位
//							Data[6]----温度
//							Data[7]----NULL
//
//参数类型:
//		Motor_Pos_Info* 位置环电机信息指针
//		uint8_t* 电机信息的数组
//
//移植建议:
//		大框架不需要改, 要改的话, 信息解析的地方根据通信协议来改就行.
//--------------------------------------------------------------------------------------------------//
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data)
{
  //数据解析
  Motor->Angle=(uint16_t)RX_Data[0]<<8|RX_Data[1];
  Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
  Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
  Motor->Temperature=RX_Data[6];

  //绝对角度计算
  if (Motor->Speed!=0)
  {
    int16_t Error=Motor->Angle-Motor->Last_Angle;
    Motor->Abs_Angle += Error;
    if (Error < -4096)Motor->Abs_Angle += 8192;
    else if (Error > 4096)  Motor->Abs_Angle -= 8192;
  }Motor->Last_Angle=Motor->Angle;

  //相对角度计算, 默认范围0-360
  if(Motor->Abs_Angle>=0) Motor->Relative_Angle=(Motor->Abs_Angle%ONE_CIRCLE)*360.0/ONE_CIRCLE;
  else Motor->Relative_Angle=360-((-Motor->Abs_Angle)%ONE_CIRCLE)*360.0/ONE_CIRCLE;
}
//--------------------------------------------------------------------------------------------------//
//函数名称:
//		PID计算函数
//
//函数功能:
//		进行PID计算
//
//参数类型:
//		PID* PID指针
//		float 当前值
//		float 目标值
//
//移植建议:
//		不管封装得怎么样, 建议保留该函数不要修改, 作为向下兼容或者检查错误的函数
//
//--------------------------------------------------------------------------------------------------//
float PID_General_Cal(PID *pid, float fdbV, float tarV)
{
	pid->error =  tarV - fdbV;
	if(pid->error > pid->error_max)
		pid->error = pid->error_max;
	if(pid->error < -pid->error_max)
		pid->error = -pid->error_max;
	if(pid->error > 0 && pid->error < pid->dead_line)
		pid->error = 0;
	if(pid->error < 0 && pid->error > pid->dead_line)
		pid->error = 0;
	
	pid->intergral = pid->intergral + pid->error;
	if(pid->intergral > pid->intergral_max)
		pid->intergral = pid->intergral_max;
	if(pid->intergral < -pid->intergral_max)
		pid->intergral = -pid->intergral_max;
	
	pid->derivative = pid->error - pid->error_last;
	pid->error_last = pid->error;
	
	pid->output = pid->Kp*pid->error + pid->Ki*pid->intergral + pid->Kd*pid->derivative;
	
	if(pid->output > pid->output_max)
		pid->output = pid->output_max;
	if(pid->output < -pid->output_max)
		pid->output = -pid->output_max;
	
	return pid->output;
}
//--------------------------------------------------------------------------------------------------//
//函数名称:
//		速度环PID计算函数
//
//函数功能:
//		进行速度环PID计算
//
//参数类型:
//		Speed_System* 速度环系统指针
//		uint8_t* 发送数据的数组
//
//--------------------------------------------------------------------------------------------------//
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg)
{
	Speed_Motor->Speed_PID.error =  Speed_Motor->Tar_Speed - Speed_Motor->Info.Speed;
	if(Speed_Motor->Speed_PID.error > Speed_Motor->Speed_PID.error_max)
		Speed_Motor->Speed_PID.error = Speed_Motor->Speed_PID.error_max;
	if(Speed_Motor->Speed_PID.error < -Speed_Motor->Speed_PID.error_max)
		Speed_Motor->Speed_PID.error = -Speed_Motor->Speed_PID.error_max;
	if(Speed_Motor->Speed_PID.error > 0 && Speed_Motor->Speed_PID.error < Speed_Motor->Speed_PID.dead_line)
		Speed_Motor->Speed_PID.error = 0;
	if(Speed_Motor->Speed_PID.error < 0 && Speed_Motor->Speed_PID.error > Speed_Motor->Speed_PID.dead_line)
		Speed_Motor->Speed_PID.error = 0;
	
	Speed_Motor->Speed_PID.intergral = Speed_Motor->Speed_PID.intergral + Speed_Motor->Speed_PID.error;
	if(Speed_Motor->Speed_PID.intergral > Speed_Motor->Speed_PID.intergral_max)
		Speed_Motor->Speed_PID.intergral = Speed_Motor->Speed_PID.intergral_max;
	if(Speed_Motor->Speed_PID.intergral < -Speed_Motor->Speed_PID.intergral_max)
		Speed_Motor->Speed_PID.intergral = -Speed_Motor->Speed_PID.intergral_max;
	
	Speed_Motor->Speed_PID.derivative = Speed_Motor->Speed_PID.error - Speed_Motor->Speed_PID.error_last;
	Speed_Motor->Speed_PID.error_last = Speed_Motor->Speed_PID.error;
	
	Speed_Motor->Speed_PID.output = Speed_Motor->Speed_PID.Kp*Speed_Motor->Speed_PID.error + Speed_Motor->Speed_PID.Ki*Speed_Motor->Speed_PID.intergral + Speed_Motor->Speed_PID.Kd*Speed_Motor->Speed_PID.derivative;
	
	if(Speed_Motor->Speed_PID.output > Speed_Motor->Speed_PID.output_max)
		Speed_Motor->Speed_PID.output = Speed_Motor->Speed_PID.output_max;
	if(Speed_Motor->Speed_PID.output < -Speed_Motor->Speed_PID.output_max)
		Speed_Motor->Speed_PID.output = -Speed_Motor->Speed_PID.output_max;
	
	Tx_msg[Speed_Motor->Motor_Num*2] = ((int16_t)Speed_Motor->Speed_PID.output)>>8;
	Tx_msg[Speed_Motor->Motor_Num*2+1] = (int16_t)Speed_Motor->Speed_PID.output;
}

void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg,uint8_t quaternion)       
{
	//位置pid
	Pos_Motor->Angle_PID.error =  Pos_Motor->Tar_Pos - quaternion;//误差
	if(Pos_Motor->Angle_PID.error > Pos_Motor->Angle_PID.error_max)	Pos_Motor->Angle_PID.error = Pos_Motor->Angle_PID.error_max;
	if(Pos_Motor->Angle_PID.error < -Pos_Motor->Angle_PID.error_max)	Pos_Motor->Angle_PID.error = -Pos_Motor->Angle_PID.error_max;
	
	if(Pos_Motor->Angle_PID.error > 0 && Pos_Motor->Angle_PID.error < Pos_Motor->Angle_PID.dead_line)	Pos_Motor->Angle_PID.error = 0;
	if(Pos_Motor->Angle_PID.error < 0 && Pos_Motor->Angle_PID.error > Pos_Motor->Angle_PID.dead_line)	Pos_Motor->Angle_PID.error = 0;
	
	Pos_Motor->Angle_PID.intergral = Pos_Motor->Angle_PID.intergral + Pos_Motor->Angle_PID.error;//积分
	if(Pos_Motor->Angle_PID.intergral > Pos_Motor->Angle_PID.intergral_max)	Pos_Motor->Angle_PID.intergral = Pos_Motor->Angle_PID.intergral_max;
	if(Pos_Motor->Angle_PID.intergral < -Pos_Motor->Angle_PID.intergral_max)	Pos_Motor->Angle_PID.intergral = -Pos_Motor->Angle_PID.intergral_max;
	
	Pos_Motor->Angle_PID.derivative = Pos_Motor->Angle_PID.error - Pos_Motor->Angle_PID.error_last;//微分
	Pos_Motor->Angle_PID.error_last = Pos_Motor->Angle_PID.error;
	
	Pos_Motor->Angle_PID.output = Pos_Motor->Angle_PID.Kp*Pos_Motor->Angle_PID.error + Pos_Motor->Angle_PID.Ki*Pos_Motor->Angle_PID.intergral + Pos_Motor->Angle_PID.Kd*Pos_Motor->Angle_PID.derivative;//输出
	if(Pos_Motor->Angle_PID.output > Pos_Motor->Angle_PID.output_max)	Pos_Motor->Angle_PID.output = Pos_Motor->Angle_PID.output_max;
	if(Pos_Motor->Angle_PID.output < -Pos_Motor->Angle_PID.output_max)	Pos_Motor->Angle_PID.output = -Pos_Motor->Angle_PID.output_max;
	
	//速度pid
		Pos_Motor->Speed_PID.error =  Pos_Motor->Angle_PID.output - Pos_Motor->Info.Speed;//误差	
	if(Pos_Motor->Speed_PID.error > Pos_Motor->Speed_PID.error_max)
		Pos_Motor->Speed_PID.error = Pos_Motor->Speed_PID.error_max;
	if(Pos_Motor->Speed_PID.error < -Pos_Motor->Speed_PID.error_max)
		Pos_Motor->Speed_PID.error = -Pos_Motor->Speed_PID.error_max;
	if(Pos_Motor->Speed_PID.error > 0 && Pos_Motor->Speed_PID.error < Pos_Motor->Speed_PID.dead_line)
		Pos_Motor->Speed_PID.error = 0;
	if(Pos_Motor->Speed_PID.error < 0 && Pos_Motor->Speed_PID.error > Pos_Motor->Speed_PID.dead_line)
		Pos_Motor->Speed_PID.error = 0;
	
	Pos_Motor->Speed_PID.intergral = Pos_Motor->Speed_PID.intergral + Pos_Motor->Speed_PID.error;//积分
	if(Pos_Motor->Speed_PID.intergral > Pos_Motor->Speed_PID.intergral_max)
		Pos_Motor->Speed_PID.intergral = Pos_Motor->Speed_PID.intergral_max;
	if(Pos_Motor->Speed_PID.intergral < -Pos_Motor->Speed_PID.intergral_max)
		Pos_Motor->Speed_PID.intergral = -Pos_Motor->Speed_PID.intergral_max;
	
	Pos_Motor->Speed_PID.derivative = Pos_Motor->Speed_PID.error - Pos_Motor->Speed_PID.error_last;//微分
	Pos_Motor->Speed_PID.error_last = Pos_Motor->Speed_PID.error;
	
	Pos_Motor->Speed_PID.output = Pos_Motor->Speed_PID.Kp*Pos_Motor->Speed_PID.error + Pos_Motor->Speed_PID.Ki*Pos_Motor->Speed_PID.intergral + Pos_Motor->Speed_PID.Kd*Pos_Motor->Speed_PID.derivative;//输出
	if(Pos_Motor->Speed_PID.output > Pos_Motor->Speed_PID.output_max)
		Pos_Motor->Speed_PID.output = Pos_Motor->Speed_PID.output_max;
	if(Pos_Motor->Speed_PID.output < -Pos_Motor->Speed_PID.output_max)
		Pos_Motor->Speed_PID.output = -Pos_Motor->Speed_PID.output_max;
	
	Tx_msg[Pos_Motor->Motor_Num*2]=((int16_t)Pos_Motor->Speed_PID.output)>>8;
	Tx_msg[Pos_Motor->Motor_Num*2+1]=(int16_t)Pos_Motor->Speed_PID.output;
}

