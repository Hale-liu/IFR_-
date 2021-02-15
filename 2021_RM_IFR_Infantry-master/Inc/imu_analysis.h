#ifndef __SPI_ANALYSIS__H
#define __SPI_ANALYSIS__H

#include "main.h"

typedef struct
{
	float x;
	float y;
	float z;
}Axis3f;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
	
	int16_t mx_offset;
	int16_t my_offset;
	int16_t mz_offset;
} mpu_data_t;



typedef struct
{
	struct
	{
		float pitch;
		float yaw;
		float roll;
	}quaternion;
	
		struct
	{
		float pitch;
		float yaw;
		float roll;
	}eular_acc;
	
	struct
	{
	  int16_t x;
	  int16_t y;
	  int16_t z;
	}m;	
	
  Axis3f gyro;	
  Axis3f acc;	

	float Yaw_Ingetral;
	float Pit_Ingetral;
	float Rol_Ingetral;		
	float temp;
	
	float norm_g;
	float Gravity;
	
} IMU_T;



extern mpu_data_t mpu_data;

void mpu_get_data(void);
void  mpu_device_init(void);
void mpu_offset_call(void);
void imu_attitude_update(void);
uint16_t Imu_TempControl(float temp);
uint8_t Test_Temp(void);
float Filter_one(float data,float Kp);  //Ò»½ÚÂË²¨

#endif
