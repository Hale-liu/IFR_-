#include "init.h"

void All_Init(void)
{
	Power_Init();
	Laser_Run();
	Led_Run();
	
	Usart_Init();
	
	HAL_TIM_Base_Start_IT(&htim2);
	
	CAN_FilterConfig(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
	
	CAN_FilterConfig(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	
	BASE_Init(&Robo);
	Yuntab_Init(&Robo_yun);
	
	mpu_device_init();
}

