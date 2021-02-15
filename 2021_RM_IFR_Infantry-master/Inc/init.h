#ifndef __INIT_H__
#define __INIT_H__

#include "gpio.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "robo_base.h"
#include "YUNTAB.h"
#include "imu_analysis.h"
#include "main.h"

extern ROBO_BASE Robo;
extern ROBO_Yuntab Robo_yun;

void All_Init(void);

#endif

