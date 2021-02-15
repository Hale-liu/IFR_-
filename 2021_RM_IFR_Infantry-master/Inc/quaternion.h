#ifndef __QUANTERNION_H
#define __QUANTERNION_H

#include "main.h"
#include "imu_analysis.h"



void imuUpdate(Axis3f acc, Axis3f gyro, float dt);
void quaternion_init(float angle_pit, float angle_rol);

#endif
