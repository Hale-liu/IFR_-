#include "math.h"
#include "Kalman_Filter.h"

float PI = 3.1415;
float K1 =0.02; 
float angle, angle_dot; 	
int16_t angle0=0;
float Q_angle=0.001;
float Q_gyro=0.003;	
float R_angle=0.5;	
float dt=0.001;   	
char  C_0 = 1;
float Q_bias, Angle_err;  
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float P_temp[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

float Kalman_Filter(float Accel,float Gyro)		
{
     float Angle;
     Angle+=(Gyro - Q_bias) * dt; 
    

	P_temp[0]=Q_angle - PP[0][1] - PP[1][0]; 
	P_temp[1]=-PP[1][1];
	P_temp[2]=-PP[1][1];
	P_temp[3]=Q_gyro;

	PP[0][0] += P_temp[0] * dt;   
	PP[0][1] += P_temp[1] * dt;   
	PP[1][0] += P_temp[2] * dt;
	PP[1][1] += P_temp[3] * dt;

  //PP[0][0] += Q_angle-(PP[0][1]+PP[1][0])*dt+PP[1][1]*dt*dt; 
	//PP[0][1] += -PP[1][1] * dt;
	//PP[1][0] += -PP[1][1] * dt;
	//PP[1][1] += Q_gyro;

	PCt_0 = C_0 * PP[0][0]; 
	PCt_1 = C_0 * PP[1][0]; 
	
	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	Angle_err = Accel - Angle;   

	Angle    += K_0 * Angle_err;  
    Q_bias   += K_1 * Angle_err;    

    angle_dot    = Gyro - Q_bias;      
    
	   
	t_0 = PCt_0; 		
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;        
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
		
    return Angle;
		
}

