#include "imu_analysis.h"
#include "ist8310_reg.h" 
#include "math.h"
#include "mpu6500_reg.h"
#include "spi.h"
#include "quaternion.h"
#include "string.h"

#define BOARD_DOWN (1)   
#define IST8310
#define MPU_HSPI hspi5
#define MPU_NSS_LOW     	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET)
#define MPU_NSS_HIGH      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET)
#define abs(x) 						((x)>0? (x):(-(x)))

#define GYRO_MAX					1000.0f

//PID_TypeDef 				pid_temperature;

static uint8_t        		tx, rx;
uint8_t               		ist_buff[6];                           /* buffer to save IST8310 raw data */
mpu_data_t            		mpu_data;
IMU_T                 		IMU={0};
uint8_t 									id = 0;
float                 		Init_G = 0.0;
long 											a_offset_total[3];
long 											m_offset_total[3];
//u32 											temp_control = 0;


/**
  * @brief  ���ټ��㷴ƽ���� 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}

/**
  * @brief  ��һ���ֽڵ�����д��ָ���ļĴ���
  * @param  reg:  ����д��ע���ַ
  *         data: Ҫд�������
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
		rx = SPI5_ReadWriteByte(tx);
	  tx = data;
		rx = SPI5_ReadWriteByte(tx);
	  MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  ��ָ���ļĴ����ж�ȡһ���ֽڵ�����
  * @param  reg: Ҫ��ȡ�ļĴ�����ַ
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
		rx = SPI5_ReadWriteByte(tx);
		rx = SPI5_ReadWriteByte(tx);
  	MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  ��ָ���Ĵ�����ȡ�����ֽ�
  * @param  reg: ����д���ַ
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t  regAddr, uint8_t* pData, uint8_t len)
{
		  int i;
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
		rx = SPI5_ReadWriteByte(tx);
	  for(i = 0; i < len; i++)
	  {
      tx = regAddr | 0x80;		
			*pData = SPI5_ReadWriteByte(tx);	
			pData++;
			regAddr++;
	  }
    MPU_NSS_HIGH;
    return 0;
}


/**
	* @brief  ͨ��MPU6500��I2C����д��IST8310�Ĵ���
  * @param  addr: IST8310�Ĵ�����д���ַ
  *         data: Ҫд�������
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* ���ȹرմӻ�1 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    HAL_Delay(2);
    /* �򿪴ӻ�1������һ���ֽ� */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* �ȴ�����ʱ����ȷ���ӻ�1��������*/
    HAL_Delay(10);
}

/**
	* @brief  ͨ��MPU6500��I2C����д��IST8310�Ĵ���
	* @param  addr: Ҫ��ȡIST8310�Ĵ����ĵ�ַ
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    HAL_Delay(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
    return retval;
}

/**
	* @brief    ��ʼ��MPU6500 I2C�ӻ�0�Զ�ȡI2C��
* @param    device_address:�豸��ַ����ַ[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * ����IST8310���豸��ַ
     * ʹ�ôӻ�1���Զ����䵥����ģʽ
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    HAL_Delay(2);

    /* ʹ��slave0���Զ���ȡ���� */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    HAL_Delay(2);

    /* ÿ�˸�mpu6500�ڲ�����һ��i2c����ȡ */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    HAL_Delay(2);
    /* ���ôӻ�0��1�����ӳ�*/
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    HAL_Delay(2);
    /* ���ôӻ�1�Զ����� */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* �ȴ�6ms��16���ڲ�ƽ�����õ���С�ȴ�ʱ�䣩 */
    HAL_Delay(6); 
    /* ʹ�ö�ȡ�������ֽ������ôӻ�0 */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    HAL_Delay(2);
}

/**
	* @brief  ��ʼ��IST8310�豸
	* @param  
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t ist8310_init()
{
	  /* ����iic��ģʽ */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    HAL_Delay(10);
	  /* ����iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    HAL_Delay(10);
    /* �򿪴ӻ�1����д�룬�ӻ�4���ж�ȡ*/
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    HAL_Delay(10);
    /* IST8310 CONFB 0x01=�豸rst*/
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;
		/* ��λ*/
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    HAL_Delay(10);
		/* ����Ϊ����ģʽ�Է��ʼĴ��� */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    HAL_Delay(10);
		/* ����״̬����int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    HAL_Delay(10);	
    /* ���õ�����ģʽ��x��y��z��16ʱ��1ƽ��ֵ */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    HAL_Delay(10);
    /* ����/�����������ʱ�����ã�����ģʽ */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    HAL_Delay(10);
    /* �رմ���1�ʹ���4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
    /* ���ò����ô���0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    HAL_Delay(100);
    return 0;
}

/**
	* @brief  ��ȡIST8310������
  * @param  buff: ����IST8310���ݵĻ�����
	* @retval 
  * @usage  call in mpu_get_data() function
	*/
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}




/**
	* @brief  ����imu 6500�����ǲ�����Χ
  * @param  fsr: range(0,?50dps;1,?00dps;2,?000dps;3,?000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  ����imu 6050/6500���ٲ�����Χ
  * @param  fsr: range(0,?g;1,?g;2,?g;3,?6g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}




/**
	* @brief  ��ȡMPU6500��ƫ������
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i = 0;	
	int offset_tmp[6] = {0};
	uint8_t mpu_buff_offset[14] = {0};
		
	HAL_Delay(20);
	for (i=0; i<100;i++)
	{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff_offset, 14);
		
		offset_tmp[0] += (int16_t)(mpu_buff_offset[0] << 8 | mpu_buff_offset[1]);
		offset_tmp[1] += (int16_t)(mpu_buff_offset[2] << 8 | mpu_buff_offset[3]);
		offset_tmp[2] += (int16_t)(mpu_buff_offset[4] << 8 | mpu_buff_offset[5]);
		offset_tmp[3] += (int16_t)(mpu_buff_offset[8] << 8 | mpu_buff_offset[9]);
		offset_tmp[4] += (int16_t)(mpu_buff_offset[10] << 8 | mpu_buff_offset[11]);
		offset_tmp[5] += (int16_t)(mpu_buff_offset[12] << 8 | mpu_buff_offset[13]);
		
		HAL_Delay(5);
	}
	
	mpu_data.ax_offset = (int16_t)(offset_tmp[0] / 100);
	mpu_data.ay_offset = (int16_t)(offset_tmp[1] / 100);
	mpu_data.az_offset = (int16_t)(offset_tmp[2] / 100);
	mpu_data.gx_offset = (int16_t)(offset_tmp[3] / 100);
	mpu_data.gy_offset = (int16_t)(offset_tmp[4] / 100);
	mpu_data.gz_offset = (int16_t)(offset_tmp[5] / 100);
	
	IMU.norm_g = sqrt(mpu_data.ax_offset*mpu_data.ax_offset+mpu_data.ay_offset*mpu_data.ay_offset+mpu_data.az_offset*mpu_data.az_offset);
	IMU.Gravity = IMU.norm_g / 4096.0f;
	
	if(IMU.Gravity > 1.13f || IMU.Gravity < 0.87f)
	NVIC_SystemReset();
	
	HAL_Delay(20);

}

/**
	* @brief  ��ʼ��imu mpu6500�ʹ�ǿ��ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_device_init(void)
{
	HAL_Delay(100);
	float quaternion_Init[2] 				 = {0};
	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
																			{ MPU6500_SIGNAL_PATH_RESET, 0x07	},   /*�����ǡ����ٶȼơ��¶ȼƸ�λ*/
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */
	HAL_Delay(50);
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		HAL_Delay(10);
	}


//	ist8310_init();
//	HAL_Delay(10);	
	
//	while(Test_Temp());	
	
	mpu_offset_call();
	HAL_Delay(5);	

	quaternion_Init[0] = asin(mpu_data.ax_offset / IMU.norm_g);
	quaternion_Init[1] = asin(mpu_data.ay_offset / IMU.norm_g);
	quaternion_init(quaternion_Init[0], quaternion_Init[1]);
 
}

/**
	* @brief  ��ȡimu����
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data()
{
		uint8_t mpu_buff[14] = {0};
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];
    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);
		
//    ist8310_get_data(ist_buff);
//    memcpy(&mpu_data.mx, ist_buff, 6);
		
    IMU.temp = 21 + mpu_data.temp / 333.87f;
	  /* 2000dps -> rad/s */
		
		IMU.acc.x = mpu_data.ax / 4096.0f;
		if(IMU.acc.x>IMU.Gravity)  IMU.acc.x=IMU.Gravity;
		if(IMU.acc.x<-IMU.Gravity)  IMU.acc.x=-IMU.Gravity;		
		IMU.acc.y = mpu_data.ay / 4096.0f;
		if(IMU.acc.y>IMU.Gravity)  IMU.acc.y=IMU.Gravity;
		if(IMU.acc.y<-IMU.Gravity)  IMU.acc.y=-IMU.Gravity;				
		IMU.acc.z = mpu_data.az / 4096.0f;
		if(IMU.acc.z>IMU.Gravity)  IMU.acc.z=IMU.Gravity;
		if(IMU.acc.z<-IMU.Gravity)  IMU.acc.z=-IMU.Gravity;		
		
		IMU.acc.x = Filter_one(IMU.acc.x,0.1f);
		IMU.acc.y = Filter_one(IMU.acc.y,0.1f);
		IMU.acc.z = Filter_one(IMU.acc.z,0.1f);		
		
		IMU.gyro.x = mpu_data.gx / 16.384f;
//		if(IMU.gyro.x>GYRO_MAX)		IMU.gyro.x = GYRO_MAX;
		IMU.gyro.y = mpu_data.gy / 16.384f;		
//		if(IMU.gyro.y>GYRO_MAX)		IMU.gyro.y = GYRO_MAX;		
		IMU.gyro.z = mpu_data.gz / 16.384f;
//		if(IMU.gyro.z>GYRO_MAX)		IMU.gyro.z = GYRO_MAX;
		
		IMU.gyro.x = Filter_one(IMU.gyro.x,0.1f);
		IMU.gyro.y = Filter_one(IMU.gyro.y,0.1f);
		IMU.gyro.z = Filter_one(IMU.gyro.z,0.1f);		

		imu_attitude_update();

}

void imu_attitude_update(void)
{
	float dt = 0.001;
	IMU.eular_acc.pitch = asin(mpu_data.ay_offset/IMU.norm_g)*57.29578f;	
	
	IMU.Yaw_Ingetral = IMU.Yaw_Ingetral + IMU.gyro.z*dt;	
	IMU.Pit_Ingetral = IMU.Pit_Ingetral + IMU.gyro.x*dt;
	IMU.Rol_Ingetral = IMU.Rol_Ingetral + IMU.gyro.y*dt;

	imuUpdate(IMU.acc, IMU.gyro, 0.001);	
}

uint8_t Test_Temp(void)
{
	uint8_t mpu_buff[14];
	mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);	
	mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];
  IMU.temp = 21 + mpu_data.temp / 333.87f;
	
	if(IMU.temp >= 45.0f)  return 0;
	else  return 1;
}

uint16_t Imu_TempControl(float temp)
{	
	uint16_t Temp_pwm;
	if(IMU.temp > 45.f)   Temp_pwm = 0;
	else Temp_pwm = 20000;
	return Temp_pwm;
}

float Filter_one(float data,float Kp)  //һ���˲�
{
	static float last_data = 0;
	static uint8_t first_flag = 1;
	float res = 0;
	
	if(first_flag)
	{
		first_flag = 0;
		res = data;
	}
	else
	{
		res = (1.0f - Kp)*data + Kp*last_data;
	}
	last_data = res;
	
	return res;
}

