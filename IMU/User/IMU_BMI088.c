/*!***************************************************
 * @brief: 用于配置对陀螺仪加速度计进行单位换算
 * @note:	
 ****************************************************/

#include "IMU_BMI088.h"
#include "BMI088.h"
//陀螺仪零偏矫正
mpu BMI088;

/**
  * @brief 	读取陀螺仪的数据        
  * @retval  将读取出来的数据赋值给原始数
	*	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
	* @attention 从这开始实际的频率编程 2000hz
  */
void BMI088_Read_Gyro_Data(void)
{
    BMI088.gyro.origin [xx] = BMI088.GYRO.data[xx];
    BMI088.gyro.origin [yy] = BMI088.GYRO.data[yy];
    BMI088.gyro.origin [zz] = BMI088.GYRO.data[zz];
}

/**
  * @brief 	读取加速度计的数据        
  * @retval  将读取出来的数据赋值给原始数
	*	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
	* @attention 从这开始实际的频率编程 1600hz
  */
void BMI088_Read_Acc_Data(void)
{
    BMI088.acc.origin [xx] = BMI088.ACC.data[xx];
    BMI088.acc.origin [yy] = BMI088.ACC.data[yy];
    BMI088.acc.origin [zz] = BMI088.ACC.data[zz];
}

/**
  * @brief 	读取温度     
  * @retval  将读取出来的数据赋值给原始数
	*	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
	* @attention 从这开始实际的频率编程 1600hz
  */
void BMI088_Read_Tmp_Data(void)
{
	BMI088_Read_TMP(&BMI088.Temperature);
}

/*!***************************************************
 * @file: MYGYROData.c
 * @brief:读取加速度，并转换为 m/(s^2)
 * @note:	
 ****************************************************/
void IMU_Read(void)
{
	mpu *mympu=&BMI088;
	BMI088_Read_Gyro_Data();	/* 读取陀螺仪数据 */
	BMI088_Read_Acc_Data();		/* 读取加速度数据 */
	
	mympu->gyro.dps[xx]  = mympu->gyro.origin[xx] * MPU_GYRO_TO_DPS;
	mympu->gyro.dps[yy]  = mympu->gyro.origin[yy] * MPU_GYRO_TO_DPS;
	mympu->gyro.dps[zz]  = mympu->gyro.origin[zz] * MPU_GYRO_TO_DPS;
	
	mympu->acc.m_s_2 [xx] = mympu->acc.origin[xx] * MPU_ACCE_M_S_2;
	mympu->acc.m_s_2 [yy] = mympu->acc.origin[yy] * MPU_ACCE_M_S_2;
	mympu->acc.m_s_2 [zz] = mympu->acc.origin[zz] * MPU_ACCE_M_S_2;
}
