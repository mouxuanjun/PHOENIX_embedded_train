/*!***************************************************
 * @file: MYGYROData.c
 * @brief: 用于配置对陀螺仪加速度计进行数据单位换算
 * @note:	
 ****************************************************/
#define BMI088 BMI088
#include "stm32f4xx.h"
#define xx 0
#define yy 1
#define zz 2

#define G_Z 0

/*陀螺仪加速度计去零漂方式选择*/

#define MPU_ACC_CALOFFSET_AUTO
#define  MPU_GROY_CALOFFSET_AUTO
/*---------------------------------------------*/

#define XAISN   (3)  //3个坐标
#define ITEMS  6
#define MPU9250_FILTER_NUM   4
#define MPU_CORRECTION_FLASH     0x0800F000        //存储校正数据的FLASH地址，SIZE=6*3*4字节
#define GG 9.8															// 重力加速?
/* 现在量程设定为±6g */
#define MPU_ACCE_M_S_2 (24.0 * GG / 32768.0) //单位换算，将LSB化为m/(s^2)，GG前面的"X"需根据量程修改
#define MPU_GYRO_TO_DPS (2000 / 32768.0)    //单位换算，将LSB化为 gegree/s
#define MPU_MAGN_TO_GS (4800 / 16384.0 / 100.0)            //单位换算，将LSB化为Gs
#define MPU_MAGN_TO_UT (4800 / 16384.0)            //单位换算，将GS化为UT
#define MPU_TEMP_K (0.002995177763f)            //degC/LSB

typedef struct _accdata
{
    short origin[XAISN];  //原始值
    float offset[XAISN];      //零偏值
    float offset_max[XAISN];  //零偏值最大值
    float offset_min[XAISN];  //零偏值最小值
    float calibration[XAISN]; //校准值
    float filter[XAISN];      //滑动平均滤波值
    float m_s_2[XAISN];      //米每二次方秒
} accdata;
typedef struct _gyrodata
{
    short origin[XAISN];  //原始值
    float offset_max[XAISN];  //零偏值最大值
    float offset_min[XAISN];  //零偏值最小值
    float offset[XAISN];      //零偏值
    float calibration[XAISN]; //校准值
    float filter[XAISN];      //滑动平均滤波值
    float dps[XAISN];         //度每秒
    float radps[XAISN];       //弧度每秒
		/* 2022-01-11 加入滑动滤波 */
		float last_filter[XAISN];
} gyrodata;

struct _mpu
{
    accdata acc;
    gyrodata gyro;

    float Temperature;
    uint8_t acc_id, gyro_id;
    union
    {
        int16_t data[3];
        uint8_t buff[6];
    } ACC, GYRO;
    uint8_t temp_originalbuff[2];
    uint8_t gyro_times;
    uint8_t acc_times;
    enum
    {
        ReadingACC,
        ReadingGYRO,
        IDLE,
    } state;
		
    float pitch;
    float roll;
    float yaw;
    float lastyaw;
		/* 记录陀螺仪累计旋转多少度 */
		float yawsum;
		/* 陀螺仪动态数据矫正 */
		uint8_t DynamicOffsetEnable;	// 是否进行动态校准标志位
		int32_t DynamicTmp[3];				// 动态数据累加
		uint32_t DynamicTimes;				// 校准时间计数
		uint16_t OffsetCnt;						// 校准周期
		uint16_t OffsetErrorRange;		// 校准误差范围
		float gyro_z;
		int16_t yaw_turns;
};

typedef struct _mpu mpu;
extern mpu BMI088;

/**
  * @brief 	读取陀螺仪的数据        
  * @retval  将读取出来的数据赋值给原始数
  *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
  * @attention 从这开始实际的频率编程 2000hz
  */
void BMI088_Read_Gyro_Data(void);

/**
  * @brief 	读取加速度计的数据        
  * @retval  将读取出来的数据赋值给原始数
  *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
  * @attention 从这开始实际的频率编程 1600hz
  */
void BMI088_Read_Acc_Data(void);

/**
  * @brief 	读取温度     
  * @retval  将读取出来的数据赋值给原始数
  *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
  * @attention 从这开始实际的频率编程 1600hz
  */
void BMI088_Read_Tmp_Data(void);

/*!***************************************************
 * @file: MYGYROData.c
 * @brief:读取加速度，并转换为 m/(s^2)
 * @note:	
 ****************************************************/
void IMU_Read(void);
