/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置BMI088陀螺仪 加速度计的各项参数并读取数据
 * @author:   
 * @date: 2022/05/31
 * @note:	
 ****************************************************/
#ifndef __BMI088_H__
#define __BMI088_H__
#include "stm32f4xx.h"
#define ACC_CHIP_ID 0X1E

#define	ACC_PWR_CTRL  0X7D
#define	ACC_OFF  0X00
#define	ACC_ON  0X04

#define	ACC_PWR_CONF  0X7C
#define	ACC_ACTIVE  0X00
#define	ACC_SUSPEND  0X03

/* 加速度量程范围设定 */
#define	ACC_RANG  0X41
#define	Plus_Minus_3G  0X00
#define	Plus_Minus_6G  0X01
#define	Plus_Minus_12G  0X02
#define	Plus_Minus_24G  0X03

/* 加速度信号输出频率设定 */
#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)


#define GYRO_CHIP_ID 0X0F

#define	GYRO_RANG  0X0F
#define	Plus_Minus_2000  0X00
#define	Plus_Minus_1000  0X01
#define	Plus_Minus_500   0X02
#define	Plus_Minus_250   0X03
#define	Plus_Minus_125   0X04


#define	GYRO_BANDWIDTH  0X10
/* 2000代表采样频率，532代表数据宽度 */
#define	ODR_2000_FD_532  0X00
#define	ODR_2000_FD_230  0X01
#define	ODR_1000_FD_116  0X02
#define	ODR_400_FD_47  0X03
#define	ODR_200_FD_23  0X04
#define	ODR_100_FD_12 0X05
#define	ODR_200_FD_64 0X06
#define	ODR_100_FD_32 0X07

/* 温度读取 */
#define BMI088_TEMP_M 0x22

#define BMI088_TEMP_L 0x23
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f


typedef struct {
    uint8_t Init_OK;  // 初始化状态标志
} mtime_t;

extern mtime_t mtime;  // 全局变量声明
/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于BMI088传感器中的软件延时
 * @author:   
 * @date: 2022/05/31
 * @note:	
 ****************************************************/
static void BMI088_Delay(int times);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 通过SPI通讯总线读取和发送传感器数据
 * @author:   
 * @date: 2022/05/31
 * @note:	
 ****************************************************/
static uint8_t BMI088_SPIReadSend(uint8_t data);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取加速度计的数据
 * @author:   
 * @date: 2022/05/31
 * @note:	已经知道读取加速度计数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_ACC(uint8_t Addr, int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪的数据
 * @author:   
 * @date: 2022/05/31
 * @note:	已经知道读取陀螺仪数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_GYRO(uint8_t Addr, int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:   
 * @date: 2022/05/31
 * @note:	
 ****************************************************/
static void BMI088_Write_Reg(uint8_t Addr, uint8_t Val, uint8_t ACC_OR_GYRO);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的加速度计
 * @author:   
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_ACC_Congfig(int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪
 * @author:   
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_GYRO_Congfig(int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器高八位数据
 * @author:   
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_ACCEL_NS_H(void);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器低八位数据
 * @author:   
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_ACCEL_NS_L(void);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:   
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_read_write_byte(uint8_t txdata);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取BMI088传感器中多个寄存器的数据
 * @author:   
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪和加速度计
 * @author:   
 * @date: 2022/05/31
 * @note:	
 ****************************************************/
uint8_t BMI088_FLOAT_ACC_GYRO_Init(void);


/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取传感器的温度
 * @author:   
 * @date: 2021/10/18
 * @note:	读取出来温度进行温度补偿
 ****************************************************/
void BMI088_Read_TMP(float *temperate);
#endif
