/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置BMI088陀螺仪 加速度计的各项参数并读取数据
 * @author:        
 * @date:2022/05/31
 * @note:	
 ****************************************************/
#include "BMI088.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "IMU_BMI088.h"
#include "spi.h"
#define SPI1_ACC_Enable       {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);}//PA4 PB0
#define SPI1_ACC_Disable       {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);}//PA4 PB0
#define SPI1_GYRO_Enable     {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);}//PA4 PB0
#define SPI1_GYRO_Disable     {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);}//PA4 PB0
//PA4
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_ACCEL_Pin 4


#define CS_GYRO  0
#define CS_ACC   1


#define BOARD_IMU 0
#define FLOAT_IMU 1

/* 用于读取BMI088温度数据 */
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
mtime_t mtime;
/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于BMI088传感器中的软件延时
 * @author:        
 * @date:2022/05/31
 * @note:	
 ****************************************************/
//加速度计可以使用软件重置寄存器对所有寄存器的值进行恢复，对地址为 0x7E 的
//ACC_SOFTRESET 寄存器写入0xB6，就会将加速度计内所有寄存器恢复为默认值（一般
//为0）。在初始化时，会通过软件重置寄存器进行重置。
static void BMI088_Delay(int times)
{
	
 osDelay(times);
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 通过SPI通讯总线读取和发送传感器数据
 * @author:        
 * @date:2022/05/31
 * @note:	
 ****************************************************/
static uint8_t BMI088_SPIReadSend(uint8_t data)
{
    uint8_t ret = 0xff;
    HAL_SPI_TransmitReceive(&hspi1, &data, &ret, 1, 0xffff);
    return ret;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪的数据
 * @author:        
 * @date:2022/05/31
 * @note:	已经知道读取陀螺仪数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_GYRO(uint8_t Addr, int BOARD_OR_FLOAT)
{
    uint8_t val;
    SPI1_GYRO_Enable
    BMI088_SPIReadSend(Addr | 0x80);
    val = (uint8_t)(BMI088_SPIReadSend(0x00) & 0xFF);
    SPI1_GYRO_Disable;
    return val;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取加速度计的数据
 * @author:        
 * @date:2022/05/31
 * @note:	已经知道读取加速度计数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_ACC(uint8_t Addr, int BOARD_OR_FLOAT)
{
    uint8_t val;
    SPI1_ACC_Enable
    BMI088_SPIReadSend(Addr | 0x80);
    BMI088_SPIReadSend(0x00) ;
    val = (uint8_t)(BMI088_SPIReadSend(0x00) & 0xFF);
    SPI1_ACC_Disable
    return val;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:        
 * @date:2022/05/31
 * @note:	
 ****************************************************/
static void BMI088_Write_Reg(uint8_t Addr, uint8_t Val, uint8_t ACC_OR_GYRO)
{
    if (ACC_OR_GYRO == CS_ACC)
    {
        SPI1_ACC_Enable
    }
    else if (ACC_OR_GYRO == CS_GYRO)
    {
        SPI1_GYRO_Enable
    }
    BMI088_SPIReadSend(Addr & 0x7f);
    BMI088_SPIReadSend(Val);
    if (ACC_OR_GYRO == CS_ACC)
    {
        SPI1_ACC_Disable
    }
    else if (ACC_OR_GYRO == CS_GYRO)
    {
        SPI1_GYRO_Disable
    }
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的加速度计
 * @author:        
 * @date:2022/05/31
 * @note:	配置加速度计产生中断的IO口的输出方式，数据输出频率1600hz
				加速度量程是 ±24g 有点大，目前的车无需这么大的量程
 ****************************************************/
/**
  * @brief  
  * @retval  尝试降低采样率是否能提高数据稳定性
  * @date: 2021/10/17
  */
int ret = 0;
static uint8_t BMI088_ACC_Congfig(int BOARD_OR_FLOAT)
{
    SPI1_ACC_Disable;
    BMI088_Write_Reg(0x7e, 0xb6, CS_ACC);//Soft Reset
    while (BMI088.acc_id != ACC_CHIP_ID) //Rising edge ,turn to spi
    {

        BMI088_Delay(100);
        BMI088.acc_id = BMI088_Read_ACC(0x00, BOARD_OR_FLOAT); //id:1E
    }
    BMI088_Delay(50);//> 1 ms ;
    ret = 0;
    while (ret != ACC_ON)
    {
        ret = BMI088_Read_ACC(ACC_PWR_CTRL, BOARD_OR_FLOAT);
        BMI088_Write_Reg(ACC_PWR_CTRL, ACC_ON, CS_ACC);
        BMI088_Delay(5);
    }
	/* 可根据实际使用和加速度寄存器表修改加速度计测量范围 */
    while (BMI088_Read_ACC(ACC_RANG, BOARD_OR_FLOAT) != Plus_Minus_24G)
    {
        BMI088_Write_Reg(ACC_RANG, Plus_Minus_24G, CS_ACC); //ACC Rang +- 24g;//
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(0x40, BOARD_OR_FLOAT) != 0xBC)
    {
		/* 可根据实际使用和加速度寄存器表修改加速度计数据读取频率 */
        BMI088_Write_Reg(0x40, 0xBC, CS_ACC); //
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(0X53, BOARD_OR_FLOAT) != 0X08)
    {
        BMI088_Write_Reg(0X53, 0X08, CS_ACC); //0000 1000 INT1 OUTPUT PUSH-PULL  Active low
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(0X54, BOARD_OR_FLOAT) != 0X08)
    {
        BMI088_Write_Reg(0X54, 0X08, CS_ACC); //0000 1000 INT2 OUTPUT PUSH-PULL  Active low
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(0X58, BOARD_OR_FLOAT) != 0x44)
    {
        BMI088_Write_Reg(0X58, 0x44, CS_ACC); //0100 0100 data ready interrupt to INT1 and INT2
        BMI088_Delay(5);
    }

    BMI088_Delay(20);//>1ms
    while (BMI088_Read_ACC(ACC_PWR_CONF, BOARD_OR_FLOAT) != ACC_ACTIVE)
    {
        BMI088_Write_Reg(ACC_PWR_CONF, ACC_ACTIVE, CS_ACC); //ACtive mode
        BMI088_Delay(600);//>50ms
    }
    return 0;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪
 * @author:        
 * @date:2022/05/31
 * @note:	配置陀螺仪产生中断的IO口的输出方式，数据输出频率2000hz
 ****************************************************/
static uint8_t BMI088_GYRO_Congfig(int BOARD_OR_FLOAT)
{
    SPI1_ACC_Disable;

    while (BMI088.gyro_id != GYRO_CHIP_ID)
    {
        BMI088.gyro_id = BMI088_Read_GYRO(0x00, BOARD_OR_FLOAT); //0x0f
        BMI088_Delay(5);
    }
    while (BMI088_Read_GYRO(GYRO_RANG, BOARD_OR_FLOAT) != Plus_Minus_2000)
    {
        BMI088_Write_Reg(GYRO_RANG, Plus_Minus_2000, CS_GYRO);// rang +-2000
        BMI088_Delay(5);
    }
    //bit #7 is Read Only
		
	/* 可根据陀螺仪寄存器对照表陀螺仪采样率是2000hz */
    BMI088_Write_Reg(GYRO_BANDWIDTH, ODR_1000_FD_116, CS_GYRO);

    while (BMI088_Read_GYRO(0X11, BOARD_OR_FLOAT) != 0x00)
    {
        BMI088_Write_Reg(0X11, 0x00, CS_GYRO);//normal
        BMI088_Delay(5);
    }
    while (BMI088_Read_GYRO(0X15, BOARD_OR_FLOAT) != 0X80)
    {
        BMI088_Write_Reg(0X15, 0X80, CS_GYRO);// //interrupt
        BMI088_Delay(5);
    }
    while (BMI088_Read_GYRO(0X16, BOARD_OR_FLOAT) != 0X00)
    {
        BMI088_Write_Reg(0X16, 0X00, CS_GYRO);//OUTPUT PUSH-PULL  Active low
        BMI088_Delay(5);
    }
    while (BMI088_Read_GYRO(0X18, BOARD_OR_FLOAT) != 0X81)
    {
        BMI088_Write_Reg(0X18, 0X81, CS_GYRO);//data ready interrupt to INT1 and INT2
        BMI088_Delay(5);
    }
    return 0;
}

/*****************************以下几个函数用于读取BMI088传感器的温度数据****************************/
/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器高八位数据
 * @author:        
 * @date:2022/05/31
 * @note:	参考的大疆源码BMI088传感器的温度补偿控制
			温度传感器应该是挂载在加速度传感器上，所以读取温度
			需要使用加速度传感器
 ****************************************************/
static void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器低八位数据
 * @author:        
 * @date:2022/05/31
 * @note:	参考的大疆源码BMI088传感器的温度补偿控制
			温度传感器应该是挂载在加速度传感器上，所以读取温度
			需要使用加速度传感器
 ****************************************************/
static void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:        
 * @date:2022/05/31
 * @note:	该函数与 BMI088_Write_Reg 函数不同，使用的时候需要注意
 ****************************************************/
static uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取BMI088传感器中多个寄存器的数据
 * @author:        
 * @date:2022/05/31
 * @note:	参考的大疆源码BMI088传感器的温度补偿控制
 ****************************************************/
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);
    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
/***************************************************************************************************/

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪和加速度计
 * @author:        
 * @date:2022/05/31
 * @note:	
 ****************************************************/
uint8_t BMI088_FLOAT_ACC_GYRO_Init(void)
{
    BMI088_ACC_Congfig(FLOAT_IMU);
    BMI088_GYRO_Congfig(FLOAT_IMU);
    return 1;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取传感器的温度
 * @author:        
 * @date: 2021/10/18
 * @note:	读取出来温度进行温度补偿
 ****************************************************/
void BMI088_Read_TMP(float *temperate)
{
	int16_t bmi088_raw_temp;
	bmi088_raw_temp = (int16_t)((BMI088.temp_originalbuff[0] << 3) | (BMI088.temp_originalbuff[1] >> 5));
	if (bmi088_raw_temp > 1023)
	{
			bmi088_raw_temp -= 2048;
	}
	*temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* 按键外部中断定义变量 */
extern uint8_t exit_flag;
extern uint8_t rising_falling_flag;

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪
 * @author:        
 * @date:2022/05/31
 * @note:	I/O外部中断回调函数，用于接收陀螺仪和加速度数据
 ****************************************************/
/* 定义变量用于读取陀螺仪数据具体传输频率 */
int16_t Gyro_Cnt = 0;
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if( mtime.Init_OK!=1)
        return;

    switch (GPIO_Pin)
    {
    case GPIO_PIN_4://ACC
        BMI088.ACC.buff[0] = BMI088_Read_ACC(0X12, FLOAT_IMU);
        BMI088.ACC.buff[1] = BMI088_Read_ACC(0X13, FLOAT_IMU);
        BMI088.ACC.buff[2] = BMI088_Read_ACC(0X14, FLOAT_IMU);
        BMI088.ACC.buff[3] = BMI088_Read_ACC(0X15, FLOAT_IMU);
        BMI088.ACC.buff[4] = BMI088_Read_ACC(0X16, FLOAT_IMU);
        BMI088.ACC.buff[5] = BMI088_Read_ACC(0X17, FLOAT_IMU);
				/* 用于读取传感器的温度 */
				BMI088_accel_read_muli_reg(BMI088_TEMP_M, BMI088.temp_originalbuff, 2);
        break;
    case GPIO_PIN_5://GYRO
        BMI088.GYRO.buff[0] = BMI088_Read_GYRO(0X02, FLOAT_IMU);
        BMI088.GYRO.buff[1] = BMI088_Read_GYRO(0X03, FLOAT_IMU);
        BMI088.GYRO.buff[2] = BMI088_Read_GYRO(0X04, FLOAT_IMU);
        BMI088.GYRO.buff[3] = BMI088_Read_GYRO(0X05, FLOAT_IMU);
        BMI088.GYRO.buff[4] = BMI088_Read_GYRO(0X06, FLOAT_IMU);
        BMI088.GYRO.buff[5] = BMI088_Read_GYRO(0X07, FLOAT_IMU);
		// 陀螺仪的引脚触发中断一次，计数+1，用于确定陀螺仪传感器实际的传输频率
        break;
    default:
        break;
    }
}
