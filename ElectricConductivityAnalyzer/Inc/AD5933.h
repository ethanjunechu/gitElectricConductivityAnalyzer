#ifndef _I2C_h_
#define _I2C_h_

//#include "sys.h"
#include "delay.h"

#include <math.h>

//IO方向设置
#define SDA_IN();  {GPIO_InitTypeDef GPIO_InitStruct = { 0 };GPIO_InitStruct.Pin = GPIO_PIN_11;GPIO_InitStruct.Mode = GPIO_MODE_INPUT;GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);}
#define SDA_OUT(); {GPIO_InitTypeDef GPIO_InitStruct = { 0 };GPIO_InitStruct.Pin = GPIO_PIN_11;GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);}

//IO操作函数
#define SCL(x) (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET))
#define SDA(x) (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET))

/* 比较电阻选择IO */
#define S0(x) (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET))
#define S1(x) (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET))
#define S2(x) (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET))
#define S3(x) (x == 1 ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET))
#define S4(x) (x == 1 ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET))
#define S5(x) (x == 1 ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET))

#define READ_SDA HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) //??SDA

#define AD5933_SYS_Init (1) << 12
#define AD5933_Begin_Fre_Scan (2) << 12
#define AD5933_Fre_UP (3) << 12
#define AD5933_Fre_Rep (4) << 12
#define AD5933_Get_Temp (9) << 12
#define AD5933_Sleep (10) << 12
#define AD5933_Standby (11) << 12

#define AD5933_OUTPUT_2V (0) << 9
#define AD5933_OUTPUT_1V (3) << 9
#define AD5933_OUTPUT_400mV (2) << 9
#define AD5933_OUTPUT_200mV (1) << 9

#define AD5933_Gain_1 (1) << 8
#define AD5933_Gain_5 (0) << 8

#define AD5933_IN_MCLK (0) << 3
#define AD5933_OUT_MCLK (1) << 3

#define AD5933_Reset (1) << 4

//定义函数
void Ini_I2c(void);
void RangeSelect(uint8_t index);
void SDA_1(void);
void SDA_0(void);
void SCL_1(void);
void SCL_0(void);
void GetACK(void);
void SendNACK(void);
void START(void);
void STOP(void);
void SendByte(uint8_t txd); // 发送一个字节数据子函数
uint8_t ReadByte(void);     //读一个字节数据
void Write_Byte(char nAddr, unsigned int nValue);
void SetPointer(char nAddr);
int Rece_Byte(char nAddr);
void Delay_ms(unsigned long nValue);
//float Scale_imp(uint8_t *SValue, uint8_t *IValue, uint8_t *NValue,
//		uint8_t *CValue);
void Scale_imp(void);
float check_AD5933(void);
uint16_t AD5933_Tempter(void);
float Get_resistance(uint16_t num);
float AD5933_Sweep(float Fre_Begin, float Fre_UP, uint16_t UP_Num,
		uint16_t OUTPUT_Vatage, uint16_t Gain, uint16_t SWeep_Rep);
float DA5933_Get_Rs(void);
float DA5933_Get_Cap(float Fre);
float DA5933_Get_L(float Fre);
#endif
