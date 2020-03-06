#ifndef __AD5933_H__
#define __AD5933_H__
#ifdef __cplusplus
 extern "C" {
#endif

#define Delay_IIC 1
#define AD5933 1
#define AD5933_MCLK 16.776  //=536870912/MCLK;
//#define	AD5933_MCLK_USE_OUT	1	//0内部时钟  1外部时钟
#define AD5933_Correction 101615461.47044108
//IO方向设置
#define SDA_IN()  GPIO_InitStruct.Pin = GPIO_PIN_11;GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB, &GPIO_InitStruct)
#define SDA_OUT() GPIO_InitStruct.Pin = GPIO_PIN_11;GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB, &GPIO_InitStruct)

//IO操作函数
#define SCL(x)	(x==1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET))
#define SDA(x)	(x==1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET))
#define READ_SDA()   HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)  //输入SDA

//#define AD5933_SYS_Init				(1)<<12
//#define AD5933_Begin_Fre_Scan		(2)<<12
//#define AD5933_Fre_UP				(3)<<12
//#define AD5933_Fre_Rep				(4)<<12
//#define AD5933_Get_Temp				(9)<<12
//#define AD5933_Sleep				(10)<<12
//#define AD5933_Standby				(11)<<12
//
//#define AD5933_OUTPUT_2V			(0)<<9
//#define AD5933_OUTPUT_1V			(3)<<9
//#define AD5933_OUTPUT_400mV			(2)<<9
//#define AD5933_OUTPUT_200mV			(1)<<9
//
//#define AD5933_Gain_1				(1)<<8
//#define AD5933_Gain_5				(0)<<8
//
//#define AD5933_IN_MCLK				(0)<<3
//#define AD5933_OUT_MCLK				(1)<<3
//
//#define AD5933_Reset				(1)<<4
#define AD5933_SYS_Init				1
#define AD5933_Begin_Fre_Scan		2
#define AD5933_Fre_UP				3
#define AD5933_Fre_Rep				4
#define AD5933_Get_Temp				9
#define AD5933_Sleep				10
#define AD5933_Standby				11

#define AD5933_OUTPUT_2V			0
#define AD5933_OUTPUT_1V			3
#define AD5933_OUTPUT_400mV			2
#define AD5933_OUTPUT_200mV			1

#define AD5933_Gain_1				1
#define AD5933_Gain_5				0

#define AD5933_IN_MCLK				0
#define AD5933_OUT_MCLK				1

#define AD5933_Reset				1
//定义函数
void Ini_I2c(void);
void SDA_1(void);
void SDA_0(void);
void SCL_1(void);
void SCL_0(void);
void GetACK(void);
void SendNACK(void);
void START(void);
void STOP(void);
void SendByte(uint8_t txd);	// 发送一个字节数据子函数
uint8_t ReadByte(void);  //读一个字节数据
void Write_Byte(char nAddr, uint32_t nValue);
void SetPointer(char nAddr);
int Rece_Byte(char nAddr);
float Scale_imp(uint8_t *SValue, uint8_t *IValue, uint8_t *NValue,
		uint8_t *CValue);
uint16_t AD5933_Tempter(void);
float Get_resistance(uint16_t num);
float AD5933_Sweep(float Fre_Begin, float Fre_UP, uint16_t UP_Num,
		uint16_t OUTPUT_Vatage, uint16_t Gain, uint16_t SWeep_Rep);
float DA5933_Get_Rs(void);
float DA5933_Get_Cap(float Fre);
float DA5933_Get_L(float Fre);
void delay_us(uint16_t us);
float Rs;
#endif /*__AD5933_H */
/******************* (C) COPYRIGHT 2015-2020 Ethan *****END OF FILE****/
