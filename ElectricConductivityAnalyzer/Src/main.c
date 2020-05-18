/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2020 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dac.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_spi_flash.h"
#include "bsp_rtc.c"
#include "AD5933.h"
#include "string.h"
#include <stdlib.h>
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* 调试模式 */
//#define DEBUG 1
/* 串口3发送等待时间 */
#define USARTSENDTIME 0xFFFF
/* 目前可调范围 +-99 */
#define ADMAX 4008
#define ADMIN 770
#define ADMAXCAL 99
#define ADMINCAL 99
/* 电极距离 | 横截面积 /cm ｜ cm2 */
#define CELLLENGTH 1
#define CELLAREA 1
DAC_HandleTypeDef hdac;
/* GPIO宏定义 */
#define LED_LO(x)   (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET))
#define LED_HI(x) (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET))
#define LED_LS(x) (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET))
#define LED_WASH(x) (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET))
#define RELAY_WASH(x) (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET))
#define RELAY_LO(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET))
#define RELAY_HI(x) (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET))
#define BTN_CONFIG() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)
#define BTN_CAL() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)
#define BTN_MODE() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define BTN_RIGHT() HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)
#define BTN_ENTER() HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)
#define RS485_EN(x) (x == 1 ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET))

//校正次数宏定义——1次1s
#define CAL_COUNT 10

//不同温度下纯水的饱和蒸汽压力(0度-39度)
static float waterPerssure[] = { 0.6112, 0.6571, 0.7060, 0.7582, 0.8135, 0.8726,
		0.9354, 1.0021, 1.0730, 1.1482, 1.2282, 1.3130, 1.4028, 1.4980, 1.5989,
		1.7057, 1.8188, 1.9383, 2.0644, 2.1982, 2.3392, 2.4881, 2.6452, 2.8109,
		2.9856, 3.1698, 3.3637, 3.5679, 3.7828, 4.0089, 4.2467, 4.4966, 4.7563,
		5.0351, 5.3247, 5.6286, 5.9475, 6.2819, 6.6324, 6.9997 };
//氧在不同温度的水中饱和浓度表(0度-39度)
static float waterDO[] = { 14.62, 14.22, 13.83, 13.46, 13.11, 12.77, 12.45,
		12.14, 11.84, 11.56, 11.29, 11.03, 10.78, 10.54, 10.31, 10.08, 9.87,
		9.66, 9.47, 9.28, 9.09, 8.91, 8.74, 8.58, 8.42, 8.26, 8.11, 7.97, 7.83,
		7.69, 7.56, 7.43, 7.30, 7.18, 7.07, 6.95, 6.84, 6.73, 6.63, 6.53 };
// CRC 高位字节值表
static unsigned char auchCRCHi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };
// CRC 低位字节值表
static char auchCRCLo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
		0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
		0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F,
		0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
		0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1,
		0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB,
		0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
		0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5,
		0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
		0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
		0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79,
		0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
		0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73,
		0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D,
		0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F,
		0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
		0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

/* 历史记录 */
float History_PPM[400];
RTC_TimeTypeDef History_TIME[400];
RTC_DateTypeDef History_DATE[400];
static long historyCNT = 0;
static long historyStart = 0;
static long historyEnd = 0;

//设置背景白色
uint8_t SetBackWhiteCMD[8] = { 0xEE, 0x42, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF };

//设置前景红色
uint8_t SetForeRedCMD[8] = { 0xEE, 0x41, 0xF8, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//设置前景黑色
uint8_t SetForeBlackCMD[8] = { 0xEE, 0x41, 0x00, 0x01, 0xFF, 0xFC, 0xFF, 0xFF };

//设置前景灰色
uint8_t SetForeGrayCMD[8] = { 0xEE, 0x41, 0xC6, 0x18, 0xFF, 0xFC, 0xFF, 0xFF };

//设置前景蓝色
uint8_t SetForeBlueCMD[8] = { 0xEE, 0x41, 0x00, 0x1F, 0xFF, 0xFC, 0xFF, 0xFF };

//根据已设置背景色清屏
uint8_t ClearBackBlackCMD[6] = { 0xEE, 0x01, 0xFF, 0xFC, 0xFF, 0xFF };

//显示主数字第1位												/ 数字位图片编号 /
uint8_t ShowMainNum1CMD[13] = { 0xEE, 0x32, 0x00, 86, 0x00, 58, 0x00, 1, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示主数字第2位
uint8_t ShowMainNum2CMD[13] = { 0xEE, 0x32, 0x00, 150, 0x00, 58, 0x00, 1, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示主数字第3位
uint8_t ShowMainNum3CMD[13] = { 0xEE, 0x32, 0x00, 215, 0x00, 58, 0x00, 21, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示主数字第4位
uint8_t ShowMainNum4CMD[13] = { 0xEE, 0x32, 0x00, 238, 0x00, 58, 0x00, 1, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示主数字第5位
uint8_t ShowMainNum5CMD[13] = { 0xEE, 0x32, 0x01, 46, 0x00, 58, 0x00, 1, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };

//显示温度数字第1位
uint8_t ShowTempNum1CMD[13] = { 0xEE, 0x32, 0x01, 30, 0x00, 164, 0x00, 0x46,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度数字第2位
uint8_t ShowTempNum2CMD[13] = { 0xEE, 0x32, 0x01, 58, 0x00, 164, 0x00, 0x46,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度数字第3位
uint8_t ShowTempNum3CMD[13] = { 0xEE, 0x32, 0x01, 85, 0x00, 164, 0x00, 0x45,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度数字第4位
uint8_t ShowTempNum4CMD[13] = { 0xEE, 0x32, 0x01, 96, 0x00, 164, 0x00, 0x46,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示氧浓度单位
uint8_t ShowMainUnitCMD[13] = { 0xEE, 0x32, 0x01, 130, 0x00, 62, 0x00, 0x1B,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度单位
uint8_t ShowTempUnitCMD[13] = { 0xEE, 0x32, 0x01, 126, 0x00, 168, 0x00, 0x51,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示界面状态
uint8_t ShowPageStatusCMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x29, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示报警状态
uint8_t ShowAlarmStatusCMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x2B, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示清洗状态
uint8_t ShowWashStatusCMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x44, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示电极状态文字
uint8_t ShowElectrodeTextCMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		45, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示电极状态
uint8_t ShowElectrodeStatusCMD[13] = { 0xEE, 0x32, 0x00, 0x0A, 0x00, 240, 0x00,
		46, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示输出电流文字
uint8_t ShowmATextCMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 82,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示输出电流数字第1位
uint8_t ShowmANum1CMD[13] = { 0xEE, 0x32, 0x01, 100, 0x00, 242, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示输出电流数字第2位
uint8_t ShowmANum2CMD[13] = { 0xEE, 0x32, 0x01, 116, 0x00, 242, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示输出电流数字第3位-小数点
uint8_t ShowmANum3CMD[13] = { 0xEE, 0x32, 0x01, 134, 0x00, 242, 0x00, 30, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示输出电流数字第4位
uint8_t ShowmANum4CMD[13] = { 0xEE, 0x32, 0x01, 142, 0x00, 242, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示输出电流单位
uint8_t ShowmAUnitCMD[13] = { 0xEE, 0x32, 0x01, 161, 0x00, 241, 0x00, 25, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };

//显示日期数字第1位
uint8_t ShowDateNum1CMD[13] = { 0xEE, 0x32, 0x00, 200, 0x00, 0, 0x00, 33, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第2位
uint8_t ShowDateNum2CMD[13] = { 0xEE, 0x32, 0x00, 216, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第3位
uint8_t ShowDateNum3CMD[13] = { 0xEE, 0x32, 0x00, 232, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第4位
uint8_t ShowDateNum4CMD[13] = { 0xEE, 0x32, 0x00, 248, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第5位
uint8_t ShowDateNum5CMD[13] = { 0xEE, 0x32, 0x01, 8, 0x00, 0, 0x00, 89, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第6位
uint8_t ShowDateNum6CMD[13] = { 0xEE, 0x32, 0x01, 24, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第7位
uint8_t ShowDateNum7CMD[13] = { 0xEE, 0x32, 0x01, 40, 0x00, 0, 0x00, 32, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第8位
uint8_t ShowDateNum8CMD[13] = { 0xEE, 0x32, 0x01, 56, 0x00, 0, 0x00, 89, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第9位
uint8_t ShowDateNum9CMD[13] = { 0xEE, 0x32, 0x01, 72, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示日期数字第10位
uint8_t ShowDateNum10CMD[13] = { 0xEE, 0x32, 0x01, 88, 0x00, 0, 0x00, 32, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示时间数字第1位
uint8_t ShowTimeNum1CMD[13] = { 0xEE, 0x32, 0x01, 126, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示时间数字第2位
uint8_t ShowTimeNum2CMD[13] = { 0xEE, 0x32, 0x01, 142, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示时间数字第3位
uint8_t ShowTimeNum3CMD[13] = { 0xEE, 0x32, 0x01, 161, 0x00, 0, 0x00, 90, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示时间数字第4位
uint8_t ShowTimeNum4CMD[13] = { 0xEE, 0x32, 0x01, 170, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示时间数字第5位
uint8_t ShowTimeNum5CMD[13] = { 0xEE, 0x32, 0x01, 186, 0x00, 0, 0x00, 31, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };

//显示设置界面选择条
uint8_t ShowConfigSelect1CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 54, 0x00, 65,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigSelect2CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 100, 0x00, 65,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigSelect3CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 150, 0x00, 65,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigSelect4CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 200, 0x00, 65,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigSelect5CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 248, 0x00, 65,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigUnselect1CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 54, 0x00, 92,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigUnselect2CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 100, 0x00,
		92, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigUnselect3CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 150, 0x00,
		92, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
uint8_t ShowConfigUnselect4CMD[13] = { 0xEE, 0x32, 0x00, 30, 0x00, 200, 0x00,
		92, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面1
uint8_t ShowConfPage1CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 53,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面1.2
uint8_t ShowConfPage1_2CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		107, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面2
uint8_t ShowConfigPage2CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 54,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面3
uint8_t ShowConfigPage3CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 55,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面4
uint8_t ShowConfigPage4CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 56,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面5
uint8_t ShowConfigPage5CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 57,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面6
uint8_t ShowConfigPage6CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		117, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面7
uint8_t ShowConfigPage7CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 93,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示设置界面8
uint8_t ShowConfigPage8CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 94,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示密码页面
uint8_t ShowPassWPageCMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 113,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示模式设置
uint8_t ShowConfModeCMD[13] = { 0xEE, 0x32, 0x01, 32, 0x00, 62, 0x00, 109, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
////显示盐度设置数字1
//uint8_t ShowConfSalinityNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 114, 0x00,
//		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
////显示盐度设置数字2
//uint8_t ShowConfSalinityNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 114, 0x00,
//		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
////显示盐度设置数字3-小数点
//uint8_t ShowConfSalinityNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00,
//		30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
////显示盐度设置数字4
//uint8_t ShowConfSalinityNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00,
//		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
////显示盐度设置单位
//uint8_t ShowConfSalinityUnitCMD[13] = { 0xEE, 0x32, 0x01, 104, 0x00, 109, 0x00,
//		84, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度设置数字1
uint8_t ShowConfTempNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 114, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度设置数字2
uint8_t ShowConfTempNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 114, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度设置数字3-小数点
uint8_t ShowConfTempNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00, 30,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度设置数字4
uint8_t ShowConfTempNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示温度设置单位
uint8_t ShowConfTempUnitCMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 112, 0x00, 81,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示当前温度数字1
uint8_t ShowConfTempNowNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示当前温度数字2
uint8_t ShowConfTempNowNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示当前温度数字3-小数点
uint8_t ShowConfTempNowNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 164, 0x00,
		30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示当前温度数字4
uint8_t ShowConfTempNowNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示当前温度设置单位
uint8_t ShowConfTempNowUnitCMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 162, 0x00,
		81, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA设置数字1
uint8_t ShowConf4mANum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 62, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA设置数字2
uint8_t ShowConf4mANum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 62, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA设置数字3-小数点
uint8_t ShowConf4mANum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 62, 0x00, 30,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA设置数字4
uint8_t ShowConf4mANum4CMD[13] = { 0xEE, 0x32, 0x01, 64, 0x00, 62, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA设置数字5
uint8_t ShowConf4mANum5CMD[13] = { 0xEE, 0x32, 0x01, 84, 0x00, 62, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA设置单位
uint8_t ShowConf4mAUnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 58, 0x00, 27,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA设置数字1
uint8_t ShowConf20mANum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 114, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA设置数字2
uint8_t ShowConf20mANum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 114, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA设置数字3-小数点
uint8_t ShowConf20mANum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00, 30,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA设置数字4
uint8_t ShowConf20mANum4CMD[13] = { 0xEE, 0x32, 0x01, 64, 0x00, 114, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA设置数字5
uint8_t ShowConf20mANum5CMD[13] = { 0xEE, 0x32, 0x01, 84, 0x00, 114, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA设置单位
uint8_t ShowConf20mAUnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 109, 0x00, 27,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA温度设置数字1
uint8_t ShowConf4mATempNum1CMD[13] = { 0xEE, 0x32, 0x01, 58, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA温度设置数字2
uint8_t ShowConf4mATempNum2CMD[13] = { 0xEE, 0x32, 0x01, 78, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA温度设置数字3-小数点
uint8_t ShowConf4mATempNum3CMD[13] = { 0xEE, 0x32, 0x01, 98, 0x00, 164, 0x00,
		30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA温度设置数字4
uint8_t ShowConf4mATempNum4CMD[13] = { 0xEE, 0x32, 0x01, 108, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA温度设置数字5
uint8_t ShowConf4mATempNum5CMD[13] = { 0xEE, 0x32, 0x01, 128, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示4mA温度设置单位
uint8_t ShowConf4mATempUnitCMD[13] = { 0xEE, 0x32, 0x01, 144, 0x00, 161, 0x00,
		81, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA温度设置数字1
uint8_t ShowConf20mATempNum1CMD[13] = { 0xEE, 0x32, 0x01, 58, 0x00, 214, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA温度设置数字2
uint8_t ShowConf20mATempNum2CMD[13] = { 0xEE, 0x32, 0x01, 78, 0x00, 214, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA温度设置数字3-小数点
uint8_t ShowConf20mATempNum3CMD[13] = { 0xEE, 0x32, 0x01, 98, 0x00, 214, 0x00,
		30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA温度设置数字4
uint8_t ShowConf20mATempNum4CMD[13] = { 0xEE, 0x32, 0x01, 108, 0x00, 214, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA温度设置数字5
uint8_t ShowConf20mATempNum5CMD[13] = { 0xEE, 0x32, 0x01, 128, 0x00, 214, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示20mA温度设置单位
uint8_t ShowConf20mATempUnitCMD[13] = { 0xEE, 0x32, 0x01, 144, 0x00, 211, 0x00,
		81, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示报警延时单位-秒
uint8_t ShowConfLimitDelayUnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 159,
		0x00, 27, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警状态
uint8_t ShowConfUpLimitAutoCMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 62, 0x00, 51,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警设置数字1
uint8_t ShowConfUpLimitNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警设置数字2
uint8_t ShowConfUpLimitNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警设置数字3-小数点
uint8_t ShowConfUpLimitNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00,
		30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警设置数字4
uint8_t ShowConfUpLimitNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警设置数字5
uint8_t ShowConfUpLimitNum5CMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警设置单位
uint8_t ShowConfUpLimitUnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 109, 0x00,
		27, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警延时时间数字1
uint8_t ShowConfUpLimitDelayNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警延时时间数字2
uint8_t ShowConfUpLimitDelayNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警延时时间数字3-小数点
uint8_t ShowConfUpLimitDelayNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 164,
		0x00, 30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警延时时间数字4
uint8_t ShowConfUpLimitDelayNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示高点报警延时时间数字5
uint8_t ShowConfUpLimitDelayNum5CMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警状态
uint8_t ShowConfLowLimitAutoCMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 62, 0x00,
		51, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警设置数字1
uint8_t ShowConfLowLimitNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警设置数字2
uint8_t ShowConfLowLimitNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警设置数字3-小数点
uint8_t ShowConfLowLimitNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00,
		30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警设置数字4
uint8_t ShowConfLowLimitNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警设置数字5
uint8_t ShowConfLowLimitNum5CMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 114, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警设置单位
uint8_t ShowConfLowLimitUnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 109, 0x00,
		27, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警延时时间数字1
uint8_t ShowConfLowLimitDelayNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警延时时间数字2
uint8_t ShowConfLowLimitDelayNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警延时时间数字3-小数点
uint8_t ShowConfLowLimitDelayNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 164,
		0x00, 30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警延时时间数字4
uint8_t ShowConfLowLimitDelayNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示低点报警延时时间数字5
uint8_t ShowConfLowLimitDelayNum5CMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 164,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示清洗持续时间数字1
uint8_t ShowConfWashHoldTimeNum1CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 62,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示清洗持续时间数字2
uint8_t ShowConfWashHoldTimeNum2CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 62,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示清洗持续时间单位-秒
uint8_t ShowConfWashHoldTimeUnitCMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 62,
		0x00, 61, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示清洗间隔时间数字1
uint8_t ShowConfWashDelayTimeNum1CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 114,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示清洗间隔时间数字2
uint8_t ShowConfWashDelayTimeNum2CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114,
		0x00, 31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示清洗间隔时间单位-小时
uint8_t ShowConfWashDelayTimeUnitCMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 112,
		0x00, 85, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示滤波系数数字1
uint8_t ShowConfFilterNum1CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 164, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示滤波系数数字2
uint8_t ShowConfFilterNum2CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 164, 0x00, 31,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示滤波系数单位
uint8_t ShowConfFilterUnitCMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 164, 0x00, 61,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示亮度设置数字1
uint8_t ShowConfBrightnessNum1CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 214, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示亮度设置数字2
uint8_t ShowConfBrightnessNum2CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 214, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//亮度设置(第一个255激活亮度|第二个255待机亮度|0～65535进入待机)
uint8_t ChangeBrightnessCMD[11] = { 0xEE, 0x77, 0x03, 255, 255, 0, 0, 0xFF,
		0xFC, 0xFF, 0xFF };
//显示校准界面1
uint8_t ShowCalPage1CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 64,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示校准界面2
uint8_t ShowCalPage2CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 86,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示校正模式
uint8_t ShowCalTypeCMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 64, 0x00, 87, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//显示校正大气压单位
uint8_t ShowAirPressureUnitCMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 112, 0x00,
		24, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示大气压数字1
uint8_t ShowAirPressureNum1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示大气压数字2-小数点
uint8_t ShowAirPressureNum2CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 164, 0x00,
		30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示大气压数字3
uint8_t ShowAirPressureNum3CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示大气压数字4
uint8_t ShowAirPressureNum4CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示大气压数字5
uint8_t ShowAirPressureNum5CMD[13] = { 0xEE, 0x32, 0x01, 94, 0x00, 164, 0x00,
		31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示大气压单位
uint8_t ShowAirPressureNumUnitCMD[13] = { 0xEE, 0x32, 0x01, 114, 0x00, 164,
		0x00, 24, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示正在校正状态
uint8_t ShowCalingStatusCMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 110, 0x00, 44,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示校正完成状态1
uint8_t ShowCalFinishStatus1CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 110, 0x00,
		63, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
//显示校正完成状态2
uint8_t ShowCalFinishStatus2CMD[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 110, 0x00,
		62, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示校正修正值单位
uint8_t ShowCalPPMFixedUnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 159, 0x00,
		27, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示历史界面1
uint8_t ShowHistoryPage1CMD[13] = { 0xEE, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		98, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

//显示历史时间1 - 旧时间
uint8_t ShowHistoryTime1CMD[31] = { 0xEE, 0x20, 0x00, 184, 0x00, 22, 0x00, 0x01,
		0x32, 0x30, 0x31, 0x39, 0x2F, 0x30, 0x38, 0x2F, 0x33, 0x31, 0x20, 0x32,
		0x33, 0x3A, 0x30, 0x30, 0x20, 0x2D, 0x20, 0xFF, 0xFC, 0xFF, 0xFF };
//显示历史时间2 - 新时间
uint8_t ShowHistoryTime2CMD[28] = { 0xEE, 0x20, 0x01, 70, 0x00, 22, 0x00, 0x01,
		0x32, 0x30, 0x31, 0x39, 0x2F, 0x30, 0x39, 0x2F, 0x30, 0x35, 0x20, 0x32,
		0x33, 0x3A, 0x30, 0x30, 0xFF, 0xFC, 0xFF, 0xFF };
//显示200mS/cm
uint8_t Show200mScmCMD[20] = { 0xEE, 0x20, 0x00, 6, 0x00, 46, 0x00, 0x00, 0x32,
		0x30, 0x30, 0x6D, 0x53, 0x2F, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示150mS/cm
uint8_t Show150mScmCMD[20] = { 0xEE, 0x20, 0x00, 6, 0x00, 96, 0x00, 0x00, 0x31,
		0x35, 0x30, 0x6D, 0x53, 0x2F, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示100mS/cm
uint8_t Show100mScmCMD[20] = { 0xEE, 0x20, 0x00, 6, 0x00, 146, 0x00, 0x00, 0x31,
		0x30, 0x30, 0x6D, 0x53, 0x2F, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示50mS/cm
uint8_t Show50mScmCMD[19] = { 0xEE, 0x20, 0x00, 12, 0x00, 196, 0x00, 0x00, 0x35,
		0x30, 0x6D, 0x53, 0x2F, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示0mS/cm
uint8_t Show0mScmCMD[18] = { 0xEE, 0x20, 0x00, 20, 0x00, 246, 0x00, 0x00, 0x30,
		0x6D, 0x53, 0x2F, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };

//显示20兆欧姆.cm
uint8_t Show20MRCMD[20] = { 0xEE, 0x20, 0x00, 10, 0x00, 46, 0x00, 0x00, 0x32,
		0x30, 0x4D, 0xA6, 0xB8, 0x2E, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示15兆欧姆.cm
uint8_t Show15MRCMD[20] = { 0xEE, 0x20, 0x00, 10, 0x00, 96, 0x00, 0x00, 0x31,
		0x35, 0x4D, 0xA6, 0xB8, 0x2E, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示10兆欧姆.cm
uint8_t Show10MRCMD[20] = { 0xEE, 0x20, 0x00, 10, 0x00, 146, 0x00, 0x00, 0x31,
		0x30, 0x4D, 0xA6, 0xB8, 0x2E, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示5兆欧姆.cm
uint8_t Show5MRCMD[19] = { 0xEE, 0x20, 0x00, 16, 0x00, 196, 0x00, 0x00, 0x35,
		0x4D, 0xA6, 0xB8, 0x2E, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };
//显示0兆欧姆.cm
uint8_t Show0MRCMD[19] = { 0xEE, 0x20, 0x00, 16, 0x00, 246, 0x00, 0x00, 0x30,
		0x4D, 0xA6, 0xB8, 0x2E, 0x63, 0x6D, 0xFF, 0xFC, 0xFF, 0xFF };

//显示70ppt
uint8_t Show700pptCMD[20] = { 0xEE, 0x20, 0x00, 16, 0x00, 46, 0x00, 0x00, 0x37,
		0x30, 0x2E, 0x30, 0x70, 0x70, 0x74, 0xFF, 0xFC, 0xFF, 0xFF };
//显示52.5ppt
uint8_t Show525pptCMD[19] = { 0xEE, 0x20, 0x00, 16, 0x00, 96, 0x00, 0x00, 0x35,
		0x32, 0x2E, 0x35, 0x70, 0x70, 0x74, 0xFF, 0xFC, 0xFF, 0xFF };
//显示35ppt
uint8_t Show350pptCMD[19] = { 0xEE, 0x20, 0x00, 16, 0x00, 146, 0x00, 0x00, 0x33,
		0x35, 0x2E, 0x30, 0x70, 0x70, 0x74, 0xFF, 0xFC, 0xFF, 0xFF };
//显示17.5ppt
uint8_t Show175pptCMD[19] = { 0xEE, 0x20, 0x00, 16, 0x00, 196, 0x00, 0x00, 0x31,
		0x37, 0x2E, 0x35, 0x70, 0x70, 0x74, 0xFF, 0xFC, 0xFF, 0xFF };
//显示0ppt
uint8_t Show0pptCMD[19] = { 0xEE, 0x20, 0x00, 16, 0x00, 246, 0x00, 0x00, 0x30,
		0x2E, 0x30, 0x30, 0x70, 0x70, 0x74, 0xFF, 0xFC, 0xFF, 0xFF };

//获取传感器数据
uint8_t GetSensor[8] = { 0x03, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC4, 0x2F };

//是否需要刷新标志位
uint8_t refreshFlag = 0;

//进入设置标志位
uint8_t configFlag = 0;

//进入校准标志位
uint8_t calFlag = 0;

//进入历史标志位
uint8_t historyFlag = 0;

//传感器接收数据
uint8_t sensorRevBuf[50];

float f_Rs;
//修正值
float f_Rs_fixed[60];
//滤波数组
float f_Rs_filter;
//滤波计数器
uint8_t filterCNT = 0;

float f_Temp;
float f_Temp_fixed;
float f_k;

//float f_PPM_Uplimit = 20;
//float f_PPM_Downlimit = 0;

//继电器标志位
uint8_t Relay_Flag = 1;			//0:低位报警 | 1:正常 | 2:高位报警
uint8_t Relay_Flag_Low = 0;
uint8_t Relay_Flag_Up = 0;
uint8_t Relay_WASH_Flag = 0;	//0:不清洗 | 1:清洗

//按钮时间标志位
uint64_t Button_Conf_Flag = 0;
uint64_t Button_Cal_Flag = 0;
uint64_t Button_Right_Flag = 0;

//获取传感器数据时间标志位
long Sensor_Time = 0;
//传感器状态标志位
long Sensor_Status = 0;
//校准结果状态
uint8_t result = 0;

//flash保存参数
typedef struct {
	uint8_t mode;
	float tempfactor;
	float temp;
	float ppm4mA;
	float ppm20mA;
	float temp4mA;
	float temp20mA;
	uint8_t uplimitauto;
	float uplimit;
	float uplimitdelay;
	uint8_t lowlimitauto;
	float lowlimit;
	float lowlimitdelay;
	uint8_t washholdtime;
	uint8_t washdelaytime;
	uint8_t filter;
	uint8_t brightness;
	uint8_t caltype;
	uint8_t airpressureunit;
	float cell;
	float kdo;
	float ktemp;
	uint8_t interval;
	float bdo;
	float MIN;
	float MAX;
	uint8_t password;
	uint8_t tempfactortype;
} SAVEDATA;
SAVEDATA savedata;
SAVEDATA tempdata;

uint8_t Flashed = 0;
uint8_t SPI_Flashed = 0;
/* 当前时间 */
RTC_DateTypeDef sdatestructureget;
RTC_TimeTypeDef stimestructureget;
/* 设置时间 */
RTC_DateTypeDef sdateconfstructureget;
RTC_TimeTypeDef stimeconfstructureget;
/* 设置对比时间是否变化 */
RTC_DateTypeDef sdateconftempstructureget;
RTC_TimeTypeDef stimeconftempstructureget;
/* 上次清洗继电器时间 */
RTC_DateTypeDef sdatewashstructureget;
RTC_TimeTypeDef stimewashstructureget;
/* 清洗继电器启动时间 */
RTC_DateTypeDef sdatewashholdstructureget;
RTC_TimeTypeDef stimewashholdstructureget;
/* 报警延迟时间 */
RTC_DateTypeDef sdatelowlimitdelaystructureget;
RTC_TimeTypeDef stimelowlimitdelaystructureget;
RTC_DateTypeDef sdateuplimitdelaystructureget;
RTC_TimeTypeDef stimeuplimitdelaystructureget;
/* 掉电时间 */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef DateToUpdate;

uint8_t lastHours = 25;
#define FLASHBASEADDR 0x0800F800
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void readConfig(void);
void writeConfig(void);
void application(void);
void LCD_Update(void);
void ImA_Update(void);
void Relay_Update(void);
void RTC_Update(void);
long calcTimeSpan(uint8_t datenowYear, uint8_t datenowMonth,
		uint8_t datenowDate, uint8_t timenowHours, uint8_t timenowMinutes,
		uint8_t timenowSeconds, uint8_t dateoldYear, uint8_t dateoldMonth,
		uint8_t dateoldDate, uint8_t timeoldHours, uint8_t timeoldMinutes,
		uint8_t timeoldSeconds);
void LCD_Init(void);
void getRs(void);
void Enter_Conf_Page1(void);
void Enter_Conf_Page1_2(void);
void Enter_Conf_Page2(void);
void Enter_Conf_Page3(void);
void Enter_Conf_Page4(void);
void Enter_Conf_Page5(void);
void Enter_Conf_Page6(void);
void Enter_Conf_Page7(void);
void Enter_Conf_Page8(void);
uint8_t Enter_PasW_Page(uint8_t lastPage);
void Enter_Cal_Page1(void);
void Enter_Cal_Page2(void);
void Change_Conf_Unit(uint8_t mode);
//void Change_Conf_Salinity(float salinity);
void Change_Conf_Temp(float temp);
void Change_Conf_Temp_Now(float temp);
void Change_Conf_TempFactorType(uint8_t TempFactorType);
void Change_Conf_TempFactor(float Factor);
void Change_Conf_PPM4mA(float ppm4mA);
void Change_Conf_ppm20mA(float ppm20mA);
void Change_Conf_temp4mA(float temp4mA);
void Change_Conf_temp20mA(float temp20mA);
void Change_Conf_ADMAX(float temp20mA);
void Change_Conf_ADMIN(float temp4mA);
void Change_Conf_UpLimitAuto(uint8_t uplimitauto);
void Change_Conf_UpLimit(float uplimit);
void Change_Conf_UpLimitDelay(float delaytime);
void Change_Conf_LowLimitAuto(uint8_t uplimitauto);
void Change_Conf_LowLimit(float lowlimit);
void Change_Conf_LowLimitDelay(float delaytime);
void Change_Conf_WashHoldTime(uint8_t washholdtime);
void Change_Conf_WashDelayTime(uint8_t washdelaytime);
void Change_Conf_Filter(uint8_t filter);
void Change_Conf_Brightness(uint8_t brightness);
void Change_Conf_Year(uint8_t year);
void Change_Conf_Month(uint8_t month);
void Change_Conf_Date(uint8_t date);
void Change_Conf_Hours(uint8_t hours);
void Change_Conf_Minutes(uint8_t minutes);
void Change_Conf_Seconds(uint8_t seconds);
void Change_Conf_Interval(uint8_t interval);
void Change_Conf_PassW(uint8_t PassW);
void Change_CalType(uint8_t caltype);
void Change_AirPressureUnit(uint8_t unit);
void Change_AirPressure(float cell);
void Select_Next(uint8_t Selection);
void Conf_UI(void);
void Cal_UI(void);
void History_UI(void);
void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void UART_RxIDLECallback(UART_HandleTypeDef *uartHandle);
unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
void change_float_big_485rom(unsigned int j);
void Button_Scan(void);
void eepromReadSetting(void);
void eepromWriteSetting(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_DAC_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_SPI2_Init();
	MX_RTC_Init();
//	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	/* 检测数据是否保存在RTC备份寄存器1：如果已经保存就不需要运行日期和时间设置 */
	/* 读取备份寄存器1数据 */
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F1) {
		/* 配置RTC万年历：时间和日期 */
		RTC_CalendarConfig();
	} else {
		DateToUpdate.Year = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
		DateToUpdate.Month = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
		DateToUpdate.Date = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5);
		HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	}
	/* 获取当前时间 */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* 获取当前日期 */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	memcpy(&sdatewashstructureget, &sdatestructureget,
			sizeof(sdatestructureget));
	memcpy(&stimewashstructureget, &stimestructureget,
			sizeof(stimestructureget));
	/* 前后总共15s启动LOGO延时, 保证传感器开机时间 */
	HAL_Delay(3000);
	/* 读取EEPROM */
	readConfig();
	eepromReadSetting();
	if (SPI_Flashed != 99) {
		uint16_t i;
		historyCNT = 0;
		historyEnd = 0;
		historyStart = 0;
		SPI_Flashed = 99;
		for (i = 0; i < 400; i++) {
			History_PPM[i] = 0;
			memcpy(&History_DATE[i], &sdatestructureget,
					sizeof(sdatestructureget));
			memcpy(&History_TIME[i], &stimestructureget,
					sizeof(stimestructureget));
		}
		eepromWriteSetting();
	}
	eepromReadSetting();
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, savedata.MIN);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, savedata.MIN);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	/* 预转换获取电导，保证开机有数据 */
	getRs();
	/* 前后总共15s启动LOGO延时, 保证传感器开机时间 */
//	HAL_Delay(3000);
//	/* 开机获取传感器数据 */
//	getRs();
//	HAL_Delay(1500);
//	getRs();
//	HAL_Delay(1500);
//	getRs();
//	HAL_Delay(1500);
	/* 防止未接传感器，开机误报低报警 */
	if (refreshFlag == 0) {
		Sensor_Status = 1;
	}

	/* 初始化主界面 */
	LCD_Init();

	refreshFlag = 0;
	Sensor_Status = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		application();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/**
 * @功能简介 : 从EEPROM读取用户配置
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void readConfig(void) {
	Flashed = *(__IO uint8_t*) (FLASHBASEADDR);
	if (Flashed == 0x99) {
		Flashed = *(__IO uint8_t*) (FLASHBASEADDR);
		savedata.mode = *(__IO uint8_t*) (FLASHBASEADDR + 8);
		savedata.tempfactor = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 16)))
				/ 1000;
		savedata.temp = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 24)))
				/ 1000;
		savedata.ppm4mA = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 32)))
				/ 1000;
		savedata.ppm20mA = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 40)))
				/ 1000;
		savedata.temp4mA = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 48)))
				/ 1000;
		savedata.temp20mA = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 56)))
				/ 1000;
		savedata.uplimitauto = *(__IO uint8_t*) (FLASHBASEADDR + 64);
		savedata.uplimit = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 72)))
				/ 1000;
		savedata.uplimitdelay =
				((float) (*(__IO uint8_t*) (FLASHBASEADDR + 80))) / 1000;
		savedata.lowlimitauto = *(__IO uint8_t*) (FLASHBASEADDR + 88);
		savedata.lowlimit = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 96)))
				/ 1000;
		savedata.lowlimitdelay =
				((float) (*(__IO uint8_t*) (FLASHBASEADDR + 104))) / 1000;
		savedata.washholdtime = *(__IO uint8_t*) (FLASHBASEADDR + 112);
		savedata.washdelaytime = *(__IO uint8_t*) (FLASHBASEADDR + 120);
		savedata.filter = *(__IO uint8_t*) (FLASHBASEADDR + 128);
		savedata.brightness = *(__IO uint8_t*) (FLASHBASEADDR + 136);
		savedata.caltype = *(__IO uint8_t*) (FLASHBASEADDR + 144);
		savedata.airpressureunit = *(__IO uint8_t*) (FLASHBASEADDR + 152);
		savedata.cell =
				((float) (*(__IO int32_t*) (FLASHBASEADDR + 160))) / 1000;
		savedata.kdo = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 168)))
				/ 1000;
		savedata.ktemp = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 176)))
				/ 1000;
		savedata.interval = *(__IO uint8_t*) (FLASHBASEADDR + 184);
		savedata.bdo = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 192)))
				/ 1000;
		savedata.MAX = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 200)))
				/ 1000;
		savedata.MIN = ((float) (*(__IO int32_t*) (FLASHBASEADDR + 208)))
				/ 1000;
		savedata.password = *(__IO uint8_t*) (FLASHBASEADDR + 216);
		savedata.tempfactortype = *(__IO uint8_t*) (FLASHBASEADDR + 224);
	} else {
		savedata.mode = 0;
		savedata.tempfactor = 2.0;
		savedata.temp = 0;
		savedata.ppm4mA = 0;
		savedata.ppm20mA = 200000;
		savedata.temp4mA = 0;
		savedata.temp20mA = 99.9;
		savedata.uplimitauto = 1;
		savedata.uplimit = 20;
		savedata.uplimitdelay = 0.10;
		savedata.lowlimitauto = 1;
		savedata.lowlimit = 0;
		savedata.lowlimitdelay = 0.10;
		savedata.washholdtime = 60;
		savedata.washdelaytime = 24;
		savedata.filter = 0;
		savedata.brightness = 10;
		savedata.caltype = 0;
		savedata.airpressureunit = 0;
		savedata.cell = 1.01325;
		savedata.kdo = 1;
		savedata.ktemp = 1;
		savedata.interval = 1;
		savedata.bdo = 0;
		/* 47 - 4mA | 235 - 20mA */
		savedata.MAX = ADMAX;
		savedata.MIN = ADMIN;
		savedata.password = 0;
		savedata.tempfactortype = 0;
		writeConfig();
	}
}

/**
 * @功能简介 : 写用户配置到EEPROM
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void writeConfig(void) {
	HAL_FLASH_Unlock();
	//2、擦除FLASH
	//初始化FLASH_EraseInitTypeDef
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = FLASHBASEADDR;
	f.NbPages = 1;
	//设置PageError
	uint32_t PageError = 0;
	//调用擦除函数
	HAL_FLASHEx_Erase(&f, &PageError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR), 0x99);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 8),
			savedata.mode);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 16),
			(int32_t) (savedata.tempfactor * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 24),
			(int32_t) (savedata.temp * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 32),
			(int32_t) (savedata.ppm4mA * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 40),
			(int32_t) (savedata.ppm20mA * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 48),
			(int32_t) (savedata.temp4mA * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 56),
			(int32_t) (savedata.temp20mA * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 64),
			savedata.uplimitauto);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 72),
			(int32_t) (savedata.uplimit * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 80),
			(int32_t) (savedata.uplimitdelay * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 88),
			savedata.lowlimitauto);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 96),
			(int32_t) (savedata.lowlimit * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 104),
			(int32_t) (savedata.lowlimitdelay * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 112),
			savedata.washholdtime);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 120),
			savedata.washdelaytime);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 128),
			savedata.filter);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 136),
			savedata.brightness);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 144),
			savedata.caltype);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 152),
			savedata.airpressureunit);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 160),
			(int32_t) (savedata.cell * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 168),
			(int32_t) (savedata.kdo * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 176),
			(int32_t) (savedata.ktemp * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 184),
			savedata.interval);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 192),
			(int32_t) (savedata.bdo * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 200),
			(int32_t) (savedata.MAX * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASHBASEADDR + 208),
			(int32_t) (savedata.MIN * 1000));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 216),
			savedata.password);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASHBASEADDR + 224),
			savedata.tempfactortype);
	HAL_FLASH_Lock();
}
/**
 * @功能简介 : 恢复出厂设置
 * @入口参数 : conf 1 - 参数恢复 | 2 - 校准恢复
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void factoryConfig(uint8_t conf) {
	if (conf == 1) {
		ShowPageStatusCMD[7] = 95;
		HAL_UART_Transmit(&huart1, ShowPageStatusCMD, 13, USARTSENDTIME);
		savedata.mode = 0;
		savedata.tempfactor = 2.0;
		savedata.temp = 0;
		savedata.ppm4mA = 0;
		savedata.ppm20mA = 20;
		savedata.temp4mA = 0;
		savedata.temp20mA = 99.9;
		savedata.uplimitauto = 1;
		savedata.uplimit = 20;
		savedata.uplimitdelay = 0.10;
		savedata.lowlimitauto = 1;
		savedata.lowlimit = 0;
		savedata.lowlimitdelay = 0.10;
		savedata.washholdtime = 60;
		savedata.washdelaytime = 24;
		savedata.filter = 0;
		savedata.brightness = 10;
		savedata.caltype = 0;
		savedata.interval = 1;
		savedata.MAX = ADMAX;
		savedata.MIN = ADMIN;
		savedata.password = 0;
		savedata.tempfactortype = 0;
		HAL_Delay(1500);
		writeConfig();
		HAL_Delay(1500);
		ShowPageStatusCMD[7] = 97;
		HAL_UART_Transmit(&huart1, ShowPageStatusCMD, 13, USARTSENDTIME);
	}
	if (conf == 2) {
		ShowPageStatusCMD[7] = 96;
		HAL_UART_Transmit(&huart1, ShowPageStatusCMD, 13, USARTSENDTIME);
		savedata.airpressureunit = 0;
		savedata.cell = 1.01325;
		savedata.kdo = 1;
		savedata.ktemp = 1;
		savedata.bdo = 0;
		HAL_Delay(1500);
		writeConfig();
		HAL_Delay(1500);
		ShowPageStatusCMD[7] = 97;
		HAL_UART_Transmit(&huart1, ShowPageStatusCMD, 13, USARTSENDTIME);
	}
}

/**
 * @功能简介 : 主程序循环
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void application(void) {
	/* 正常刷新数据 */
	if (refreshFlag == 1) {
#ifdef DEBUG
		uint8_t debugTemp1[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01,
				0x02, 0x30, 0x31, 0xFF, 0xFC, 0xFF, 0xFF };
		HAL_UART_Transmit(&huart1, debugTemp1, 14, USARTSENDTIME);
#endif
		refreshFlag = 0;
		Sensor_Status = 0;
		/* 刷新界面 */
		LCD_Update();
		/* 刷新4-20mA电流 */
		ImA_Update();
		/* 刷新继电器 */
		Relay_Update();
	}
	/* 获取传感器数据 */
	if (refreshFlag == 0 && configFlag == 0 && Sensor_Time == 100000) {
#ifdef DEBUG
		uint8_t debugTemp2[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01,
				0x02, 0x30, 0x32, 0xFF, 0xFC, 0xFF, 0xFF };
		HAL_UART_Transmit(&huart1, debugTemp2, 14, USARTSENDTIME);
#endif
		/* 显示更新时间和日期 */
		RTC_Update();
		getRs();
	}
	/* 扫描按键 */
	Button_Scan();

//	if (Sensor_Status == 0) {
//		ShowElectrodeStatusCMD[7] = 50;
//		HAL_UART_Transmit(&huart1, ShowElectrodeStatusCMD, 13,
//		USARTSENDTIME);
//	}
	/* 电极未接 */
	if (Sensor_Status >= 600000) {
#ifdef DEBUG
		uint8_t debugTemp3[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01,
				0x02, 0x34, 0x33, 0xFF, 0xFC, 0xFF, 0xFF };
		HAL_UART_Transmit(&huart1, debugTemp3, 14, USARTSENDTIME);
#endif
		Sensor_Status = 1;
		ShowElectrodeStatusCMD[7] = 46;
		HAL_UART_Transmit(&huart1, ShowElectrodeStatusCMD, 13,
		USARTSENDTIME);
		f_Rs_filter = 0;
		f_Temp = 0;
		filterCNT = 0;
		/* 刷新界面 */
		LCD_Update();
		/* 刷新4-20mA电流 */
		ImA_Update();
		/* 刷新继电器 */
		Relay_Update();
	}

	/* 备份时间 */
	if (lastHours != stimestructureget.Hours) {
#ifdef DEBUG
		uint8_t debugTemp4[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01,
				0x02, 0x30, 0x34, 0xFF, 0xFC, 0xFF, 0xFF };
		HAL_UART_Transmit(&huart1, debugTemp4, 14, USARTSENDTIME);
#endif
		lastHours = stimestructureget.Hours;
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F1);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, sdatestructureget.Year);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, sdatestructureget.Month);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, sdatestructureget.Date);
		/* 保存历史数据 */
		if (stimestructureget.Minutes == 0) {
#ifdef DEBUG
			uint8_t debugTemp5[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01,
					0x02, 0x35, 0x35, 0xFF, 0xFC, 0xFF, 0xFF };
			HAL_UART_Transmit(&huart1, debugTemp5, 14, USARTSENDTIME);
#endif
			if (calcTimeSpan(sdatestructureget.Year, sdatestructureget.Month,
					sdatestructureget.Date, stimestructureget.Hours,
					stimestructureget.Minutes, stimestructureget.Seconds,
					History_DATE[historyEnd].Year,
					History_DATE[historyEnd].Month,
					History_DATE[historyEnd].Date,
					History_TIME[historyEnd].Hours,
					History_TIME[historyEnd].Minutes,
					History_TIME[historyEnd].Seconds)
					> savedata.interval * 3600) {
				memcpy(&History_DATE[historyEnd], &sdatestructureget,
						sizeof(sdatestructureget));
				memcpy(&History_TIME[historyEnd], &stimestructureget,
						sizeof(stimestructureget));
				if (historyCNT == 399) {
					historyStart++;
					historyEnd++;
					if (historyStart > 399) {
						historyStart = 0;
					}
					if (historyEnd > 399) {
						historyEnd = 0;
					}
					History_PPM[historyEnd] = f_Rs_filter;
					memcpy(&History_DATE[historyEnd], &sdatestructureget,
							sizeof(sdatestructureget));
					memcpy(&History_TIME[historyEnd], &stimestructureget,
							sizeof(stimestructureget));
				}
				if (historyCNT < 399) {
					historyCNT++;
					historyEnd++;
					if (historyEnd > 399) {
						historyEnd = 0;
					}
					History_PPM[historyEnd] = f_Rs_filter;
					memcpy(&History_DATE[historyEnd], &sdatestructureget,
							sizeof(sdatestructureget));
					memcpy(&History_TIME[historyEnd], &stimestructureget,
							sizeof(stimestructureget));
				}
				eepromWriteSetting();
			}
			/* 清洗时间计算 */
			if (calcTimeSpan(sdatestructureget.Year, sdatestructureget.Month,
					sdatestructureget.Date, stimestructureget.Hours,
					stimestructureget.Minutes, stimestructureget.Seconds,
					sdatewashstructureget.Year, sdatewashstructureget.Month,
					sdatewashstructureget.Date, stimewashstructureget.Hours,
					stimewashstructureget.Minutes,
					stimewashstructureget.Seconds)
					> savedata.washdelaytime * 3600) {
				memcpy(&sdatewashstructureget, &sdatestructureget,
						sizeof(sdatestructureget));
				memcpy(&stimewashstructureget, &stimestructureget,
						sizeof(stimestructureget));
				memcpy(&sdatewashholdstructureget, &sdatestructureget,
						sizeof(sdatestructureget));
				memcpy(&stimewashholdstructureget, &stimestructureget,
						sizeof(stimestructureget));
				Relay_WASH_Flag = 1;
			}
		}
	}

	/* 延时报警 */
	if (Relay_Flag == 0 && Relay_Flag_Low == 0) {
		Relay_Flag_Low = 1;
	}
	if (Relay_Flag == 2 && Relay_Flag_Up == 0) {
		Relay_Flag_Up = 1;
	}

	if (Relay_WASH_Flag == 2) {
#ifdef DEBUG
		uint8_t debugTemp6[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01,
				0x02, 0x30, 0x36, 0xFF, 0xFC, 0xFF, 0xFF };
		HAL_UART_Transmit(&huart1, debugTemp6, 14, USARTSENDTIME);
#endif
		if (calcTimeSpan(sdatestructureget.Year, sdatestructureget.Month,
				sdatestructureget.Date, stimestructureget.Hours,
				stimestructureget.Minutes, stimestructureget.Seconds,
				sdatewashholdstructureget.Year, sdatewashholdstructureget.Month,
				sdatewashholdstructureget.Date, stimewashholdstructureget.Hours,
				stimewashholdstructureget.Minutes,
				stimewashholdstructureget.Seconds) > savedata.washholdtime) {
			Relay_WASH_Flag = 0;
		}
	}

	/* 传感器状态判断 */
	if (Sensor_Time > 300000) {
		Sensor_Time = 0;
	}
	Sensor_Time++;
	Sensor_Status++;
}

/**
 * @功能简介 : 更新UI
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/08/03
 */
void LCD_Update(void) {
	uint8_t u_10PPM = 0, u_1PPM = 0, u_01PPM = 0, u_001PPM = 0;
	uint8_t u_10mA, u_1mA, u_01mA;
	uint8_t u_10Temp, u_1Temp, u_01Temp;
	uint8_t index = 1;
	float f_Rs_Show = 0;

#ifdef DEBUG
	uint8_t debugTemp7[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01, 0x02,
			0x30, 0x37, 0xFF, 0xFC, 0xFF, 0xFF };
	HAL_UART_Transmit(&huart1, debugTemp7, 14, USARTSENDTIME);
#endif
	float f_mA = 16 * f_Rs_filter / (savedata.ppm20mA - savedata.ppm4mA) + 20
			- 16 * savedata.ppm20mA / (savedata.ppm20mA - savedata.ppm4mA);
	if (f_mA < 4) {
		f_mA = 4;
	}
	if (f_mA > 20) {
		f_mA = 20;
	}
	//TODO
	/* 根据电导率切换单位 暂时屏蔽 */
//	if (f_Rs_filter > 15000000) {
//		printf("%fΩ%s", f_Rs_filter / 1000000, "\r\n");
//	} else if (f_Rs_filter > 1000000) {
//		printf("%03.03fMΩ%s", f_Rs_filter / 1000000, "\r\n");
//	} else if (f_Rs_filter > 1000) {
//		printf("%03.03fKΩ%s", Rs / 1000, "\r\n");
//	} else if (f_Rs_filter) {
//		printf("%03.03fΩ%s", f_Rs_filter, "\r\n");
//	}
	switch (savedata.mode) {
	case 0:
		/* 电导率 */
		if (f_Rs_filter < 10000) {
			ShowMainUnitCMD[7] = 27;
			f_Rs_Show = f_Rs_filter;
		} else {
			ShowMainUnitCMD[7] = 28;
			f_Rs_Show = f_Rs_filter / 1000;
		}
		break;
	case 1:
		/* 电阻率 */
		ShowMainUnitCMD[7] = 29;
		/** 纯水电阻率10MΩ*cm，则相应的电导率为0.1μS/cm
		 **/
		f_Rs_Show = 1 / f_Rs_filter;
		break;
	case 2:
		/* 盐度 */
		/* 若盐度值（以NaCl计算）记为yNaCl（单位为PPm），电导率值记为x（单位为μs/cm），当前水温为t，则换算公式为: */
		/* yNaCl=1.3888*x-0.02478*x*t-6171.9 */
		ShowMainUnitCMD[7] = 84;
		f_Rs_Show = (1.3888 * f_Rs_filter - 0.02478 * f_Rs_filter * f_Temp_fixed
				- 6171.9) * 1000000;
		break;
	}
	if (f_Rs_Show >= 1000) {
		ShowMainNum1CMD[2] = 0x00;
		ShowMainNum1CMD[3] = 86;
		ShowMainNum2CMD[2] = 0x00;
		ShowMainNum2CMD[3] = 156;
		ShowMainNum3CMD[2] = 0x01;
		ShowMainNum3CMD[3] = 25;
		ShowMainNum4CMD[2] = 0x00;
		ShowMainNum4CMD[3] = 227;
		ShowMainNum5CMD[2] = 0x01;
		ShowMainNum5CMD[3] = 46;
		u_10PPM = f_Rs_Show / 1000;
		u_1PPM = (f_Rs_Show - u_10PPM * 1000) / 100;
		u_01PPM = (f_Rs_Show - u_10PPM * 1000 - u_1PPM * 100) / 10;
		u_001PPM = f_Rs_Show - u_10PPM * 1000 - u_1PPM * 100 - u_01PPM * 10;
	} else if (f_Rs_Show >= 100 && f_Rs_Show < 1000) {
		ShowMainNum1CMD[2] = 0x00;
		ShowMainNum1CMD[3] = 86;
		ShowMainNum2CMD[2] = 0x00;
		ShowMainNum2CMD[3] = 150;
		ShowMainNum3CMD[2] = 0x01;
		ShowMainNum3CMD[3] = 25;
		ShowMainNum4CMD[2] = 0x00;
		ShowMainNum4CMD[3] = 215;
		ShowMainNum5CMD[2] = 0x01;
		ShowMainNum5CMD[3] = 46;
		u_10PPM = f_Rs_Show / 100;
		u_1PPM = (f_Rs_Show - u_10PPM * 100) / 10;
		u_01PPM = f_Rs_Show - u_10PPM * 100 - u_1PPM * 10;
		u_001PPM = (f_Rs_Show - u_10PPM * 100 - u_1PPM * 10 - u_01PPM) * 10;
	} else if (f_Rs_Show < 100 && f_Rs_Show >= 10) {
		ShowMainNum1CMD[2] = 0x00;
		ShowMainNum1CMD[3] = 86;
		ShowMainNum2CMD[2] = 0x00;
		ShowMainNum2CMD[3] = 150;
		ShowMainNum3CMD[2] = 0x00;
		ShowMainNum3CMD[3] = 215;
		ShowMainNum4CMD[2] = 0x00;
		ShowMainNum4CMD[3] = 238;
		ShowMainNum5CMD[2] = 0x01;
		ShowMainNum5CMD[3] = 46;
		u_10PPM = f_Rs_Show / 10;
		u_1PPM = f_Rs_Show - u_10PPM * 10;
		u_01PPM = (f_Rs_Show - u_10PPM * 10 - u_1PPM) * 10;
		u_001PPM = (f_Rs_Show - u_10PPM * 10 - u_1PPM - u_01PPM * 0.1) * 100;
	} else if (f_Rs_Show < 10) {
		ShowMainNum1CMD[2] = 0x00;
		ShowMainNum1CMD[3] = 86;
		ShowMainNum2CMD[2] = 0x00;
		ShowMainNum2CMD[3] = 173;
		ShowMainNum3CMD[2] = 0x00;
		ShowMainNum3CMD[3] = 150;
		ShowMainNum4CMD[2] = 0x00;
		ShowMainNum4CMD[3] = 238;
		ShowMainNum5CMD[2] = 0x01;
		ShowMainNum5CMD[3] = 46;
		u_10PPM = f_Rs_Show;
		u_1PPM = (f_Rs_Show - u_10PPM) * 10;
		u_01PPM = (f_Rs_Show - u_10PPM - u_1PPM * 0.1) * 100;
		u_001PPM = (f_Rs_Show - u_10PPM - u_1PPM * 0.1 - u_01PPM * 0.01) * 1000;
	}

	if ((f_Rs_filter > savedata.lowlimit && f_Rs_filter < savedata.uplimit)
			&& Sensor_Status == 0) {
		index = 1;
		ShowElectrodeStatusCMD[7] = 49;
		Relay_Flag = 1;
		Relay_Flag_Low = 0;
		Relay_Flag_Up = 0;
	} else if ((f_Rs_filter <= savedata.lowlimit) && (Sensor_Status == 0)
			&& (savedata.lowlimitauto == 1)) {
		index = 11;
		ShowElectrodeStatusCMD[7] = 47;
		if (Relay_Flag != 0) {
			Relay_Flag = 0;
//			memcpy(&sdatelowlimitdelaystructureget, &sdatestructureget,
//					sizeof(sdatestructureget));
//			memcpy(&stimelowlimitdelaystructureget, &stimestructureget,
//					sizeof(stimestructureget));
		}
	} else if ((f_Rs_filter >= savedata.uplimit) && (Sensor_Status == 0)
			&& (savedata.uplimitauto == 1)) {
		index = 11;
		ShowElectrodeStatusCMD[7] = 48;
		if (Relay_Flag != 2) {
			Relay_Flag = 2;
//			memcpy(&sdateuplimitdelaystructureget, &sdatestructureget,
//					sizeof(sdatestructureget));
//			memcpy(&stimeuplimitdelaystructureget, &stimestructureget,
//					sizeof(stimestructureget));
		}
	} else if (Sensor_Status) {
		ShowElectrodeStatusCMD[7] = 46;
		Relay_Flag = 1;
		Relay_Flag_Low = 0;
		Relay_Flag_Up = 0;
	}
//	if (Sensor_Status != 0) {
//		Relay_Flag = 1;
//	}
	u_10mA = f_mA / 10;
	u_1mA = f_mA - u_10mA * 10;
	u_01mA = (f_mA - u_10mA * 10 - u_1mA) * 10;
	if (u_10PPM != 0 || f_Rs_Show < 10) {
		ShowMainNum1CMD[7] = u_10PPM + index;
	} else
		ShowMainNum1CMD[7] = 104;
	if (u_10mA != 0) {
		ShowmANum1CMD[7] = u_10mA + 31;
	} else
		ShowmANum1CMD[7] = 91;
	ShowMainNum2CMD[7] = u_1PPM + index;
	ShowmANum2CMD[7] = u_1mA + 31;
	ShowMainNum3CMD[7] = (index == 1 ? 21 : 22);
	if (f_Rs_Show > 999) {
		ShowMainNum3CMD[7] = 104;
	} else {
		ShowMainNum3CMD[7] = (index == 1 ? 21 : 22);
	}
	ShowMainNum4CMD[7] = u_01PPM + index;
	ShowmANum4CMD[7] = u_01mA + 31;
	ShowMainNum5CMD[7] = u_001PPM + index;

	u_10Temp = f_Temp_fixed / 10;
	u_1Temp = f_Temp_fixed - u_10Temp * 10;
	u_01Temp = (f_Temp_fixed - u_10Temp * 10 - u_1Temp) * 10;
	if (u_10Temp != 0) {
		ShowTempNum1CMD[7] = u_10Temp + 70;
	} else
		ShowTempNum1CMD[7] = 103;
	ShowTempNum2CMD[7] = u_1Temp + 70;
	ShowTempNum4CMD[7] = u_01Temp + 70;

	HAL_UART_Transmit(&huart1, ShowMainUnitCMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowMainNum1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowMainNum3CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowMainNum2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowMainNum4CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowMainNum5CMD, 13, USARTSENDTIME);

	HAL_UART_Transmit(&huart1, ShowTempNum1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempNum2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempNum3CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempNum4CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowElectrodeStatusCMD, 13, USARTSENDTIME);
	/* 清洗状态 - 屏蔽 */
//	HAL_UART_Transmit(&huart1, ShowWashStatusCMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum3CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum4CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 根据传感器采集值更新PPM和温度的4-20mA电流信号
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/08/17
 */
void ImA_Update(void) {
	int32_t PPMDAC = 0, TempDAC = 0;
#ifdef DEBUG
	uint8_t debugTemp8[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01, 0x02,
			0x30, 0x38, 0xFF, 0xFC, 0xFF, 0xFF };
	HAL_UART_Transmit(&huart1, debugTemp8, 14, USARTSENDTIME);
#endif
	PPMDAC = (16 * f_Rs_filter / (savedata.ppm20mA - savedata.ppm4mA) + 20
			- 16 * savedata.ppm20mA / (savedata.ppm20mA - savedata.ppm4mA))
			* (savedata.MAX - savedata.MIN) / 16 + savedata.MAX
			- 20 * (savedata.MAX - savedata.MIN) / 16;
	TempDAC = (16 * f_Temp_fixed / (savedata.temp20mA - savedata.temp4mA) + 20
			- 16 * savedata.temp20mA / (savedata.temp20mA - savedata.temp4mA))
			* (savedata.MAX - savedata.MIN) / 16 + savedata.MAX
			- 20 * (savedata.MAX - savedata.MIN) / 16;

	if (PPMDAC > savedata.MAX) {
		PPMDAC = savedata.MAX;
	}
	if (PPMDAC < savedata.MIN) {
		PPMDAC = savedata.MIN;
	}
	if (TempDAC > savedata.MAX) {
		TempDAC = savedata.MAX;
	}
	if (TempDAC < savedata.MIN) {
		TempDAC = savedata.MIN;
	}
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, PPMDAC);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, TempDAC);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

/**
 * @功能简介 : 根据传感器采集值更新继电器高低点报警、根据时间更新清洗继电器
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Relay_Update(void) {
#ifdef DEBUG
	uint8_t debugTemp9[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01, 0x02,
			0x30, 0x39, 0xFF, 0xFC, 0xFF, 0xFF };
	HAL_UART_Transmit(&huart1, debugTemp9, 14, USARTSENDTIME);
#endif
	switch (Relay_Flag) {
	case 0:
		if (savedata.lowlimitauto == 1 && Relay_Flag_Low == 1) {
			RELAY_LO(1);
		} else if (f_Rs_filter > (savedata.lowlimit + savedata.lowlimitdelay))
			RELAY_LO(0);
		RELAY_HI(0);
		break;
	case 1:
		if (f_Rs_filter > (savedata.lowlimit + savedata.lowlimitdelay))
			RELAY_LO(0);
		if (f_Rs_filter < (savedata.uplimit - savedata.uplimitdelay))
			RELAY_HI(0);
		break;
	case 2:
		RELAY_LO(0);
		if (savedata.uplimitauto == 1 && Relay_Flag_Up == 1) {
			RELAY_HI(1);
		} else if (f_Rs_filter < (savedata.uplimit - savedata.uplimitdelay))
			RELAY_HI(0);
		break;
	}
	if (Relay_WASH_Flag == 1) {
		RELAY_WASH(1);
		Relay_WASH_Flag = 2;
	} else if (Relay_WASH_Flag == 0) {
		RELAY_WASH(0);
	}
}

/**
 * 函数功能: 显示当前时间和日期
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 */
void RTC_Update(void) {
	//	uint8_t str[30]; // 字符串暂存
	//	static uint8_t FirstDisplay = 1;

	/* 获取当前时间 */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
#ifdef DEBUG
		uint8_t debugTemp10[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01, 0x02,
				0x31, 0x30, 0xFF, 0xFC, 0xFF, 0xFF };
		HAL_UART_Transmit(&huart1, debugTemp10, 14, USARTSENDTIME);
	#endif
	switch (ShowTimeNum3CMD[7]) {
	case 90:
		ShowTimeNum3CMD[7] = 105;

		/* 显示时间 */
		//	printf("%02d:%02d:%02d\n", stimestructureget.Hours,
		//			stimestructureget.Minutes, stimestructureget.Seconds);
		ShowTimeNum1CMD[7] = stimestructureget.Hours / 10;
		ShowTimeNum2CMD[7] = stimestructureget.Hours - ShowTimeNum1CMD[7] * 10
				+ 31;
		ShowTimeNum1CMD[7] += 31;

		ShowTimeNum4CMD[7] = stimestructureget.Minutes / 10;
		ShowTimeNum5CMD[7] = stimestructureget.Minutes - ShowTimeNum4CMD[7] * 10
				+ 31;
		ShowTimeNum4CMD[7] += 31;
		break;
	case 105:
		ShowTimeNum3CMD[7] = 90;
		//	if (FirstDisplay) {
		//		GetChinaCalendarStr(sdatestructureget.Year + 2000,
		//				sdatestructureget.Month, sdatestructureget.Date, str);
		//		printf("今天农历：%s\n", str);
		//
		//		if (GetJieQiStr(sdatestructureget.Year + 2000, sdatestructureget.Month,
		//				sdatestructureget.Date, str))
		//			printf("今天农历：%s\n", str);
		//
		//		FirstDisplay = 0;
		//	}
		/* 显示日期*/
		//	printf("当前时间为: %02d年(%s年)%02d月%02d日(星期%s)  ", 2000 + sdatestructureget.Year,
		//			zodiac_sign[(2000 + sdatestructureget.Year - 3) % 12],
		//			sdatestructureget.Month, sdatestructureget.Date,
		//			WEEK_STR[sdatestructureget.WeekDay]);
		/* 获取当前日期 */
		HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

		ShowDateNum3CMD[7] = sdatestructureget.Year / 10;
		ShowDateNum4CMD[7] = sdatestructureget.Year - ShowDateNum3CMD[7] * 10
				+ 31;
		ShowDateNum3CMD[7] += 31;

		ShowDateNum6CMD[7] = sdatestructureget.Month / 10;
		ShowDateNum7CMD[7] = sdatestructureget.Month - ShowDateNum6CMD[7] * 10
				+ 31;
		ShowDateNum6CMD[7] += 31;

		ShowDateNum9CMD[7] = sdatestructureget.Date / 10;
		ShowDateNum10CMD[7] = sdatestructureget.Date - ShowDateNum9CMD[7] * 10
				+ 31;
		ShowDateNum9CMD[7] += 31;
		break;
	}
	HAL_UART_Transmit(&huart1, ShowDateNum1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum3CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum4CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum5CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum6CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum7CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum8CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum9CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowDateNum10CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTimeNum1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTimeNum2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTimeNum3CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTimeNum4CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTimeNum5CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 计算时间差(秒)
 * @入口参数 : 新旧日期和时间
 * @出口参数 : 相差秒数
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
long calcTimeSpan(uint8_t datenowYear, uint8_t datenowMonth,
		uint8_t datenowDate, uint8_t timenowHours, uint8_t timenowMinutes,
		uint8_t timenowSeconds, uint8_t dateoldYear, uint8_t dateoldMonth,
		uint8_t dateoldDate, uint8_t timeoldHours, uint8_t timeoldMinutes,
		uint8_t timeoldSeconds) {
	long seconds = 0;
#ifdef DEBUG
	uint8_t debugTemp11[14] = { 0xEE, 0x20, 0x00, 0x0A, 0x00, 0x0A, 0x01, 0x02,
			0x31, 0x31, 0xFF, 0xFC, 0xFF, 0xFF };
	HAL_UART_Transmit(&huart1, debugTemp11, 14, USARTSENDTIME);
#endif
	seconds = (datenowYear - dateoldYear) * 31536000;
	seconds += (datenowMonth - dateoldMonth) * 2592000;
	seconds += (datenowDate - dateoldDate) * 86400;
	seconds += (timenowHours - timeoldHours) * 3600;
	seconds += (timenowMinutes - timeoldMinutes) * 60;
	seconds += timenowSeconds - timeoldSeconds;
	/* 防止用户时钟设置比之前小 */
	if (seconds < 0) {
		return 999999;
	} else
		return seconds;
}

/**
 * @功能简介 : 初始化UI
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void LCD_Init(void) {
	/* 根据保存的设置调整亮度 */
	ChangeBrightnessCMD[3] = savedata.brightness * 20 + 55;
	ChangeBrightnessCMD[4] = ChangeBrightnessCMD[3];
	HAL_UART_Transmit(&huart1, ChangeBrightnessCMD, 11, USARTSENDTIME);

	HAL_UART_Transmit(&huart1, SetBackWhiteCMD, 8, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ClearBackBlackCMD, 6, USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowMainNum1CMD, 13, USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowMainNum2CMD, 13, USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowMainNum3CMD, 13, USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowMainNum4CMD, 13, USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowMainNum5CMD, 13, USARTSENDTIME);
	LCD_Update();
	switch (savedata.mode) {
	case 0:
		if (f_Rs_filter < 9999) {
			ShowMainUnitCMD[7] = 27;
		} else {
			ShowMainUnitCMD[7] = 28;
		}
		break;
	case 1:
		ShowMainUnitCMD[7] = 29;
		break;
	case 2:
		ShowMainUnitCMD[7] = 84;
		break;
	}
	HAL_UART_Transmit(&huart1, ShowMainUnitCMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempNum1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempNum2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempNum3CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempNum4CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowTempUnitCMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowPageStatusCMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowElectrodeStatusCMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum3CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmANum4CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowmAUnitCMD, 13, USARTSENDTIME);

	RTC_Update();
}
/**
 * @功能简介 : 获取电导率
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2020/02/03
 */
void getRs(void) {
	float temp;
	/**
	 * 电阻率 = (电阻 * 横截面积) / 距离
	 * 电导率 = 1 / 电阻率 (µS/cm)
	 * Ω * cm = (Ω * cm2) / cm
	 * 电导率 = 1 * 距离 / (电阻 * 横截面积)
	 * µS/cm = (1000000 / Ω) / cm
	 */
	//TODO 测试电阻用
	RangeSelect(9);
	f_Rs = DA5933_Get_Rs();
	/* 刷新标志位 */
	refreshFlag = 1;
	//TODO 电导率完整计算
//	f_Rs = (CELLLENGTH * 1000000 / DA5933_Get_Rs()) / CELLAREA;
	//TODO 修改温度获取
//	memcpy((&f_Temp), &sensorRevBuf[13], 4);
	result++;
	if (result > 40) {
		result = 0;
	}
//			if (f_Rs > 20) {
//				f_Rs = 20;
//			}
//			if (f_Rs < 0) {
//				f_Rs = 0;
//			}
//	f_Rs_fixed[filterCNT] = savedata.kdo * (100 - savedata.tempfactor)
//			* (f_Rs + savedata.bdo) / 100;

	/* 换算成千欧 */
	f_Rs_fixed[filterCNT] = f_Rs / 1000;

//	if (f_Rs_fixed[filterCNT] > 200000) {
//		f_Rs_fixed[filterCNT] = 0;
//		/* 刷新标志位 */
//		refreshFlag = 0;
//	}
	if (f_Rs_fixed[filterCNT] <= 0) {
		f_Rs_fixed[filterCNT] = 0;
		/* 刷新标志位 */
		refreshFlag = 0;
	}
	filterCNT++;
	if (f_Temp > 99) {
		f_Temp = 99;
	}
	if (f_Temp < 0) {
		f_Temp = 0;
	}
	f_Temp_fixed = f_Temp * savedata.ktemp;
	if (filterCNT > savedata.filter) {
		uint8_t i;
		for (i = 0; i < savedata.filter; i++) {
			for (filterCNT = 0; filterCNT < savedata.filter - i; filterCNT++) {
				if (f_Rs_fixed[filterCNT] > f_Rs_fixed[filterCNT + 1]) {
					temp = f_Rs_fixed[filterCNT + 1];
					f_Rs_fixed[filterCNT + 1] = f_Rs_fixed[filterCNT];
					f_Rs_fixed[filterCNT] = temp;
				}
			}
		}
		f_Rs_filter = f_Rs_fixed[savedata.filter / 2];
		filterCNT = 0;
	}
}
/**
 * @功能简介 : 进入Config页面1
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page1(void) {
	tempdata.temp = f_Temp_fixed;
	HAL_UART_Transmit(&huart1, ShowConfPage1CMD, 13, USARTSENDTIME);
	Change_Conf_Unit(tempdata.mode);
//	Change_Conf_Salinity(tempdata.salinity);
	Change_Conf_Temp(tempdata.temp);
	HAL_UART_Transmit(&huart1, ShowConfTempUnitCMD, 13, USARTSENDTIME);
	Change_Conf_Temp_Now(f_Temp);
	HAL_UART_Transmit(&huart1, ShowConfTempNowUnitCMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}
/**
 * @功能简介 : 进入Config页面1.2
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page1_2(void) {
	HAL_UART_Transmit(&huart1, ShowConfPage1_2CMD, 13, USARTSENDTIME);
	Change_Conf_TempFactorType(tempdata.tempfactortype);
	Change_Conf_TempFactor(tempdata.tempfactor);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}
/**
 * @功能简介 : 进入Config页面2
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page2(void) {
	HAL_UART_Transmit(&huart1, ShowConfigPage2CMD, 13, USARTSENDTIME);
	Change_Conf_PPM4mA(tempdata.ppm4mA);
	Change_Conf_ppm20mA(tempdata.ppm20mA);
	Change_Conf_temp4mA(tempdata.temp4mA);
	Change_Conf_temp20mA(tempdata.temp20mA);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 进入Config页面3
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page3(void) {
	HAL_UART_Transmit(&huart1, ShowConfigPage3CMD, 13, USARTSENDTIME);
	Change_Conf_UpLimitAuto(tempdata.uplimitauto);
	Change_Conf_UpLimit(tempdata.uplimit);
	Change_Conf_UpLimitDelay(tempdata.uplimitdelay);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);

}

/**
 * @功能简介 : 进入Config页面4
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page4(void) {
	HAL_UART_Transmit(&huart1, ShowConfigPage4CMD, 13, USARTSENDTIME);
	Change_Conf_LowLimitAuto(tempdata.lowlimitauto);
	Change_Conf_LowLimit(tempdata.lowlimit);
	Change_Conf_LowLimitDelay(tempdata.lowlimitdelay);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 进入Config页面5
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page5(void) {
	HAL_UART_Transmit(&huart1, ShowConfigPage5CMD, 13, USARTSENDTIME);
	Change_Conf_WashHoldTime(tempdata.washholdtime);
	Change_Conf_WashDelayTime(tempdata.washdelaytime);
	Change_Conf_Filter(tempdata.filter);
	Change_Conf_Brightness(tempdata.brightness);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 进入Config页面6
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page6(void) {
	HAL_UART_Transmit(&huart1, ShowConfigPage6CMD, 13, USARTSENDTIME);
	Change_Conf_ADMAX((tempdata.MAX - ADMAX) / 10);
	Change_Conf_ADMIN((tempdata.MIN - ADMIN) / 10);
	Change_Conf_Interval(tempdata.interval);
	Change_Conf_PassW(tempdata.password);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 进入Config页面7
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Conf_Page7(void) {
	HAL_UART_Transmit(&huart1, ShowConfigPage7CMD, 13, USARTSENDTIME);
	Change_Conf_Year(sdateconfstructureget.Year);
	Change_Conf_Month(sdateconfstructureget.Month);
	Change_Conf_Date(sdateconfstructureget.Date);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 进入Config页面8
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2020/01/03
 */
void Enter_Conf_Page8(void) {
	HAL_UART_Transmit(&huart1, ShowConfigPage8CMD, 13, USARTSENDTIME);
	Change_Conf_Hours(stimeconfstructureget.Hours);
	Change_Conf_Minutes(stimeconfstructureget.Minutes);
	Change_Conf_Seconds(stimeconfstructureget.Seconds);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}
/**
 * @功能简介 : 进入密码页面
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2020/01/03
 */
uint8_t Enter_PasW_Page(uint8_t lastPage) {
	uint8_t a = 0, b = 0, c = 0, select = 0;
	uint8_t ShowA[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 114, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t ShowB[13] = { 0xEE, 0x32, 0x01, 44, 0x00, 114, 0x00, 114, 0x00,
			0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowC[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00, 114, 0x00,
			0xFF, 0xFC, 0xFF, 0xFF };
	//显示密码界面选择条
	uint8_t ShowPasWSelect1CMD[13] = { 0xEE, 0x32, 0x01, 15, 0x00, 109, 0x00,
			115, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowPasWSelect2CMD[13] = { 0xEE, 0x32, 0x01, 15, 0x00, 143, 0x00,
			115, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowPasWUnselect1CMD[13] = { 0xEE, 0x32, 0x01, 15, 0x00, 109, 0x00,
			116, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowPasWUnselect2CMD[13] = { 0xEE, 0x32, 0x01, 15, 0x00, 143, 0x00,
			116, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };

	HAL_UART_Transmit(&huart1, ShowPassWPageCMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowA, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowB, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowC, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowPasWSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowPasWSelect2CMD, 13, USARTSENDTIME);
	//屏蔽刚进入设置按键未释放
	while (!BTN_MODE())
		;
	while (!BTN_CONFIG())
		;
	while (!BTN_CAL())
		;
	while (!BTN_RIGHT())
		;
	while (!BTN_ENTER())
		;
	HAL_Delay(1000);
	while (1) {
		if (!BTN_MODE()) {
			switch (select) {
			case 0:
				a++;
				if (a == 10)
					a = 0;
				ShowA[7] = 31 + a;
				HAL_UART_Transmit(&huart1, ShowA, 13, USARTSENDTIME);
				break;
			case 1:
				b++;
				if (b == 10)
					b = 0;
				ShowB[7] = 31 + b;
				HAL_UART_Transmit(&huart1, ShowB, 13, USARTSENDTIME);
				break;
			case 2:
				c++;
				if (c == 10)
					c = 0;
				ShowC[7] = 31 + c;
				HAL_UART_Transmit(&huart1, ShowC, 13, USARTSENDTIME);
				break;
			default:
				break;
			}
		}
		if (!BTN_RIGHT()) {
			switch (select) {
			case 0:
				a--;
				if (a > 10)
					a = 9;
				ShowA[7] = 31 + a;
				HAL_UART_Transmit(&huart1, ShowA, 13, USARTSENDTIME);
				break;
			case 1:
				b--;
				if (b > 10)
					b = 9;
				ShowB[7] = 31 + b;
				HAL_UART_Transmit(&huart1, ShowB, 13, USARTSENDTIME);
				break;
			case 2:
				c--;
				if (c > 10)
					c = 9;
				ShowC[7] = 31 + c;
				HAL_UART_Transmit(&huart1, ShowC, 13, USARTSENDTIME);
				break;
			default:
				break;
			}
		}
		if (!BTN_CAL()) {
			while (!BTN_CAL())
				;
			if (lastPage == 1) {
				Enter_Conf_Page6();
				Select_Next(2);
				Select_Next(3);
				Select_Next(4);
			}
			return 0;
		}
		if (!BTN_CONFIG()) {
			while (!BTN_CONFIG())
				;
			if (lastPage == 1) {
				Enter_Conf_Page6();
				Select_Next(2);
				Select_Next(3);
				Select_Next(4);
			}
			return 0;
		}
		if (!BTN_ENTER()) {
			while (!BTN_ENTER())
				;
			select++;
			switch (select) {
			case 0:
				ShowA[7] = 31 + a;
				ShowB[7] = 114;
				ShowC[7] = 114;
				HAL_UART_Transmit(&huart1, ShowA, 13, USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowB, 13, USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowC, 13, USARTSENDTIME);
				break;
			case 1:
				ShowA[7] = 114;
				ShowB[7] = b + 31;
				ShowC[7] = 114;
				HAL_UART_Transmit(&huart1, ShowA, 13, USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowB, 13, USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowC, 13, USARTSENDTIME);

				ShowPasWSelect1CMD[3] = 45;
				ShowPasWSelect2CMD[3] = 45;
				HAL_UART_Transmit(&huart1, ShowPasWUnselect1CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowPasWUnselect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowPasWSelect1CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowPasWSelect2CMD, 13,
				USARTSENDTIME);
				break;
			case 2:
				ShowA[7] = 114;
				ShowB[7] = 114;
				ShowC[7] = c + 31;
				HAL_UART_Transmit(&huart1, ShowA, 13, USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowB, 13, USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowC, 13, USARTSENDTIME);

				ShowPasWSelect1CMD[3] = 75;
				ShowPasWSelect2CMD[3] = 75;
				ShowPasWUnselect1CMD[3] = 45;
				ShowPasWUnselect2CMD[3] = 45;
				HAL_UART_Transmit(&huart1, ShowPasWUnselect1CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowPasWUnselect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowPasWSelect1CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowPasWSelect2CMD, 13,
				USARTSENDTIME);
				break;
			default:
				break;
			}
			if (select == 3) {
				if (lastPage == 1) {
					Enter_Conf_Page6();
					Select_Next(2);
					Select_Next(3);
					Select_Next(4);
				}
				if (a == 5 && b == 3 && c == 2) {
					return 1;
				} else
					return 0;
			}
		}
		HAL_Delay(200);
	}
}
/**
 * @功能简介 : 进入校准页面1
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Cal_Page1(void) {
	HAL_UART_Transmit(&huart1, ShowCalPage1CMD, 13, USARTSENDTIME);
	tempdata.caltype = 0;
	Change_CalType(tempdata.caltype);
	Change_AirPressureUnit(tempdata.airpressureunit);
	Change_AirPressure(tempdata.cell);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 进入校准页面2
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_Cal_Page2(void) {
	HAL_UART_Transmit(&huart1, ShowCalPage2CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect1CMD, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 进入History页面1
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Enter_History_Page1(void) {
	uint16_t i = 0;
	uint16_t temp1 = 0, temp2 = 0;
	HAL_UART_Transmit(&huart1, ShowHistoryPage1CMD, 13, USARTSENDTIME);
	/* 显示网格 */
	HAL_UART_Transmit(&huart1, SetForeGrayCMD, 8, USARTSENDTIME);
	for (i = 1; i < 6; i++) {
		DrawLine(10, (10 + i * 50), 470, (10 + i * 50));
		DrawLine((15 + i * 50), 55, (15 + i * 50), 265);
	}
	for (i = 6; i < 11; i++) {
		DrawLine((15 + i * 50), 55, (15 + i * 50), 265);
	}
	if (savedata.mode == 0) {
		HAL_UART_Transmit(&huart1, Show200mScmCMD, 20, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show150mScmCMD, 20, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show100mScmCMD, 20, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show50mScmCMD, 19, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show0mScmCMD, 18, USARTSENDTIME);
	} else if (savedata.mode == 1) {
		HAL_UART_Transmit(&huart1, Show20MRCMD, 20, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show15MRCMD, 20, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show10MRCMD, 20, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show5MRCMD, 19, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show0MRCMD, 19, USARTSENDTIME);
	} else if (savedata.mode == 2) {
		HAL_UART_Transmit(&huart1, Show700pptCMD, 19, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show525pptCMD, 19, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show350pptCMD, 19, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show175pptCMD, 19, USARTSENDTIME);
		HAL_UART_Transmit(&huart1, Show0pptCMD, 19, USARTSENDTIME);
	}
	/* 显示报警线 */
	HAL_UART_Transmit(&huart1, SetForeRedCMD, 8, USARTSENDTIME);
	DrawLine(10, (260 - savedata.lowlimit * 10), 470,
			(260 - savedata.lowlimit * 10));
	DrawLine(10, (260 - savedata.uplimit * 10), 470,
			(260 - savedata.uplimit * 10));
	/* 显示最近溶解氧 */
	HAL_UART_Transmit(&huart1, SetForeBlackCMD, 8, USARTSENDTIME);
	ShowHistoryTime1CMD[10] = History_DATE[historyStart].Year / 10;
	ShowHistoryTime1CMD[11] = History_DATE[historyStart].Year
			- ShowHistoryTime1CMD[10] * 10 + 0x30;
	ShowHistoryTime1CMD[10] += 0x30;

	ShowHistoryTime1CMD[13] = History_DATE[historyStart].Month / 10;
	ShowHistoryTime1CMD[14] = History_DATE[historyStart].Month
			- ShowHistoryTime1CMD[13] * 10 + 0x30;
	ShowHistoryTime1CMD[13] += 0x30;

	ShowHistoryTime1CMD[16] = History_DATE[historyStart].Date / 10;
	ShowHistoryTime1CMD[17] = History_DATE[historyStart].Date
			- ShowHistoryTime1CMD[16] * 10 + 0x30;
	ShowHistoryTime1CMD[16] += 0x30;

	ShowHistoryTime1CMD[19] = History_TIME[historyStart].Hours / 10;
	ShowHistoryTime1CMD[20] = History_TIME[historyStart].Hours
			- ShowHistoryTime1CMD[19] * 10 + 0x30;
	ShowHistoryTime1CMD[19] += 0x30;

	ShowHistoryTime1CMD[22] = History_TIME[historyStart].Minutes / 10;
	ShowHistoryTime1CMD[23] = History_TIME[historyStart].Minutes
			- ShowHistoryTime1CMD[22] * 10 + 0x30;
	ShowHistoryTime1CMD[22] += 0x30;

	HAL_UART_Transmit(&huart1, ShowHistoryTime1CMD, 31, USARTSENDTIME);
	ShowHistoryTime2CMD[10] = History_DATE[historyEnd].Year / 10;
	ShowHistoryTime2CMD[11] = History_DATE[historyEnd].Year
			- ShowHistoryTime2CMD[10] * 10 + 0x30;
	ShowHistoryTime2CMD[10] += 0x30;

	ShowHistoryTime2CMD[13] = History_DATE[historyEnd].Month / 10;
	ShowHistoryTime2CMD[14] = History_DATE[historyEnd].Month
			- ShowHistoryTime2CMD[13] * 10 + 0x30;
	ShowHistoryTime2CMD[13] += 0x30;

	ShowHistoryTime2CMD[16] = History_DATE[historyEnd].Date / 10;
	ShowHistoryTime2CMD[17] = History_DATE[historyEnd].Date
			- ShowHistoryTime2CMD[16] * 10 + 0x30;
	ShowHistoryTime2CMD[16] += 0x30;

	ShowHistoryTime2CMD[19] = History_TIME[historyEnd].Hours / 10;
	ShowHistoryTime2CMD[20] = History_TIME[historyEnd].Hours
			- ShowHistoryTime2CMD[19] * 10 + 0x30;
	ShowHistoryTime2CMD[19] += 0x30;

	ShowHistoryTime2CMD[22] = History_TIME[historyEnd].Minutes / 10;
	ShowHistoryTime2CMD[23] = History_TIME[historyEnd].Minutes
			- ShowHistoryTime2CMD[22] * 10 + 0x30;
	ShowHistoryTime2CMD[22] += 0x30;

	HAL_UART_Transmit(&huart1, ShowHistoryTime2CMD, 28, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, SetForeBlueCMD, 8, USARTSENDTIME);
	for (i = 0; i < historyCNT; i++) {
		temp1 = 399 + historyEnd - i;
		if (temp1 > 399) {
			temp1 -= 399;
		}
		temp2 = 399 + historyEnd - i - 1;
		if (temp2 > 399) {
			temp2 -= 399;
		}
		if (historyCNT > 1) {
			DrawLine((465 - 399 * i / historyCNT),
					(260 - History_PPM[temp1] * 10),
					(465 - 399 * (i + 1) / historyCNT),
					(260 - History_PPM[temp2] * 10));
		}
	}
}

/**
 * @功能简介 : 修改设置界面的单位设置
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Unit(uint8_t mode) {
	switch (mode) {
	case 0:
		ShowConfModeCMD[7] = 109;
		break;
	case 1:
		ShowConfModeCMD[7] = 108;
		break;
	case 2:
		ShowConfModeCMD[7] = 110;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowConfModeCMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的盐度补偿系数
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
//void Change_Conf_Salinity(float salinity) {
//	ShowConfSalinityNum1CMD[7] = salinity / 10;
//	ShowConfSalinityNum2CMD[7] = salinity - ShowConfSalinityNum1CMD[7] * 10;
//	ShowConfSalinityNum4CMD[7] = 31
//			+ (salinity - ShowConfSalinityNum1CMD[7] * 10
//					- ShowConfSalinityNum2CMD[7]) * 10;
//	ShowConfSalinityNum2CMD[7] += 31;
//	if (ShowConfSalinityNum1CMD[7] != 0) {
//		ShowConfSalinityNum1CMD[7] += 31;
//	} else
//		ShowConfSalinityNum1CMD[7] = 91;
//	HAL_UART_Transmit(&huart1, ShowConfSalinityNum1CMD,
//			sizeof(ShowConfSalinityNum1CMD), USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowConfSalinityNum2CMD,
//			sizeof(ShowConfSalinityNum2CMD), USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowConfSalinityNum3CMD,
//			sizeof(ShowConfSalinityNum3CMD), USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowConfSalinityNum4CMD,
//			sizeof(ShowConfSalinityNum4CMD), USARTSENDTIME);
//	HAL_UART_Transmit(&huart1, ShowConfSalinityUnitCMD,
//			sizeof(ShowConfSalinityUnitCMD), USARTSENDTIME);
//}
/**
 * @功能简介 : 修改设置界面的温度补偿系数
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Temp(float temp) {
	ShowConfTempNum1CMD[7] = temp / 10;
	ShowConfTempNum2CMD[7] = temp - ShowConfTempNum1CMD[7] * 10;
	ShowConfTempNum4CMD[7] = 31
			+ (temp - ShowConfTempNum1CMD[7] * 10 - ShowConfTempNum2CMD[7])
					* 10;
	ShowConfTempNum2CMD[7] += 31;
	if (ShowConfTempNum1CMD[7] != 0) {
		ShowConfTempNum1CMD[7] += 31;
	} else
		ShowConfTempNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConfTempNum1CMD, sizeof(ShowConfTempNum1CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfTempNum2CMD, sizeof(ShowConfTempNum2CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfTempNum3CMD, sizeof(ShowConfTempNum3CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfTempNum4CMD, sizeof(ShowConfTempNum4CMD),
	USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的当前温度
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Temp_Now(float temp) {
	ShowConfTempNowNum1CMD[7] = temp / 10;
	ShowConfTempNowNum2CMD[7] = temp - ShowConfTempNowNum1CMD[7] * 10;
	ShowConfTempNowNum4CMD[7] =
			31
					+ (temp - ShowConfTempNowNum1CMD[7] * 10
							- ShowConfTempNowNum2CMD[7]) * 10;
	ShowConfTempNowNum2CMD[7] += 31;
	if (ShowConfTempNowNum1CMD[7] != 0) {
		ShowConfTempNowNum1CMD[7] += 31;
	} else
		ShowConfTempNowNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConfTempNowNum1CMD,
			sizeof(ShowConfTempNowNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfTempNowNum2CMD,
			sizeof(ShowConfTempNowNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfTempNowNum3CMD,
			sizeof(ShowConfTempNowNum3CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfTempNowNum4CMD,
			sizeof(ShowConfTempNowNum4CMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面温度补偿方式
 * @入口参数 : 温度补偿方式
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2020/01/03
 */
void Change_Conf_TempFactorType(uint8_t TempFactorType) {
	uint8_t ShowConfTempFactorTypeCMD[13] = { 0xEE, 0x32, 0x01, 46, 0x00, 62,
			0x00, 111, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	if (TempFactorType == 0) {
		ShowConfTempFactorTypeCMD[7] = 111;
	} else {
		ShowConfTempFactorTypeCMD[7] = 112;
	}
	HAL_UART_Transmit(&huart1, ShowConfTempFactorTypeCMD,
			sizeof(ShowConfTempFactorTypeCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面温度系数
 * @入口参数 : 温度系数
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2020/01/03
 */
void Change_Conf_TempFactor(float Factor) {
	ShowConf20mANum1CMD[7] = Factor / 10;
	ShowConf20mANum2CMD[7] = Factor - ShowConf20mANum1CMD[7] * 10;
	ShowConf20mANum4CMD[7] = (Factor - ShowConf20mANum1CMD[7] * 10
			- ShowConf20mANum2CMD[7]) * 10;
	ShowConf20mANum5CMD[7] = 31
			+ (Factor - ShowConf20mANum1CMD[7] * 10 - ShowConf20mANum2CMD[7]
					- ShowConf20mANum4CMD[7] * 0.1) * 100;
	ShowConf20mANum4CMD[7] += 31;
	ShowConf20mANum2CMD[7] += 31;
	if (ShowConf20mANum1CMD[7] != 0) {
		ShowConf20mANum1CMD[7] += 31;
	} else
		ShowConf20mANum1CMD[7] = 91;
	if (tempdata.mode == 2) {
		ShowConf20mANum3CMD[2] = 0x01;
		ShowConf20mANum3CMD[3] = 64;
		ShowConf20mANum4CMD[2] = 0x01;
		ShowConf20mANum4CMD[3] = 54;
	} else {
		ShowConf20mANum3CMD[2] = 0x01;
		ShowConf20mANum3CMD[3] = 54;
		ShowConf20mANum4CMD[2] = 0x01;
		ShowConf20mANum4CMD[3] = 64;
	}
	HAL_UART_Transmit(&huart1, ShowConf20mANum1CMD, sizeof(ShowConf20mANum1CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum2CMD, sizeof(ShowConf20mANum2CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum3CMD, sizeof(ShowConf20mANum3CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum4CMD, sizeof(ShowConf20mANum4CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum5CMD, sizeof(ShowConf20mANum5CMD),
	USARTSENDTIME);

	ShowConf20mAUnitCMD[7] = 23;

	HAL_UART_Transmit(&huart1, ShowConf20mAUnitCMD, sizeof(ShowConf20mAUnitCMD),
	USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的4mA对应溶氧值
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_PPM4mA(float ppm4mA) {
	ppm4mA = ppm4mA + 0.005;
	ShowConf4mANum1CMD[7] = ppm4mA / 10;
	ShowConf4mANum2CMD[7] = ppm4mA - ShowConf4mANum1CMD[7] * 10;
	ShowConf4mANum4CMD[7] = (ppm4mA - ShowConf4mANum1CMD[7] * 10
			- ShowConf4mANum2CMD[7]) * 10;
	ShowConf4mANum5CMD[7] = 31
			+ (ppm4mA - ShowConf4mANum1CMD[7] * 10 - ShowConf4mANum2CMD[7]
					- ShowConf4mANum4CMD[7] * 0.1) * 100;
	ShowConf4mANum4CMD[7] += 31;
	ShowConf4mANum2CMD[7] += 31;
	if (ShowConf4mANum1CMD[7] != 0) {
		ShowConf4mANum1CMD[7] += 31;
	} else
		ShowConf4mANum1CMD[7] = 91;
	if (tempdata.mode == 2) {
		ShowConf4mANum3CMD[2] = 0x01;
		ShowConf4mANum3CMD[3] = 64;
		ShowConf4mANum4CMD[2] = 0x01;
		ShowConf4mANum4CMD[3] = 54;
	} else {
		ShowConf4mANum3CMD[2] = 0x01;
		ShowConf4mANum3CMD[3] = 54;
		ShowConf4mANum4CMD[2] = 0x01;
		ShowConf4mANum4CMD[3] = 64;
	}
	HAL_UART_Transmit(&huart1, ShowConf4mANum1CMD, sizeof(ShowConf4mANum1CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mANum2CMD, sizeof(ShowConf4mANum2CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mANum3CMD, sizeof(ShowConf4mANum3CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mANum4CMD, sizeof(ShowConf4mANum4CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mANum5CMD, sizeof(ShowConf4mANum5CMD),
	USARTSENDTIME);
	switch (tempdata.mode) {
	case 0:
		ShowConf4mAUnitCMD[7] = 27;
		break;
	case 1:
		ShowConf4mAUnitCMD[7] = 28;
		break;
	case 2:
		ShowConf4mAUnitCMD[7] = 23;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowConf4mAUnitCMD, sizeof(ShowConf4mAUnitCMD),
	USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的20mA对应溶氧值
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_ppm20mA(float ppm20mA) {
	ppm20mA = ppm20mA + 0.005;
	ShowConf20mANum1CMD[7] = ppm20mA / 10;
	ShowConf20mANum2CMD[7] = ppm20mA - ShowConf20mANum1CMD[7] * 10;
	ShowConf20mANum4CMD[7] = (ppm20mA - ShowConf20mANum1CMD[7] * 10
			- ShowConf20mANum2CMD[7]) * 10;
	ShowConf20mANum5CMD[7] = 31
			+ (ppm20mA - ShowConf20mANum1CMD[7] * 10 - ShowConf20mANum2CMD[7]
					- ShowConf20mANum4CMD[7] * 0.1) * 100;
	ShowConf20mANum4CMD[7] += 31;
	ShowConf20mANum2CMD[7] += 31;
	if (ShowConf20mANum1CMD[7] != 0) {
		ShowConf20mANum1CMD[7] += 31;
	} else
		ShowConf20mANum1CMD[7] = 91;
	if (tempdata.mode == 2) {
		ShowConf20mANum3CMD[2] = 0x01;
		ShowConf20mANum3CMD[3] = 64;
		ShowConf20mANum4CMD[2] = 0x01;
		ShowConf20mANum4CMD[3] = 54;
	} else {
		ShowConf20mANum3CMD[2] = 0x01;
		ShowConf20mANum3CMD[3] = 54;
		ShowConf20mANum4CMD[2] = 0x01;
		ShowConf20mANum4CMD[3] = 64;
	}
	HAL_UART_Transmit(&huart1, ShowConf20mANum1CMD, sizeof(ShowConf20mANum1CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum2CMD, sizeof(ShowConf20mANum2CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum3CMD, sizeof(ShowConf20mANum3CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum4CMD, sizeof(ShowConf20mANum4CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mANum5CMD, sizeof(ShowConf20mANum5CMD),
	USARTSENDTIME);
	switch (tempdata.mode) {
	case 0:
		ShowConf20mAUnitCMD[7] = 27;
		break;
	case 1:
		ShowConf20mAUnitCMD[7] = 28;
		break;
	case 2:
		ShowConf20mAUnitCMD[7] = 23;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowConf20mAUnitCMD, sizeof(ShowConf20mAUnitCMD),
	USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的4mA对应温度值
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_temp4mA(float temp4mA) {
	ShowConf4mATempNum1CMD[7] = temp4mA / 10;
	ShowConf4mATempNum2CMD[7] = temp4mA - ShowConf4mATempNum1CMD[7] * 10;
	ShowConf4mATempNum4CMD[7] = 31
			+ (temp4mA - ShowConf4mATempNum1CMD[7] * 10
					- ShowConf4mATempNum2CMD[7]) * 10;
	ShowConf4mATempNum2CMD[7] += 31;
	if (ShowConf4mATempNum1CMD[7] != 0) {
		ShowConf4mATempNum1CMD[7] += 31;
	} else
		ShowConf4mATempNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum1CMD,
			sizeof(ShowConf4mATempNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum2CMD,
			sizeof(ShowConf4mATempNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum3CMD,
			sizeof(ShowConf4mATempNum3CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum4CMD,
			sizeof(ShowConf4mATempNum4CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempUnitCMD,
			sizeof(ShowConf4mATempUnitCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的20mA对应温度值
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_temp20mA(float temp20mA) {
	ShowConf20mATempNum1CMD[7] = temp20mA / 10;
	ShowConf20mATempNum2CMD[7] = temp20mA - ShowConf20mATempNum1CMD[7] * 10;
	ShowConf20mATempNum4CMD[7] = 31
			+ (temp20mA - ShowConf20mATempNum1CMD[7] * 10
					- ShowConf20mATempNum2CMD[7]) * 10;
	ShowConf20mATempNum2CMD[7] += 31;
	if (ShowConf20mATempNum1CMD[7] != 0) {
		ShowConf20mATempNum1CMD[7] += 31;
	} else
		ShowConf20mATempNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConf20mATempNum1CMD,
			sizeof(ShowConf20mATempNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mATempNum2CMD,
			sizeof(ShowConf20mATempNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mATempNum3CMD,
			sizeof(ShowConf20mATempNum3CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mATempNum4CMD,
			sizeof(ShowConf20mATempNum4CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf20mATempUnitCMD,
			sizeof(ShowConf20mATempUnitCMD), USARTSENDTIME);
}
/**
 * @功能简介 : 修改设置界面的20mA修正值
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_ADMAX(float temp20mA) {
	/* EE 20 01 18 00 F0 00 02 2D FF FC FF FF  */
	uint8_t CMD[14] = { 0xEE, 0x20, 0x01, 24, 0x00, 114, 0x01, 0x02, 0x2D, 0x20,
			0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ADMAX20mANum1CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 114, 0x00, 31,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ADMAX20mANum2CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00, 31,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ADMAX20mANum3CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00, 30,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ADMAX20mANum4CMD[13] = { 0xEE, 0x32, 0x01, 84, 0x00, 114, 0x00, 31,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	//显示单位
	uint8_t UnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 110, 0x00, 23, 0x00,
			0xFF, 0xFC, 0xFF, 0xFF };
	HAL_UART_Transmit(&huart1, SetBackWhiteCMD, 8, USARTSENDTIME);
	if (temp20mA < 0) {
		temp20mA = -temp20mA;
		CMD[8] = 0x2D;
	} else {
		CMD[8] = 0x20;
	}
	/* +0.05进位，保证9.9正常显示 */
	temp20mA = temp20mA + 0.05;
	ADMAX20mANum1CMD[7] = temp20mA / 10;
	ADMAX20mANum2CMD[7] = temp20mA - ADMAX20mANum1CMD[7] * 10;
	ADMAX20mANum4CMD[7] = 31
			+ (temp20mA - ADMAX20mANum1CMD[7] * 10 - ADMAX20mANum2CMD[7]) * 10;
	ADMAX20mANum2CMD[7] += 31;
	if (ADMAX20mANum1CMD[7] != 0) {
		ADMAX20mANum1CMD[7] += 31;
		CMD[2] = 0x01;
		CMD[3] = 24;
		HAL_UART_Transmit(&huart1, ADMAX20mANum1CMD, sizeof(ADMAX20mANum1CMD),
		USARTSENDTIME);
	} else {
		CMD[2] = 0x01;
		CMD[3] = 44;
		ADMAX20mANum1CMD[7] = 91;
	}
	HAL_UART_Transmit(&huart1, CMD, sizeof(CMD), USARTSENDTIME);

	HAL_UART_Transmit(&huart1, ADMAX20mANum2CMD, sizeof(ADMAX20mANum2CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ADMAX20mANum3CMD, sizeof(ADMAX20mANum3CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ADMAX20mANum4CMD, sizeof(ADMAX20mANum4CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, UnitCMD, sizeof(UnitCMD), USARTSENDTIME);
}
/**
 * @功能简介 : 修改设置界面的4mA修正值
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_ADMIN(float temp4mA) {
	/* EE 20 01 18 00 F0 00 02 2D FF FC FF FF  */
	uint8_t CMD[14] = { 0xEE, 0x20, 0x01, 24, 0x00, 62, 0x01, 0x02, 0x2D, 0x20,
			0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowConf4mANum1CMD[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 62, 0x00, 31,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowConf4mANum2CMD[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 62, 0x00, 31,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowConf4mANum3CMD[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 62, 0x00, 30,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	uint8_t ShowConf4mANum4CMD[13] = { 0xEE, 0x32, 0x01, 84, 0x00, 62, 0x00, 31,
			0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	//显示单位
	uint8_t UnitCMD[13] = { 0xEE, 0x32, 0x01, 118, 0x00, 58, 0x00, 23, 0x00,
			0xFF, 0xFC, 0xFF, 0xFF };
	HAL_UART_Transmit(&huart1, SetBackWhiteCMD, 8, USARTSENDTIME);
	if (temp4mA < 0) {
		temp4mA = -temp4mA;
		CMD[8] = 0x2D;
	} else {
		CMD[8] = 0x20;
	}
	/* +0.05进位，保证9.9正常显示 */
	temp4mA = temp4mA + 0.05;
	ShowConf4mANum1CMD[7] = temp4mA / 10;
	ShowConf4mANum2CMD[7] = temp4mA - ShowConf4mANum1CMD[7] * 10;
	ShowConf4mANum4CMD[7] = 31
			+ (temp4mA - ShowConf4mANum1CMD[7] * 10 - ShowConf4mANum2CMD[7])
					* 10;
	ShowConf4mANum2CMD[7] += 31;
	if (ShowConf4mANum1CMD[7] != 0) {
		ShowConf4mANum1CMD[7] += 31;
		CMD[2] = 0x01;
		CMD[3] = 24;
		HAL_UART_Transmit(&huart1, ShowConf4mANum1CMD,
				sizeof(ShowConf4mANum1CMD), USARTSENDTIME);
	} else {
		CMD[2] = 0x01;
		CMD[3] = 44;
		ShowConf4mANum1CMD[7] = 91;
	}
	HAL_UART_Transmit(&huart1, CMD, sizeof(CMD), USARTSENDTIME);

	HAL_UART_Transmit(&huart1, ShowConf4mANum2CMD, sizeof(ShowConf4mANum2CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mANum3CMD, sizeof(ShowConf4mANum3CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mANum4CMD, sizeof(ShowConf4mANum4CMD),
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, UnitCMD, sizeof(UnitCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的高点报警自动或关闭
 * @入口参数 : 设置状态
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_UpLimitAuto(uint8_t uplimitauto) {
	if (uplimitauto == 1) {
		ShowConfUpLimitAutoCMD[7] = 52;
	} else {
		ShowConfUpLimitAutoCMD[7] = 51;
	}
	HAL_UART_Transmit(&huart1, ShowConfUpLimitAutoCMD,
			sizeof(ShowConfUpLimitAutoCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的高点报警值
 * @入口参数 : 高点报警值
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_UpLimit(float uplimit) {
	ShowConfUpLimitNum1CMD[7] = uplimit / 10;
	ShowConfUpLimitNum2CMD[7] = uplimit - ShowConfUpLimitNum1CMD[7] * 10;
	ShowConfUpLimitNum4CMD[7] = (uplimit - ShowConfUpLimitNum1CMD[7] * 10
			- ShowConfUpLimitNum2CMD[7]) * 10;
	ShowConfUpLimitNum5CMD[7] = (uplimit - ShowConfUpLimitNum1CMD[7] * 10
			- ShowConfUpLimitNum2CMD[7] - ShowConfUpLimitNum4CMD[7] * 0.1) * 100
			+ 31;
	ShowConfUpLimitNum4CMD[7] += 31;
	ShowConfUpLimitNum2CMD[7] += 31;
	if (ShowConfUpLimitNum1CMD[7] != 0) {
		ShowConfUpLimitNum1CMD[7] += 31;
	} else
		ShowConfUpLimitNum1CMD[7] = 91;
	if (tempdata.mode == 2) {
		ShowConfUpLimitNum3CMD[2] = 0x01;
		ShowConfUpLimitNum3CMD[3] = 74;
		ShowConfUpLimitNum4CMD[2] = 0x01;
		ShowConfUpLimitNum4CMD[3] = 54;
	} else {
		ShowConfUpLimitNum3CMD[2] = 0x01;
		ShowConfUpLimitNum3CMD[3] = 54;
		ShowConfUpLimitNum4CMD[2] = 0x01;
		ShowConfUpLimitNum4CMD[3] = 74;
	}
	HAL_UART_Transmit(&huart1, ShowConfUpLimitNum1CMD,
			sizeof(ShowConfUpLimitNum1CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitNum2CMD,
			sizeof(ShowConfUpLimitNum2CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitNum3CMD,
			sizeof(ShowConfUpLimitNum3CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitNum4CMD,
			sizeof(ShowConfUpLimitNum4CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitNum5CMD,
			sizeof(ShowConfUpLimitNum4CMD),
			USARTSENDTIME);
	switch (tempdata.mode) {
	case 0:
		ShowConfUpLimitUnitCMD[7] = 27;
		break;
	case 1:
		ShowConfUpLimitUnitCMD[7] = 28;
		break;
	case 2:
		ShowConfUpLimitUnitCMD[7] = 23;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowConfUpLimitUnitCMD,
			sizeof(ShowConfUpLimitUnitCMD),
			USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的高点报警延迟
 * @入口参数 : 延迟时间
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_UpLimitDelay(float delaytime) {

	ShowConfUpLimitDelayNum1CMD[7] = delaytime / 10;
	ShowConfUpLimitDelayNum2CMD[7] = delaytime
			- ShowConfUpLimitDelayNum1CMD[7] * 10;
	ShowConfUpLimitDelayNum4CMD[7] = (delaytime
			- ShowConfUpLimitDelayNum1CMD[7] * 10
			- ShowConfUpLimitDelayNum2CMD[7]) * 10;
	ShowConfUpLimitDelayNum5CMD[7] = (delaytime
			- ShowConfUpLimitDelayNum1CMD[7] * 10
			- ShowConfUpLimitDelayNum2CMD[7]
			- ShowConfUpLimitDelayNum4CMD[7] * 0.1) * 100 + 31;
	ShowConfUpLimitDelayNum4CMD[7] += 31;
	ShowConfUpLimitDelayNum2CMD[7] += 31;
	if (ShowConfUpLimitDelayNum1CMD[7] != 0) {
		ShowConfUpLimitDelayNum1CMD[7] += 31;
	} else
		ShowConfUpLimitDelayNum1CMD[7] = 91;
	if (tempdata.mode == 2) {
		ShowConfUpLimitDelayNum3CMD[2] = 0x01;
		ShowConfUpLimitDelayNum3CMD[3] = 74;
		ShowConfUpLimitDelayNum4CMD[2] = 0x01;
		ShowConfUpLimitDelayNum4CMD[3] = 54;
	} else {
		ShowConfUpLimitDelayNum3CMD[2] = 0x01;
		ShowConfUpLimitDelayNum3CMD[3] = 54;
		ShowConfUpLimitDelayNum4CMD[2] = 0x01;
		ShowConfUpLimitDelayNum4CMD[3] = 74;
	}
	HAL_UART_Transmit(&huart1, ShowConfUpLimitDelayNum1CMD,
			sizeof(ShowConfUpLimitDelayNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitDelayNum2CMD,
			sizeof(ShowConfUpLimitDelayNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitDelayNum3CMD,
			sizeof(ShowConfUpLimitDelayNum3CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitDelayNum4CMD,
			sizeof(ShowConfUpLimitDelayNum4CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfUpLimitDelayNum5CMD,
			sizeof(ShowConfUpLimitDelayNum5CMD), USARTSENDTIME);
	switch (tempdata.mode) {
	case 0:
		ShowConfLimitDelayUnitCMD[7] = 27;
		break;
	case 1:
		ShowConfLimitDelayUnitCMD[7] = 28;
		break;
	case 2:
		ShowConfLimitDelayUnitCMD[7] = 23;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowConfLimitDelayUnitCMD,
			sizeof(ShowConfLimitDelayUnitCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的低点报警自动或关闭
 * @入口参数 : 设置状态
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_LowLimitAuto(uint8_t lowlimitauto) {
	if (lowlimitauto == 1) {
		ShowConfLowLimitAutoCMD[7] = 52;
	} else {
		ShowConfLowLimitAutoCMD[7] = 51;
	}
	HAL_UART_Transmit(&huart1, ShowConfLowLimitAutoCMD,
			sizeof(ShowConfLowLimitAutoCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的低点报警值
 * @入口参数 : 低点报警值
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_LowLimit(float lowlimit) {
	ShowConfLowLimitNum1CMD[7] = lowlimit / 10;
	ShowConfLowLimitNum2CMD[7] = lowlimit - ShowConfLowLimitNum1CMD[7] * 10;
	ShowConfLowLimitNum4CMD[7] = (lowlimit - ShowConfLowLimitNum1CMD[7] * 10
			- ShowConfLowLimitNum2CMD[7]) * 10;
	ShowConfLowLimitNum5CMD[7] = (lowlimit - ShowConfLowLimitNum1CMD[7] * 10
			- ShowConfLowLimitNum2CMD[7] - ShowConfLowLimitNum4CMD[7] * 0.1)
			* 100 + 31;
	ShowConfLowLimitNum4CMD[7] += 31;
	ShowConfLowLimitNum2CMD[7] += 31;
	if (ShowConfLowLimitNum1CMD[7] != 0) {
		ShowConfLowLimitNum1CMD[7] += 31;
	} else
		ShowConfLowLimitNum1CMD[7] = 91;
	if (tempdata.mode == 2) {
		ShowConfLowLimitNum3CMD[2] = 0x01;
		ShowConfLowLimitNum3CMD[3] = 74;
		ShowConfLowLimitNum4CMD[2] = 0x01;
		ShowConfLowLimitNum4CMD[3] = 54;
	} else {
		ShowConfLowLimitNum3CMD[2] = 0x01;
		ShowConfLowLimitNum3CMD[3] = 54;
		ShowConfLowLimitNum4CMD[2] = 0x01;
		ShowConfLowLimitNum4CMD[3] = 74;
	}
	HAL_UART_Transmit(&huart1, ShowConfLowLimitNum1CMD,
			sizeof(ShowConfLowLimitNum1CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitNum2CMD,
			sizeof(ShowConfLowLimitNum2CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitNum3CMD,
			sizeof(ShowConfLowLimitNum3CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitNum4CMD,
			sizeof(ShowConfLowLimitNum4CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitNum5CMD,
			sizeof(ShowConfLowLimitNum5CMD),
			USARTSENDTIME);
	switch (tempdata.mode) {
	case 0:
		ShowConfLowLimitUnitCMD[7] = 27;
		break;
	case 1:
		ShowConfLowLimitUnitCMD[7] = 28;
		break;
	case 2:
		ShowConfLowLimitUnitCMD[7] = 23;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowConfLowLimitUnitCMD,
			sizeof(ShowConfLowLimitUnitCMD),
			USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的低点报警延迟
 * @入口参数 : 延迟时间
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_LowLimitDelay(float delaytime) {

	ShowConfLowLimitDelayNum1CMD[7] = delaytime / 10;
	ShowConfLowLimitDelayNum2CMD[7] = delaytime
			- ShowConfLowLimitDelayNum1CMD[7] * 10;
	ShowConfLowLimitDelayNum4CMD[7] = (delaytime
			- ShowConfLowLimitDelayNum1CMD[7] * 10
			- ShowConfLowLimitDelayNum2CMD[7]) * 10;
	ShowConfLowLimitDelayNum5CMD[7] = (delaytime
			- ShowConfLowLimitDelayNum1CMD[7] * 10
			- ShowConfLowLimitDelayNum2CMD[7]
			- ShowConfLowLimitDelayNum4CMD[7] * 0.1) * 100 + 31;
	ShowConfLowLimitDelayNum4CMD[7] += 31;
	ShowConfLowLimitDelayNum2CMD[7] += 31;
	if (ShowConfLowLimitDelayNum1CMD[7] != 0) {
		ShowConfLowLimitDelayNum1CMD[7] += 31;
	} else
		ShowConfLowLimitDelayNum1CMD[7] = 91;
	if (tempdata.mode == 2) {
		ShowConfLowLimitDelayNum3CMD[2] = 0x01;
		ShowConfLowLimitDelayNum3CMD[3] = 74;
		ShowConfLowLimitDelayNum4CMD[2] = 0x01;
		ShowConfLowLimitDelayNum4CMD[3] = 54;
	} else {
		ShowConfLowLimitDelayNum3CMD[2] = 0x01;
		ShowConfLowLimitDelayNum3CMD[3] = 54;
		ShowConfLowLimitDelayNum4CMD[2] = 0x01;
		ShowConfLowLimitDelayNum4CMD[3] = 74;
	}
	HAL_UART_Transmit(&huart1, ShowConfLowLimitDelayNum1CMD,
			sizeof(ShowConfLowLimitDelayNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitDelayNum2CMD,
			sizeof(ShowConfLowLimitDelayNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitDelayNum3CMD,
			sizeof(ShowConfLowLimitDelayNum3CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitDelayNum4CMD,
			sizeof(ShowConfLowLimitDelayNum4CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfLowLimitDelayNum5CMD,
			sizeof(ShowConfLowLimitDelayNum5CMD), USARTSENDTIME);
	switch (tempdata.mode) {
	case 0:
		ShowConfLimitDelayUnitCMD[7] = 27;
		break;
	case 1:
		ShowConfLimitDelayUnitCMD[7] = 28;
		break;
	case 2:
		ShowConfLimitDelayUnitCMD[7] = 23;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowConfLimitDelayUnitCMD,
			sizeof(ShowConfLimitDelayUnitCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的清洗持续时间
 * @入口参数 : 清洗持续时间
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_WashHoldTime(uint8_t washholdtime) {
	ShowConfWashHoldTimeNum1CMD[7] = washholdtime / 10;
	ShowConfWashHoldTimeNum2CMD[7] = washholdtime
			- ShowConfWashHoldTimeNum1CMD[7] * 10;
	ShowConfWashHoldTimeNum2CMD[7] += 31;
	if (ShowConfWashHoldTimeNum1CMD[7] != 0) {
		ShowConfWashHoldTimeNum1CMD[7] += 31;
	} else
		ShowConfWashHoldTimeNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConfWashHoldTimeNum1CMD,
			sizeof(ShowConfWashHoldTimeNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfWashHoldTimeNum2CMD,
			sizeof(ShowConfWashHoldTimeNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfWashHoldTimeUnitCMD,
			sizeof(ShowConfWashHoldTimeUnitCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的清洗时间间隔
 * @入口参数 : 清洗时间间隔
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_WashDelayTime(uint8_t washdelaytime) {
	ShowConfWashDelayTimeNum1CMD[7] = washdelaytime / 10;
	ShowConfWashDelayTimeNum2CMD[7] = washdelaytime
			- ShowConfWashDelayTimeNum1CMD[7] * 10;
	ShowConfWashDelayTimeNum2CMD[7] += 31;
	if (ShowConfWashDelayTimeNum1CMD[7] != 0) {
		ShowConfWashDelayTimeNum1CMD[7] += 31;
	} else
		ShowConfWashDelayTimeNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConfWashDelayTimeNum1CMD,
			sizeof(ShowConfWashDelayTimeNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfWashDelayTimeNum2CMD,
			sizeof(ShowConfWashDelayTimeNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfWashDelayTimeUnitCMD,
			sizeof(ShowConfWashDelayTimeUnitCMD), USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的滤波系数
 * @入口参数 : 滤波系数
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Filter(uint8_t filter) {
	ShowConfFilterNum1CMD[7] = filter / 10;
	ShowConfFilterNum2CMD[7] = filter - ShowConfFilterNum1CMD[7] * 10;
	ShowConfFilterNum2CMD[7] += 31;
	if (ShowConfFilterNum1CMD[7] != 0) {
		ShowConfFilterNum1CMD[7] += 31;
	} else
		ShowConfFilterNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConfFilterNum1CMD,
			sizeof(ShowConfFilterNum1CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfFilterNum2CMD,
			sizeof(ShowConfFilterNum2CMD),
			USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfFilterUnitCMD,
			sizeof(ShowConfFilterUnitCMD),
			USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的显示亮度
 * @入口参数 : 显示亮度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Brightness(uint8_t brightness) {
	ShowConfBrightnessNum1CMD[7] = brightness / 10;
	ShowConfBrightnessNum2CMD[7] = brightness
			- ShowConfBrightnessNum1CMD[7] * 10;
	ShowConfBrightnessNum2CMD[7] += 31;
	ShowConfBrightnessNum1CMD[7] += 31;
	ChangeBrightnessCMD[3] = brightness * 20 + 55;
	ChangeBrightnessCMD[4] = ChangeBrightnessCMD[3];
	HAL_UART_Transmit(&huart1, ShowConfBrightnessNum1CMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConfBrightnessNum2CMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ChangeBrightnessCMD, 11, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的年
 * @入口参数 : 年
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Year(uint8_t year) {
	uint8_t CMD1[13] = { 0xEE, 0x32, 0x01, 14, 0x00, 62, 0x00, 33, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD2[13] = { 0xEE, 0x32, 0x01, 34, 0x00, 62, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD3[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 62, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD4[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 62, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD5[13] = { 0xEE, 0x32, 0x01, 114, 0x00, 62, 0x00, 102, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	CMD3[7] = year / 10;
	CMD4[7] = year - CMD3[7] * 10;
	CMD4[7] += 31;
	CMD3[7] += 31;
	HAL_UART_Transmit(&huart1, CMD1, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD2, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD3, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD4, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD5, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的月
 * @入口参数 : 月
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Month(uint8_t month) {
	uint8_t CMD3[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD4[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD5[13] = { 0xEE, 0x32, 0x01, 114, 0x00, 114, 0x00, 101, 0x00,
			0xFF, 0xFC, 0xFF, 0xFF };
	CMD3[7] = month / 10;
	CMD4[7] = month - CMD3[7] * 10;
	CMD4[7] += 31;
	CMD3[7] += 31;
	HAL_UART_Transmit(&huart1, CMD3, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD4, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD5, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的日
 * @入口参数 : 日
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Date(uint8_t date) {
	uint8_t CMD3[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 164, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD4[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 164, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD5[13] = { 0xEE, 0x32, 0x01, 114, 0x00, 164, 0x00, 100, 0x00,
			0xFF, 0xFC, 0xFF, 0xFF };
	CMD3[7] = date / 10;
	CMD4[7] = date - CMD3[7] * 10;
	CMD4[7] += 31;
	CMD3[7] += 31;
	HAL_UART_Transmit(&huart1, CMD3, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD4, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD5, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的时
 * @入口参数 : 时
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Hours(uint8_t hours) {
	uint8_t CMD3[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 62, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD4[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 62, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD5[13] = { 0xEE, 0x32, 0x01, 114, 0x00, 62, 0x00, 99, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	CMD3[7] = hours / 10;
	CMD4[7] = hours - CMD3[7] * 10;
	CMD4[7] += 31;
	CMD3[7] += 31;
	HAL_UART_Transmit(&huart1, CMD3, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD4, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD5, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的分
 * @入口参数 : 分
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Minutes(uint8_t minutes) {
	uint8_t CMD3[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 114, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD4[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 114, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD5[13] = { 0xEE, 0x32, 0x01, 114, 0x00, 114, 0x00, 83, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	CMD3[7] = minutes / 10;
	CMD4[7] = minutes - CMD3[7] * 10;
	CMD4[7] += 31;
	CMD3[7] += 31;
	HAL_UART_Transmit(&huart1, CMD3, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD4, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD5, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的秒
 * @入口参数 : 秒
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Seconds(uint8_t seconds) {
	uint8_t CMD3[13] = { 0xEE, 0x32, 0x01, 54, 0x00, 164, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD4[13] = { 0xEE, 0x32, 0x01, 74, 0x00, 164, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD5[13] = { 0xEE, 0x32, 0x01, 114, 0x00, 164, 0x00, 61, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	CMD3[7] = seconds / 10;
	CMD4[7] = seconds - CMD3[7] * 10;
	CMD4[7] += 31;
	CMD3[7] += 31;
	HAL_UART_Transmit(&huart1, CMD3, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD4, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD5, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改设置界面的历史记录间隔
 * @入口参数 : 时间间隔
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_Conf_Interval(uint8_t interval) {
	uint8_t CMD3[13] = { 0xEE, 0x32, 0x01, 24, 0x00, 164, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD4[13] = { 0xEE, 0x32, 0x01, 44, 0x00, 164, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	uint8_t CMD5[13] = { 0xEE, 0x32, 0x01, 80, 0x00, 160, 0x00, 85, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	CMD3[7] = interval / 10;
	CMD4[7] = interval - CMD3[7] * 10;
	CMD4[7] += 31;
	if (CMD3[7] != 0) {
		CMD3[7] += 31;
	} else
		CMD3[7] = 91;
	HAL_UART_Transmit(&huart1, CMD3, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD4, 13, USARTSENDTIME);
	HAL_UART_Transmit(&huart1, CMD5, 13, USARTSENDTIME);
}
/**
 * @功能简介 : 修改设置界面的密码保护
 * @入口参数 : 时间间隔
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2020/01/03
 */
void Change_Conf_PassW(uint8_t PassW) {
	uint8_t CMD[13] = { 0xEE, 0x32, 0x01, 80, 0x00, 210, 0x00, 31, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	if (PassW == 0) {
		CMD[7] = 51;
	} else {
		CMD[7] = 52;
	}
	HAL_UART_Transmit(&huart1, CMD, 13, USARTSENDTIME);
}
/**
 * @功能简介 : 修改校准模式
 * @入口参数 : 校准模式
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_CalType(uint8_t caltype) {
	switch (caltype) {
	case 0:
		ShowCalTypeCMD[7] = 87;
		break;
	case 1:
		ShowCalTypeCMD[7] = 88;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowCalTypeCMD, 13, USARTSENDTIME);
}

/**
 * @功能简介 : 修改大气压
 * @入口参数 : 单位索引号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Change_AirPressureUnit(uint8_t unit) {
	switch (unit) {
	case 0:
		ShowAirPressureUnitCMD[7] = 24;
		ShowAirPressureNumUnitCMD[7] = 24;
		break;
	case 1:
		ShowAirPressureUnitCMD[7] = 29;
		ShowAirPressureNumUnitCMD[7] = 29;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowAirPressureUnitCMD, 13,
	USARTSENDTIME);
}

/**
 * @功能简介 : 修改上一次系数校正值
 * @入口参数 : 传感器系数
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2020/01/03
 */
void Change_AirPressure(float cell) {
	if (tempdata.airpressureunit == 0) {
		ShowAirPressureNumUnitCMD[7] = 24;
		ShowAirPressureNum1CMD[7] = cell;
		ShowAirPressureNum2CMD[7] = 30;
		ShowAirPressureNum3CMD[7] = (cell * 10
				- ShowAirPressureNum1CMD[7] * 10);
		ShowAirPressureNum4CMD[7] = cell * 100
				- ShowAirPressureNum1CMD[7] * 100
				- ShowAirPressureNum3CMD[7] * 10;
		ShowAirPressureNum5CMD[7] = 31
				+ (cell * 1000 - ShowAirPressureNum1CMD[7] * 1000
						- ShowAirPressureNum3CMD[7] * 100
						- ShowAirPressureNum4CMD[7] * 10);
		ShowAirPressureNum4CMD[7] += 31;
		ShowAirPressureNum3CMD[7] += 31;
		if (ShowAirPressureNum1CMD[7] != 0) {
			ShowAirPressureNum1CMD[7] += 31;
		} else
			ShowAirPressureNum1CMD[7] = 91;
		ShowAirPressureNum1CMD[3] = 14;
		ShowAirPressureNum2CMD[3] = 33;
		ShowAirPressureNum3CMD[3] = 46;
		ShowAirPressureNum4CMD[3] = 65;
		ShowAirPressureNum5CMD[3] = 84;
	} else if (tempdata.airpressureunit == 1) {
		ShowAirPressureNumUnitCMD[7] = 29;
		cell *= 1000;
		ShowAirPressureNum1CMD[7] = cell / 1000;
		ShowAirPressureNum2CMD[7] = (cell
				- ShowAirPressureNum1CMD[7] * 1000) / 100;
		ShowAirPressureNum3CMD[7] = (cell
				- ShowAirPressureNum1CMD[7] * 1000
				- ShowAirPressureNum2CMD[7] * 100) / 10;
		ShowAirPressureNum4CMD[7] = 31
				+ (cell - ShowAirPressureNum1CMD[7] * 1000
						- ShowAirPressureNum2CMD[7] * 100
						- ShowAirPressureNum3CMD[7] * 10);
		ShowAirPressureNum5CMD[7] = 91;
		ShowAirPressureNum3CMD[7] += 31;
		ShowAirPressureNum2CMD[7] += 31;
		if (ShowAirPressureNum1CMD[7] != 0) {
			ShowAirPressureNum1CMD[7] += 31;
		} else
			ShowAirPressureNum1CMD[7] = 91;
		ShowAirPressureNum1CMD[3] = 14;
		ShowAirPressureNum2CMD[3] = 33;
		ShowAirPressureNum3CMD[3] = 52;
		ShowAirPressureNum4CMD[3] = 71;
		ShowAirPressureNum5CMD[3] = 90;
	}

	HAL_UART_Transmit(&huart1, ShowAirPressureNum1CMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowAirPressureNum2CMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowAirPressureNum3CMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowAirPressureNum4CMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowAirPressureNum5CMD, 13,
	USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowAirPressureNumUnitCMD, 13,
	USARTSENDTIME);
}

/**
 * @功能简介 : 校正界面显示修正的值
 * @入口参数 : 修正的值
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/11/03
 */
void Change_Cal_PPM_Fixed(float ppm) {
	ShowConf4mATempNum1CMD[7] = ppm / 10;
	ShowConf4mATempNum2CMD[7] = ppm - ShowConf4mATempNum1CMD[7] * 10;
	ShowConf4mATempNum4CMD[7] = (ppm - ShowConf4mATempNum1CMD[7] * 10
			- ShowConf4mATempNum2CMD[7]) * 10;
	ShowConf4mATempNum5CMD[7] = 31
			+ (ppm - ShowConf4mATempNum1CMD[7] * 10 - ShowConf4mATempNum2CMD[7]
					- ShowConf4mATempNum4CMD[7] * 0.1) * 100;
	ShowConf4mATempNum4CMD[7] += 31;
	ShowConf4mATempNum2CMD[7] += 31;
	if (ShowConf4mATempNum1CMD[7] != 0) {
		ShowConf4mATempNum1CMD[7] += 31;
	} else
		ShowConf4mATempNum1CMD[7] = 91;
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum1CMD,
			sizeof(ShowConf4mATempNum1CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum2CMD,
			sizeof(ShowConf4mATempNum2CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum3CMD,
			sizeof(ShowConf4mATempNum3CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum4CMD,
			sizeof(ShowConf4mATempNum4CMD), USARTSENDTIME);
	HAL_UART_Transmit(&huart1, ShowConf4mATempNum5CMD,
			sizeof(ShowConf4mATempNum5CMD), USARTSENDTIME);
	switch (savedata.mode) {
	case 0:
		ShowCalPPMFixedUnitCMD[7] = 27;
		break;
	case 1:
		ShowCalPPMFixedUnitCMD[7] = 28;
		break;
	case 2:
		ShowCalPPMFixedUnitCMD[7] = 23;
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, ShowCalPPMFixedUnitCMD,
			sizeof(ShowCalPPMFixedUnitCMD),
			USARTSENDTIME);
}

/**
 * @功能简介 : 控制选择条
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Select_Next(uint8_t Selection) {
//设置前景黑色后填充长方形清除选择条
	HAL_UART_Transmit(&huart1, SetForeBlackCMD, 8, USARTSENDTIME);
	switch (Selection) {
	case 2:
		//清除选择条1
		HAL_UART_Transmit(&huart1, ShowConfigUnselect1CMD, 13,
		USARTSENDTIME);
		//显示选择条3
		HAL_UART_Transmit(&huart1, ShowConfigSelect3CMD, 13,
		USARTSENDTIME);
		break;
	case 3:
		//清除选择条2
		HAL_UART_Transmit(&huart1, ShowConfigUnselect2CMD, 13,
		USARTSENDTIME);
		//显示选择条4
		HAL_UART_Transmit(&huart1, ShowConfigSelect4CMD, 13,
		USARTSENDTIME);
		break;
	case 4:
		//清除选择条3
		HAL_UART_Transmit(&huart1, ShowConfigUnselect3CMD, 13,
		USARTSENDTIME);
		//显示选择条5
		HAL_UART_Transmit(&huart1, ShowConfigSelect5CMD, 13,
		USARTSENDTIME);
		break;
	default:
		break;
	}
}

/**
 * @功能简介 : Config界面函数
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Conf_UI(void) {
	uint8_t configCurrentPage = 1, configLastPage = 0, CurrentSelect = 1;
	uint8_t BTN_TIME = 0;
	uint32_t i = 0;
	memcpy(&tempdata, &savedata, sizeof(savedata));
	memcpy(&sdateconfstructureget, &sdatestructureget,
			sizeof(sdatestructureget));
	memcpy(&stimeconfstructureget, &stimestructureget,
			sizeof(stimestructureget));
	memcpy(&sdateconftempstructureget, &sdateconfstructureget,
			sizeof(sdateconfstructureget));
	memcpy(&stimeconftempstructureget, &stimeconfstructureget,
			sizeof(stimeconfstructureget));
	while (configFlag) {
		BTN_TIME = 0;
		if (configCurrentPage == 1 && configLastPage != 1) {
			Enter_Conf_Page1();
			configLastPage = 1;
			//屏蔽刚进入设置按键未释放
			while (!BTN_MODE() && (i < 10000))
				i++;
			while (!BTN_CONFIG() && (i < 10000))
				i++;
			while (!BTN_MODE() && (i < 10000))
				i++;
			while (!BTN_RIGHT() && (i < 10000))
				i++;
			while (!BTN_ENTER() && (i < 10000))
				i++;
			HAL_Delay(1000);
		}
		if (configCurrentPage == 8 && configLastPage != 8) {
			Enter_Conf_Page1_2();
			configLastPage = 8;
		}
		if (configCurrentPage == 2 && configLastPage != 2) {
			Enter_Conf_Page2();
			configLastPage = 2;
		}
		if (configCurrentPage == 3 && configLastPage != 3) {
			Enter_Conf_Page3();
			configLastPage = 3;
		}
		if (configCurrentPage == 4 && configLastPage != 4) {
			Enter_Conf_Page4();
			configLastPage = 4;
		}
		if (configCurrentPage == 5 && configLastPage != 5) {
			Enter_Conf_Page5();
			configLastPage = 5;
		}
		if (configCurrentPage == 6 && configLastPage != 6) {
			Enter_Conf_Page6();
			configLastPage = 6;
		}
		if (configCurrentPage == 7 && configLastPage != 7) {
			Enter_Conf_Page7();
			configLastPage = 7;
		}
		if (configCurrentPage == 9 && configLastPage != 9) {
			Enter_Conf_Page8();
			configLastPage = 9;
		}
		//按下Right键
		while (!BTN_RIGHT()) {
			//长按、短按延时优化
			if (BTN_TIME < 6) {
				HAL_Delay(200);
			} else if (BTN_TIME >= 6 && BTN_TIME < 15)
				HAL_Delay(50);
			else {
				HAL_Delay(3);
			}
			switch (configCurrentPage) {
			case 1:
				switch (CurrentSelect) {
				case 1:
					//修改测量模式设置
					tempdata.mode++;
					if (tempdata.mode > 2) {
						tempdata.mode = 0;
					}
					Change_Conf_Unit(tempdata.mode);
					break;
//				case 2:
//					//盐度修正系数
//					tempdata.salinity += 0.1;
//					if (tempdata.salinity > 45) {
//						tempdata.salinity = 0;
//					}
//					Change_Conf_Salinity(tempdata.salinity);
//					break;
				case 2:
					//温度修正系数
					tempdata.temp += 0.1;
					if (tempdata.temp > 50) {
						tempdata.temp = 0;
					}
					if (f_Temp != 0) {
						tempdata.ktemp = tempdata.temp / f_Temp;
					} else
						tempdata.ktemp = 1;
					Change_Conf_Temp(tempdata.temp);
					break;
				default:
					break;
				}
				break;
			case 2:
				switch (CurrentSelect) {
				case 1:
					//修改4mA对应溶氧值
					tempdata.ppm4mA += 0.01;
					if (tempdata.ppm4mA > 20
							|| tempdata.ppm4mA >= tempdata.ppm20mA) {
						tempdata.ppm4mA = 0;
					}
					Change_Conf_PPM4mA(tempdata.ppm4mA);
					break;
				case 2:
					//修改20mA对应溶氧值
					tempdata.ppm20mA += 0.01;
					if (tempdata.ppm20mA > 20) {
						tempdata.ppm20mA = tempdata.ppm4mA + 0.01;
					}
					Change_Conf_ppm20mA(tempdata.ppm20mA);
					break;
				case 3:
					//修改4mA对应温度值
					tempdata.temp4mA += 0.1;
					if (tempdata.temp4mA > 99.9
							|| tempdata.temp4mA >= tempdata.temp20mA) {
						tempdata.temp4mA = 0;
					}
					Change_Conf_temp4mA(tempdata.temp4mA);
					break;
				case 4:
					//修改20mA对应温度值
					tempdata.temp20mA += 0.1;
					if (tempdata.temp20mA > 99.9) {
						tempdata.temp20mA = tempdata.temp4mA + 0.1;
					}
					Change_Conf_temp20mA(tempdata.temp20mA);
					break;
				default:
					break;
				}
				break;
			case 3:
				switch (CurrentSelect) {
				case 1:
					//修改高点报警(自动/关闭)
					tempdata.uplimitauto += 1;
					if (tempdata.uplimitauto > 1) {
						tempdata.uplimitauto = 0;
					}
					Change_Conf_UpLimitAuto(tempdata.uplimitauto);
					break;
				case 2:
					//修改高点报警值
					tempdata.uplimit += 0.01;
					if (tempdata.uplimit > 20) {
						tempdata.uplimit = tempdata.lowlimit + 0.02;
					}
					Change_Conf_UpLimit(tempdata.uplimit);
					break;
				case 3:
					//修改高点报警值延时区间
					tempdata.uplimitdelay += 0.01;
					if (tempdata.uplimitdelay
							> (tempdata.uplimit - tempdata.lowlimit)) {
						tempdata.uplimitdelay = 0;
					}
					Change_Conf_UpLimitDelay(tempdata.uplimitdelay);
					break;
				default:
					break;
				}
				break;
			case 4:
				switch (CurrentSelect) {
				case 1:
					//修改低点报警(自动/关闭)
					tempdata.lowlimitauto += 1;
					if (tempdata.lowlimitauto > 1) {
						tempdata.lowlimitauto = 0;
					}
					Change_Conf_LowLimitAuto(tempdata.lowlimitauto);
					break;
				case 2:
					//修改低点报警值
					tempdata.lowlimit += 0.01;
					if (tempdata.lowlimit >= tempdata.uplimit) {
						tempdata.lowlimit = 0;
					}
					Change_Conf_LowLimit(tempdata.lowlimit);
					break;
				case 3:
					//修改低点报警值延时区间
					tempdata.lowlimitdelay += 0.01;
					if (tempdata.lowlimitdelay
							> (tempdata.uplimit - tempdata.lowlimit)) {
						tempdata.lowlimitdelay = 0;
					}
					Change_Conf_LowLimitDelay(tempdata.lowlimitdelay);
					break;
				default:
					break;
				}
				break;
			case 5:
				switch (CurrentSelect) {
				case 1:
					//修改清洗时间
					tempdata.washholdtime += 1;
					if (tempdata.washholdtime > 60) {
						tempdata.washholdtime = 1;
					}
					Change_Conf_WashHoldTime(tempdata.washholdtime);
					break;
				case 2:
					//修改清洗间隔时间
					tempdata.washdelaytime += 1;
					if (tempdata.washdelaytime > 72) {
						tempdata.washdelaytime = 1;
					}
					Change_Conf_WashDelayTime(tempdata.washdelaytime);
					break;
				case 3:
					//修改滤波系数
					tempdata.filter += 1;
					if (tempdata.filter > 60) {
						tempdata.filter = 0;
					}
					Change_Conf_Filter(tempdata.filter);
					break;
				case 4:
					//修改亮度
					tempdata.brightness += 1;
					if (tempdata.brightness > 10) {
						tempdata.brightness = 1;
					}
					Change_Conf_Brightness(tempdata.brightness);
					break;
				default:
					break;
				}
				break;
			case 6:
				switch (CurrentSelect) {
				case 1:
					//校准4mA
					tempdata.MIN += 1;
					if (tempdata.MIN > (ADMIN + ADMINCAL)) {
						tempdata.MIN = (ADMIN - ADMINCAL);
					}
					Change_Conf_ADMIN((tempdata.MIN - ADMIN) / 10);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
							tempdata.MIN);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
							tempdata.MIN);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
					break;
				case 2:
					//校准20mA
					tempdata.MAX += 1;
					if (tempdata.MAX > (ADMAX + ADMAXCAL)) {
						tempdata.MAX = (ADMAX - ADMAXCAL);
					}
					Change_Conf_ADMAX((tempdata.MAX - ADMAX) / 10);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
							tempdata.MAX);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
							tempdata.MAX);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
					break;
				case 3:
					//修改历史记录时间间隔
					tempdata.interval += 1;
					if (tempdata.interval > 24) {
						tempdata.interval = 1;
					}
					Change_Conf_Interval(tempdata.interval);
					break;
				case 4:
					//开关密码保护
					if (tempdata.password == 0) {
						tempdata.password = 1;
						Change_Conf_PassW(tempdata.password);
					} else if (tempdata.password != 0) {
						//进入输入密码界面
						if (Enter_PasW_Page(1) == 1) {
							tempdata.password = 0;
							Change_Conf_PassW(tempdata.password);
						} else {
							tempdata.password = 1;
							Change_Conf_PassW(tempdata.password);
						}
					}
					break;
				default:
					break;
				}
				break;
			case 7:
				switch (CurrentSelect) {
				case 1:
					//修改年份
					sdateconfstructureget.Year += 1;
					if (sdateconfstructureget.Year > 99.9) {
						sdateconfstructureget.Year = 0;
					}
					Change_Conf_Year(sdateconfstructureget.Year);
					break;
				case 2:
					//修改月份
					sdateconfstructureget.Month += 1;
					if (sdateconfstructureget.Month > 12) {
						sdateconfstructureget.Month = 1;
					}
					Change_Conf_Month(sdateconfstructureget.Month);
					break;
				case 3:
					//修改日期
					sdateconfstructureget.Date += 1;
					if (sdateconfstructureget.Date > 31) {
						sdateconfstructureget.Date = 1;
					}
					Change_Conf_Date(sdateconfstructureget.Date);
					break;
				default:
					break;
				}
				break;
			case 9:
				switch (CurrentSelect) {
				case 1:
					//修改时
					stimeconfstructureget.Hours += 1;
					if (stimeconfstructureget.Hours > 23) {
						stimeconfstructureget.Hours = 0;
					}
					Change_Conf_Hours(stimeconfstructureget.Hours);
					break;
				case 2:
					//修改分
					stimeconfstructureget.Minutes += 1;
					if (stimeconfstructureget.Minutes > 60) {
						stimeconfstructureget.Minutes = 0;
					}
					Change_Conf_Minutes(stimeconfstructureget.Minutes);
					break;
				case 3:
					//修改秒
					stimeconfstructureget.Seconds += 1;
					if (stimeconfstructureget.Seconds > 60) {
						stimeconfstructureget.Seconds = 0;
					}
					Change_Conf_Seconds(stimeconfstructureget.Seconds);
					break;
				default:
					break;
				}
				break;
			case 8:
				switch (CurrentSelect) {
				case 1:
					/* 温度补偿模式 */
					if (tempdata.tempfactortype == 1) {
						tempdata.tempfactortype = 0;
					} else {
						tempdata.tempfactortype = 1;
					}
					Change_Conf_TempFactorType(tempdata.tempfactortype);
					break;
				case 2:
					/* 温度补偿系数 */
					tempdata.tempfactor += 0.01;
					if (tempdata.tempfactor > 20) {
						tempdata.tempfactor = 0;
					}
					Change_Conf_TempFactor(tempdata.tempfactor);
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			BTN_TIME++;
			if (BTN_TIME > 100) {
				BTN_TIME = 100;
			}
		}	//End 按下right键

		//按下Mode键
		while (!BTN_MODE()) {
			//长按、短按延时优化
			if (BTN_TIME < 6) {
				HAL_Delay(200);
			} else if (BTN_TIME >= 6 && BTN_TIME < 15)
				HAL_Delay(50);
			else {
				HAL_Delay(3);
			}
			switch (configCurrentPage) {
			case 1:
				switch (CurrentSelect) {
				case 1:
					//修改测量模式设置
					tempdata.mode--;
					if (tempdata.mode < 0 || tempdata.mode > 2) {
						tempdata.mode = 2;
					}
					Change_Conf_Unit(tempdata.mode);
					break;
//				case 2:
//					//盐度修正系数
//					tempdata.salinity -= 0.1;
//					if (tempdata.salinity < 0) {
//						tempdata.salinity = 45;
//					}
//					Change_Conf_Salinity(tempdata.salinity);
//					break;
				case 2:
					//温度修正系数
					tempdata.temp -= 0.1;
					if (tempdata.temp < 0) {
						tempdata.temp = 50;
					}
					if (f_Temp != 0) {
						tempdata.ktemp = tempdata.temp / f_Temp;
					} else
						tempdata.ktemp = 1;
					Change_Conf_Temp(tempdata.temp);
					break;
				default:
					break;
				}
				break;
			case 2:
				switch (CurrentSelect) {
				case 1:
					//修改4mA对应溶氧值
					tempdata.ppm4mA -= 0.01;
					if (tempdata.ppm4mA < 0) {
						tempdata.ppm4mA = tempdata.ppm20mA - 0.01;
					}
					Change_Conf_PPM4mA(tempdata.ppm4mA);
					break;
				case 2:
					//修改20mA对应溶氧值
					tempdata.ppm20mA -= 0.01;
					if (tempdata.ppm20mA <= tempdata.ppm4mA
							|| tempdata.ppm20mA < 0) {
						tempdata.ppm20mA = 20;
					}
					Change_Conf_ppm20mA(tempdata.ppm20mA);
					break;
				case 3:
					//修改4mA对应温度值
					tempdata.temp4mA -= 0.1;
					if (tempdata.temp4mA < 0) {
						tempdata.temp4mA = tempdata.temp20mA - 0.1;
					}
					Change_Conf_temp4mA(tempdata.temp4mA);
					break;
				case 4:
					//修改20mA对应温度值
					tempdata.temp20mA -= 0.1;
					if (tempdata.temp20mA < 0
							|| tempdata.temp20mA <= tempdata.temp4mA) {
						tempdata.temp20mA = 99.9;
					}
					Change_Conf_temp20mA(tempdata.temp20mA);
					break;
				default:
					break;
				}
				break;
			case 3:
				switch (CurrentSelect) {
				case 1:
					//修改高点报警(自动/关闭)
					tempdata.uplimitauto -= 1;
					if (tempdata.uplimitauto < 0 || tempdata.uplimitauto > 1) {
						tempdata.uplimitauto = 1;
					}
					Change_Conf_UpLimitAuto(tempdata.uplimitauto);
					break;
				case 2:
					//修改高点报警值
					tempdata.uplimit -= 0.01;
					if (tempdata.uplimit < 0
							|| tempdata.uplimit < tempdata.lowlimit) {
						tempdata.uplimit = 20;
					}
					Change_Conf_UpLimit(tempdata.uplimit);
					break;
				case 3:
					//修改高点报警值延时区间
					tempdata.uplimitdelay -= 0.01;
					if (tempdata.uplimitdelay < 0) {
						tempdata.uplimitdelay = tempdata.uplimit
								- tempdata.lowlimit;
					}
					Change_Conf_UpLimitDelay(tempdata.uplimitdelay);
					break;
				default:
					break;
				}
				break;
			case 4:
				switch (CurrentSelect) {
				case 1:
					//修改低点报警(自动/关闭)
					tempdata.lowlimitauto -= 1;
					if (tempdata.lowlimitauto < 0
							|| tempdata.lowlimitauto > 1) {
						tempdata.lowlimitauto = 1;
					}
					Change_Conf_LowLimitAuto(tempdata.lowlimitauto);
					break;
				case 2:
					//修改低点报警值
					tempdata.lowlimit -= 0.01;
					if (tempdata.lowlimit < 0) {
						tempdata.lowlimit = tempdata.uplimit - 0.02;
					}
					Change_Conf_LowLimit(tempdata.lowlimit);
					break;
				case 3:
					//修改低点报警值延时区间
					tempdata.lowlimitdelay -= 0.01;
					if (tempdata.lowlimitdelay < 0) {
						tempdata.lowlimitdelay = tempdata.uplimit
								- tempdata.lowlimit;
					}
					Change_Conf_LowLimitDelay(tempdata.lowlimitdelay);
					break;
				default:
					break;
				}
				break;
			case 5:
				switch (CurrentSelect) {
				case 1:
					//修改清洗时间
					tempdata.washholdtime -= 1;
					if (tempdata.washholdtime < 1) {
						tempdata.washholdtime = 60;
					}
					Change_Conf_WashHoldTime(tempdata.washholdtime);
					break;
				case 2:
					//修改清洗间隔时间
					tempdata.washdelaytime -= 1;
					if (tempdata.washdelaytime < 1) {
						tempdata.washdelaytime = 72;
					}
					Change_Conf_WashDelayTime(tempdata.washdelaytime);
					break;
				case 3:
					//修改滤波系数

					if (tempdata.filter == 0) {
						tempdata.filter = 60;
					} else
						tempdata.filter -= 1;
					Change_Conf_Filter(tempdata.filter);
					break;
				case 4:
					//修改亮度
					tempdata.brightness -= 1;
					if (tempdata.brightness < 1) {
						tempdata.brightness = 10;
					}
					Change_Conf_Brightness(tempdata.brightness);
					break;
				default:
					break;
				}
				break;
			case 6:
				switch (CurrentSelect) {
				case 1:
					//校准4mA
					tempdata.MIN -= 1;
					if (tempdata.MIN < (ADMIN - ADMINCAL)) {
						tempdata.MIN = (ADMIN + ADMINCAL);
					}
					Change_Conf_ADMIN((tempdata.MIN - ADMIN) / 10);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
							tempdata.MIN);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
							tempdata.MIN);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
					break;
				case 2:
					//校准20mA
					tempdata.MAX -= 1;
					if (tempdata.MAX < (ADMAX - ADMAXCAL)) {
						tempdata.MAX = (ADMAX + ADMAXCAL);
					}
					Change_Conf_ADMAX((tempdata.MAX - ADMAX) / 10);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
							tempdata.MAX);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
							tempdata.MAX);
					HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
					break;
				case 3:
					//修改历史记录时间间隔
					tempdata.interval -= 1;
					if (tempdata.interval < 1) {
						tempdata.interval = 24;
					}
					Change_Conf_Interval(tempdata.interval);
					break;
				case 4:
					//开关密码保护
					if (tempdata.password == 0) {
						tempdata.password = 1;
						Change_Conf_PassW(tempdata.password);
					} else if (tempdata.password != 0) {
						//进入输入密码界面
						if (Enter_PasW_Page(1) == 1) {
							tempdata.password = 0;
							Change_Conf_PassW(tempdata.password);
						} else {
							tempdata.password = 1;
							Change_Conf_PassW(tempdata.password);
						}
					}
					break;
				default:
					break;
				}
				break;
			case 7:
				switch (CurrentSelect) {
				case 1:
					//修改年份
					sdateconfstructureget.Year -= 1;
					if (sdateconfstructureget.Year < 0) {
						sdateconfstructureget.Year = 99;
					}
					Change_Conf_Year(sdateconfstructureget.Year);
					break;
				case 2:
					//修改月份
					sdateconfstructureget.Month -= 1;
					if (sdateconfstructureget.Month < 1) {
						sdateconfstructureget.Month = 12;
					}
					Change_Conf_Month(sdateconfstructureget.Month);
					break;
				case 3:
					//修改日期
					sdateconfstructureget.Date -= 1;
					if (sdateconfstructureget.Date < 1) {
						sdateconfstructureget.Date = 31;
					}
					Change_Conf_Date(sdateconfstructureget.Date);
					break;
				default:
					break;
				}
				break;
			case 9:
				switch (CurrentSelect) {
				case 1:
					//修改时
					if (stimeconfstructureget.Hours == 0) {
						stimeconfstructureget.Hours = 23;
					} else
						stimeconfstructureget.Hours -= 1;
					Change_Conf_Hours(stimeconfstructureget.Hours);
					break;
				case 2:
					//修改分
					if (stimeconfstructureget.Minutes == 0) {
						stimeconfstructureget.Minutes = 60;
					} else
						stimeconfstructureget.Minutes -= 1;
					Change_Conf_Minutes(stimeconfstructureget.Minutes);
					break;
				case 3:
					//修改秒
					if (stimeconfstructureget.Seconds == 0) {
						stimeconfstructureget.Seconds = 60;
					} else
						stimeconfstructureget.Seconds -= 1;
					Change_Conf_Seconds(stimeconfstructureget.Seconds);
					break;
				default:
					break;
				}
				break;
			case 8:
				switch (CurrentSelect) {
				case 1:
					/* 温度补偿模式 */
					if (tempdata.tempfactortype == 1) {
						tempdata.tempfactortype = 0;
					} else {
						tempdata.tempfactortype = 1;
					}
					Change_Conf_TempFactorType(tempdata.tempfactortype);
					break;
				case 2:
					/* 温度补偿系数 */
					tempdata.tempfactor -= 0.01;
					if (tempdata.tempfactor < 0) {
						tempdata.tempfactor = 20;
					}
					Change_Conf_TempFactor(tempdata.tempfactor);
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			BTN_TIME++;
			if (BTN_TIME > 100) {
				BTN_TIME = 100;
			}
		}	//end 按下mode键

		//按下Enter键，选中下一条
		if (!BTN_ENTER()) {
			HAL_Delay(100);
			if (!BTN_ENTER()) {
				CurrentSelect++;
				if ((configCurrentPage == 1 || configCurrentPage == 8)
						&& (CurrentSelect == 3)) {
					CurrentSelect += 2;
				}
//				if (configCurrentPage == 8 && CurrentSelect == 3) {
//					CurrentSelect += 2;
//				}
				//每页4个设置(第三、四、六页3个设置)，超出跳转下一页，超出页数跳回首页
				if ((CurrentSelect > 3)
						&& (configCurrentPage == 3 || configCurrentPage == 4
								|| configCurrentPage == 7
								|| configCurrentPage == 9)) {
					CurrentSelect++;
				}
				if (CurrentSelect > 4) {
					CurrentSelect = 1;
					if (configCurrentPage == 1) {
						configCurrentPage = 8;
					} else if (configCurrentPage == 8) {
						configCurrentPage = 2;
					} else if (configCurrentPage == 7) {
						configCurrentPage = 9;
					} else if (configCurrentPage == 9) {
						configCurrentPage = 1;
					} else {
						configCurrentPage++;
					}
				}
				Select_Next(CurrentSelect);
				while (!BTN_ENTER())
					;
			}
		}

		//按下Conf键，保存设置，返回主界面
		if (!BTN_CONFIG()) {
			HAL_Delay(100);
			if (!BTN_CONFIG()) {
				configFlag = 0;
				if (sdateconfstructureget.Month
						!= sdateconftempstructureget.Month
						|| sdateconfstructureget.Date
								!= sdateconftempstructureget.Date
						|| sdateconfstructureget.Year
								!= sdateconftempstructureget.Year
						|| stimeconfstructureget.Hours
								!= stimeconftempstructureget.Hours
						|| stimeconfstructureget.Minutes
								!= stimeconftempstructureget.Minutes
						|| stimeconfstructureget.Seconds
								!= stimeconftempstructureget.Seconds) {
					/* 修改时间 */
					RTC_TimeTypeDef sTime;
					RTC_DateTypeDef DateToUpdate;
					/* 配置日期 */
					DateToUpdate.Month = sdateconfstructureget.Month;
					DateToUpdate.Date = sdateconfstructureget.Date;
					DateToUpdate.Year = sdateconfstructureget.Year;
					HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
					/* 配置时间 */
					sTime.Hours = stimeconfstructureget.Hours;
					sTime.Minutes = stimeconfstructureget.Minutes;
					sTime.Seconds = stimeconfstructureget.Seconds;
					HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					/* 写入一个数值：0x32F1到RTC备份数据寄存器1 */
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F1);
					lastHours = stimestructureget.Hours;
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, DateToUpdate.Year);
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, DateToUpdate.Month);
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, DateToUpdate.Date);
				}
				memcpy(&savedata, &tempdata, sizeof(savedata));
				writeConfig();
				while (!BTN_CONFIG())
					;
			}
		}
	}
	/* 退出设置后恢复主界面 */
	LCD_Init();
}

/**
 * @功能简介 : 校准界面函数
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Cal_UI(void) {
	uint8_t configCurrentPage = 1, configLastPage = 0, CurrentSelect = 1;
	uint8_t BTN_TIME = 0;
	uint8_t i;
	float f_PPM_Cal = 0;
	//显示温度设置数字1
	uint8_t ShowCalTempNum1CMD[13] = { 0xEE, 0x32, 0x01, 108, 0x00, 12, 0x00,
			31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	//显示温度设置数字2
	uint8_t ShowCalTempNum2CMD[13] = { 0xEE, 0x32, 0x01, 128, 0x00, 12, 0x00,
			31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	//显示温度设置数字3-小数点
	uint8_t ShowCalTempNum3CMD[13] = { 0xEE, 0x32, 0x01, 148, 0x00, 12, 0x00,
			30, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	//显示温度设置数字4
	uint8_t ShowCalTempNum4CMD[13] = { 0xEE, 0x32, 0x01, 160, 0x00, 12, 0x00,
			31, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	//显示温度设置单位
	uint8_t ShowCalTempUnitCMD[13] = { 0xEE, 0x32, 0x01, 180, 0x00, 10, 0x00,
			81, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	memcpy(&tempdata, &savedata, sizeof(savedata));
	while (calFlag) {
		BTN_TIME = 0;
		if (configCurrentPage == 1 && configLastPage != 1) {
			Enter_Cal_Page1();
			configLastPage = 1;
			//屏蔽刚进入设置按键未释放
			while (!BTN_MODE())
				;
			while (!BTN_CONFIG())
				;
			while (!BTN_MODE())
				;
			while (!BTN_RIGHT())
				;
			while (!BTN_ENTER())
				;
		}
		if (configCurrentPage == 2 && configLastPage != 2) {
			Enter_Cal_Page2();
			configLastPage = 2;
		}
		/* 空气校准 */
		if (configCurrentPage == 2 && configLastPage == 2 && CurrentSelect == 1
				&& tempdata.caltype == 0) {
			//显示正在校准文字
			HAL_UART_Transmit(&huart1, ShowCalingStatusCMD, 13,
			USARTSENDTIME);
			CurrentSelect = 2;
			Select_Next(CurrentSelect);
			result = 0;
			//显示正在校正的氧浓度值
			for (i = 0; i < CAL_COUNT; i++) {
				//显示正在校准闪烁
				HAL_UART_Transmit(&huart1, ShowConfigUnselect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowConfigUnselect3CMD, 13,
				USARTSENDTIME);
				/* 获取传感器数据 */
				RS485_EN(1);
				HAL_UART_Transmit(&huart2, GetSensor, 8, USARTSENDTIME);
				RS485_EN(0);
				HAL_Delay(500);
				if (f_Temp_fixed > 50)
					f_Temp_fixed = 50;
				f_PPM_Cal = waterDO[(uint32_t) (f_Temp_fixed)]
						* (tempdata.cell * 100000
								- waterPerssure[(uint32_t) (f_Temp_fixed)])
						/ (101325 - waterPerssure[(uint32_t) (f_Temp_fixed)]);
				//显示温度
				ShowCalTempNum1CMD[7] = f_Temp_fixed / 10;
				ShowCalTempNum2CMD[7] = f_Temp_fixed
						- ShowCalTempNum1CMD[7] * 10;
				ShowCalTempNum4CMD[7] = 31
						+ (f_Temp_fixed - ShowCalTempNum1CMD[7] * 10
								- ShowCalTempNum2CMD[7]) * 10;
				ShowCalTempNum2CMD[7] += 31;
				if (ShowCalTempNum1CMD[7] != 0) {
					ShowCalTempNum1CMD[7] += 31;
				} else
					ShowCalTempNum1CMD[7] = 91;
				HAL_UART_Transmit(&huart1, ShowCalTempNum1CMD,
						sizeof(ShowCalTempNum1CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum2CMD,
						sizeof(ShowCalTempNum2CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum3CMD,
						sizeof(ShowCalTempNum3CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum4CMD,
						sizeof(ShowCalTempNum4CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempUnitCMD,
						sizeof(ShowCalTempUnitCMD),
						USARTSENDTIME);

				//显示校准值
				Change_Conf_PPM4mA(f_PPM_Cal);

				//显示正在校准闪烁
				HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowConfigSelect3CMD, 13,
				USARTSENDTIME);
				HAL_Delay(500);
			}
			//显示校准结果
			if (result >= 10) {
				HAL_UART_Transmit(&huart1, ShowCalFinishStatus1CMD, 13,
				USARTSENDTIME);
				CurrentSelect = 3;
				Select_Next(CurrentSelect);
			} else {
				HAL_UART_Transmit(&huart1, ShowCalFinishStatus2CMD, 13,
				USARTSENDTIME);
				CurrentSelect = 3;
				Select_Next(CurrentSelect);
				CurrentSelect = 4;
				Select_Next(CurrentSelect);
			}
		}
		if (configCurrentPage == 2 && configLastPage == 2 && CurrentSelect == 3
				&& tempdata.caltype == 0) {
			Change_Cal_PPM_Fixed(f_PPM_Cal);
			if (f_Rs + tempdata.bdo != 0) {
				tempdata.kdo = f_PPM_Cal / (f_Rs + tempdata.bdo);
			}
		}

		/* 过程校准 */
		if (configCurrentPage == 2 && configLastPage == 2 && CurrentSelect == 1
				&& tempdata.caltype == 2) {
			//显示正在校准文字
			HAL_UART_Transmit(&huart1, ShowCalingStatusCMD, 13,
			USARTSENDTIME);
			CurrentSelect = 2;
			Select_Next(CurrentSelect);
			result = 0;
			//显示正在校正的氧浓度值
			for (i = 0; i < CAL_COUNT; i++) {
				//显示正在校准闪烁
				HAL_UART_Transmit(&huart1, ShowConfigUnselect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowConfigUnselect3CMD, 13,
				USARTSENDTIME);
				/* 获取传感器数据 */
				RS485_EN(1);
				HAL_UART_Transmit(&huart2, GetSensor, 8, USARTSENDTIME);
				RS485_EN(0);
				HAL_Delay(500);
				if (f_Temp_fixed > 50)
					f_Temp_fixed = 50;
				if (f_Rs < 0)
					f_Rs = 0;
				f_PPM_Cal = f_Rs + tempdata.bdo;
				if (f_PPM_Cal < 0)
					f_PPM_Cal = 0;
				if (f_PPM_Cal > 20)
					f_PPM_Cal = 20;
				//显示温度
				ShowCalTempNum1CMD[7] = f_Temp_fixed / 10;
				ShowCalTempNum2CMD[7] = f_Temp_fixed
						- ShowCalTempNum1CMD[7] * 10;
				ShowCalTempNum4CMD[7] = 31
						+ (f_Temp_fixed - ShowCalTempNum1CMD[7] * 10
								- ShowCalTempNum2CMD[7]) * 10;
				ShowCalTempNum2CMD[7] += 31;
				if (ShowCalTempNum1CMD[7] != 0) {
					ShowCalTempNum1CMD[7] += 31;
				} else
					ShowCalTempNum1CMD[7] = 91;
				HAL_UART_Transmit(&huart1, ShowCalTempNum1CMD,
						sizeof(ShowCalTempNum1CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum2CMD,
						sizeof(ShowCalTempNum2CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum3CMD,
						sizeof(ShowCalTempNum3CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum4CMD,
						sizeof(ShowCalTempNum4CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempUnitCMD,
						sizeof(ShowCalTempUnitCMD),
						USARTSENDTIME);
				//显示校准值
				Change_Conf_PPM4mA(f_PPM_Cal);
				//显示正在校准闪烁
				HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowConfigSelect3CMD, 13,
				USARTSENDTIME);
				HAL_Delay(500);
			}
			//显示校准结果
			if (result >= 10) {
				HAL_UART_Transmit(&huart1, ShowCalFinishStatus1CMD, 13,
				USARTSENDTIME);
				tempdata.kdo = 1;
				CurrentSelect = 3;
				Select_Next(CurrentSelect);
			} else {
				HAL_UART_Transmit(&huart1, ShowCalFinishStatus2CMD, 13,
				USARTSENDTIME);
				CurrentSelect = 3;
				Select_Next(CurrentSelect);
				CurrentSelect = 4;
				Select_Next(CurrentSelect);
			}
		}
		if (configCurrentPage == 2 && configLastPage == 2 && CurrentSelect == 3
				&& tempdata.caltype == 2) {
			Change_Cal_PPM_Fixed(f_PPM_Cal);
			if (f_Rs + tempdata.bdo != 0) {
				tempdata.kdo = f_PPM_Cal / (f_Rs + tempdata.bdo);
			}
		}

		/* 零点校准 */
		if (configCurrentPage == 2 && configLastPage == 2 && CurrentSelect == 1
				&& tempdata.caltype == 1) {
			//显示正在校准文字
			HAL_UART_Transmit(&huart1, ShowCalingStatusCMD, 13,
			USARTSENDTIME);
			CurrentSelect = 2;
			Select_Next(CurrentSelect);
			result = 0;
			//显示正在校正的氧浓度值
			for (i = 0; i < CAL_COUNT; i++) {
				//显示正在校准闪烁
				HAL_UART_Transmit(&huart1, ShowConfigUnselect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowConfigUnselect3CMD, 13,
				USARTSENDTIME);
				/* 获取传感器数据 */
				RS485_EN(1);
				HAL_UART_Transmit(&huart2, GetSensor, 8, USARTSENDTIME);
				RS485_EN(0);
				HAL_Delay(500);
				if (f_Temp_fixed > 50)
					f_Temp_fixed = 50;
				f_PPM_Cal = f_Rs * tempdata.kdo;
				if (f_PPM_Cal < 0)
					f_PPM_Cal = 0;
				if (f_PPM_Cal > 20)
					f_PPM_Cal = 20;
				if (f_PPM_Cal > 1) {
					result = 0;
					break;
				}
				//显示温度
				ShowCalTempNum1CMD[7] = f_Temp_fixed / 10;
				ShowCalTempNum2CMD[7] = f_Temp_fixed
						- ShowCalTempNum1CMD[7] * 10;
				ShowCalTempNum4CMD[7] = 31
						+ (f_Temp_fixed - ShowCalTempNum1CMD[7] * 10
								- ShowCalTempNum2CMD[7]) * 10;
				ShowCalTempNum2CMD[7] += 31;
				if (ShowCalTempNum1CMD[7] != 0) {
					ShowCalTempNum1CMD[7] += 31;
				} else
					ShowCalTempNum1CMD[7] = 91;
				HAL_UART_Transmit(&huart1, ShowCalTempNum1CMD,
						sizeof(ShowCalTempNum1CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum2CMD,
						sizeof(ShowCalTempNum2CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum3CMD,
						sizeof(ShowCalTempNum3CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempNum4CMD,
						sizeof(ShowCalTempNum4CMD),
						USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowCalTempUnitCMD,
						sizeof(ShowCalTempUnitCMD),
						USARTSENDTIME);
				//显示校准值
				Change_Conf_PPM4mA(f_PPM_Cal);
				//显示正在校准闪烁
				HAL_UART_Transmit(&huart1, ShowConfigSelect2CMD, 13,
				USARTSENDTIME);
				HAL_UART_Transmit(&huart1, ShowConfigSelect3CMD, 13,
				USARTSENDTIME);
				HAL_Delay(500);
			}
			//显示校准结果
			if (result >= 10) {
				HAL_UART_Transmit(&huart1, ShowCalFinishStatus1CMD, 13,
				USARTSENDTIME);
				CurrentSelect = 3;
				tempdata.bdo = 0;
				Select_Next(CurrentSelect);
			} else {
				HAL_UART_Transmit(&huart1, ShowCalFinishStatus2CMD, 13,
				USARTSENDTIME);
				CurrentSelect = 3;
				Select_Next(CurrentSelect);
				CurrentSelect = 4;
				Select_Next(CurrentSelect);
			}
		}
		if (configCurrentPage == 2 && configLastPage == 2 && CurrentSelect == 3
				&& tempdata.caltype == 1) {
			Change_Cal_PPM_Fixed(f_PPM_Cal);
		}
		//按下Right键
		while (!BTN_RIGHT()) {
			//长按、短按延时优化
			if (BTN_TIME < 6) {
				HAL_Delay(200);
			} else if (BTN_TIME >= 6 && BTN_TIME < 15)
				HAL_Delay(50);
			else {
				HAL_Delay(3);
			}
			switch (configCurrentPage) {
			case 1:
				switch (CurrentSelect) {
				case 1:
					//修改校正模式
					tempdata.caltype++;
					if (tempdata.caltype > 2) {
						tempdata.caltype = 0;
					}
					Change_CalType(tempdata.caltype);
					break;
				case 2:
					//修改大气压单位
					tempdata.airpressureunit++;
					if (tempdata.airpressureunit > 1) {
						tempdata.airpressureunit = 0;
					}
					Change_AirPressureUnit(tempdata.airpressureunit);
					Change_AirPressure(tempdata.cell);
					break;
				case 3:
					//修改大气压
					tempdata.cell += 0.001;
					if (tempdata.cell > 1.5) {
						tempdata.cell = 0.3;
					}
					Change_AirPressure(tempdata.cell);
					break;
				default:
					break;
				}
				break;
			case 2:
				switch (CurrentSelect) {
				case 1:
					break;
				case 2:
					break;
				case 3:
					if ((tempdata.caltype == 0) || (tempdata.caltype == 2)) {
						f_PPM_Cal = f_PPM_Cal + 0.01;
						if (f_PPM_Cal > 20)
							f_PPM_Cal = 20;
						Change_Cal_PPM_Fixed(f_PPM_Cal);
						if (f_Rs + tempdata.bdo != 0)
							tempdata.kdo = f_PPM_Cal / (f_Rs + tempdata.bdo);
					}
					if (tempdata.caltype == 1) {
						f_PPM_Cal = f_PPM_Cal + 0.01;
						if (f_PPM_Cal > 20)
							f_PPM_Cal = 20;
						Change_Cal_PPM_Fixed(f_PPM_Cal);
						if (f_Rs + tempdata.bdo != 0)
							tempdata.bdo = (f_PPM_Cal / tempdata.kdo) - f_Rs;
					}
					break;
				case 4:
//					tempdata.temp20mA += 0.1;
//					if (tempdata.temp20mA > 50) {
//						tempdata.temp20mA = tempdata.temp4mA + 0.1;
//					}
//					Change_Conf_temp20mA(tempdata.temp20mA);
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			BTN_TIME++;
			if (BTN_TIME > 100) {
				BTN_TIME = 0;
			}
		}	//end 按下Right键
			//按下Right键
		while (!BTN_MODE()) {
			//长按、短按延时优化
			if (BTN_TIME < 6) {
				HAL_Delay(200);
			} else if (BTN_TIME >= 6 && BTN_TIME < 15)
				HAL_Delay(50);
			else {
				HAL_Delay(3);
			}
			switch (configCurrentPage) {
			case 1:
				switch (CurrentSelect) {
				case 1:
					//修改校正模式
					if (tempdata.caltype == 0) {
						tempdata.caltype = 2;
					} else {
						tempdata.caltype--;
					}
					Change_CalType(tempdata.caltype);
					break;
				case 2:
					//修改大气压单位
					if (tempdata.airpressureunit == 0) {
						tempdata.airpressureunit = 1;
					} else {
						tempdata.airpressureunit = 0;
					}
					Change_AirPressureUnit(tempdata.airpressureunit);
					Change_AirPressure(tempdata.cell);
					break;
				case 3:
					//修改大气压
					tempdata.cell -= 0.001;
					if (tempdata.cell < 0.3) {
						tempdata.cell = 1.5;
					}
					Change_AirPressure(tempdata.cell);
					break;
				default:
					break;
				}
				break;
			case 2:
				switch (CurrentSelect) {
				case 1:
					break;
				case 2:
					break;
				case 3:
					if ((tempdata.caltype == 0) || (tempdata.caltype == 2)) {
						f_PPM_Cal = f_PPM_Cal - 0.01;
						if (f_PPM_Cal < 0)
							f_PPM_Cal = 0;
						Change_Cal_PPM_Fixed(f_PPM_Cal);
						if (f_Rs + tempdata.bdo != 0)
							tempdata.kdo = f_PPM_Cal / (f_Rs + tempdata.bdo);
					}
					if (tempdata.caltype == 1) {
						f_PPM_Cal = f_PPM_Cal - 0.01;
						if (f_PPM_Cal < 0)
							f_PPM_Cal = 0;
						Change_Cal_PPM_Fixed(f_PPM_Cal);
						if (tempdata.kdo != 0)
							tempdata.bdo = (f_PPM_Cal / tempdata.kdo) - f_Rs;
					}
					break;
				case 4:
//					tempdata.temp20mA -= 0.1;
//					if (tempdata.temp20mA < 0) {
//						tempdata.temp20mA = 50;
//					}
//					Change_Conf_temp20mA(tempdata.temp20mA);
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			BTN_TIME++;
			if (BTN_TIME > 100) {
				BTN_TIME = 100;
			}
		}
		//按下Enter键，保存当前值并选中下一条
		if (!BTN_ENTER()) {
			HAL_Delay(100);
			if (!BTN_ENTER()) {
				CurrentSelect++;
				//每页4个设置，超出跳转下一页，超出页数跳回首页
				if (CurrentSelect > 4) {
					CurrentSelect = 1;
					configCurrentPage++;
					if (configCurrentPage > 2) {
						if (result >= 10) {
							memcpy(&savedata, &tempdata, sizeof(savedata));
							writeConfig();
						}
						calFlag = 0;
					}
				}
				Select_Next(CurrentSelect);
				while (!BTN_ENTER())
					;
			}
		}

		//按下Cal键，返回主界面
		if (!BTN_CAL()) {
			HAL_Delay(100);
			if (!BTN_CAL()) {
				calFlag = 0;
				while (!BTN_CAL())
					;
			}
		}
	}
	/* 退出校准后恢复主界面 */
	LCD_Init();
}

/**
 * @功能简介 : History界面函数
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void History_UI(void) {
	uint8_t configCurrentPage = 1, configLastPage = 0, CurrentSelect = 1;
	uint8_t BTN_TIME = 0;
	memcpy(&tempdata, &savedata, sizeof(savedata));
	while (historyFlag) {
		BTN_TIME = 0;
		if (configCurrentPage == 1 && configLastPage != 1) {
			Enter_History_Page1();
			configLastPage = 1;
			//屏蔽刚进入设置按键未释放
			while (!BTN_RIGHT())
				;
			while (!BTN_MODE())
				;
		}

		//按下Conf键，返回主界面
		if (!BTN_CONFIG()) {
			HAL_Delay(100);
			if (!BTN_CONFIG()) {
				historyFlag = 0;
				while (!BTN_CONFIG())
					;
			}
		}
		//按下Cal键，返回主界面
		if (!BTN_CAL()) {
			HAL_Delay(100);
			if (!BTN_CAL()) {
				historyFlag = 0;
				while (!BTN_CAL())
					;
			}
		}
		//按下Enter键，返回主界面
		if (!BTN_ENTER()) {
			HAL_Delay(100);
			if (!BTN_ENTER()) {
				historyFlag = 0;
				while (!BTN_ENTER())
					;
			}
		}
	}
	/* 退出设置后恢复主界面 */
	LCD_Init();
}
void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
//EE 68 00 64 00 64 00 C8 00 64 FF FC FF FF
	uint8_t drawLine[14] = { 0xEE, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFC, 0xFF, 0xFF };
	drawLine[3] = x1;
	drawLine[2] = x1 >> 8;
	drawLine[5] = y1;
	drawLine[4] = y1 >> 8;
	drawLine[7] = x2;
	drawLine[6] = x2 >> 8;
	drawLine[9] = y2;
	drawLine[8] = y2 >> 8;
	HAL_UART_Transmit(&huart1, drawLine, 14, USARTSENDTIME);
}

/**
 * @功能简介 : 串口空闲中断服务函数
 * @入口参数 : uartHandle：串口号
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void UART_RxIDLECallback(UART_HandleTypeDef *uartHandle) {
	if (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET) {

		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		HAL_UART_DMAStop(&huart2);

		unsigned char buffer[8];
		unsigned short crc;
		float temp;
		crc = CRC16(sensorRevBuf, 23);
		//  buffer[6]=crc&0xff;
		//  buffer[7]=(crc&0xff00)>>8;
		//  modscan 是校验码高位在前，低位在后
		buffer[7] = crc & 0xff;
		buffer[6] = (crc & 0xff00) >> 8;

		// 判断地址与crc校验
		if ((sensorRevBuf[0] == 0x03)
				&& (sensorRevBuf[1] == 0x03)     //读取命令
				&& (sensorRevBuf[23] == buffer[6])
				&& (sensorRevBuf[24] == buffer[7])) { // 成功后组合数据 计算 CRC
			// 获取数据
			change_float_big_485rom(5);
			change_float_big_485rom(13);
			change_float_big_485rom(17);
			memcpy((&f_Rs), &sensorRevBuf[5], 4);
			memcpy((&f_Temp), &sensorRevBuf[13], 4);
			memcpy((&f_k), &sensorRevBuf[17], 4);
			result++;
			if (result > 40) {
				result = 0;
			}
//			if (f_Rs > 20) {
//				f_Rs = 20;
//			}
//			if (f_Rs < 0) {
//				f_Rs = 0;
//			}
			f_Rs_fixed[filterCNT] = savedata.kdo * (100 - savedata.tempfactor)
					* (f_Rs + savedata.bdo) / 100;
			if (f_Rs_fixed[filterCNT] > 20) {
				f_Rs_fixed[filterCNT] = 20;
			}
			if (f_Rs_fixed[filterCNT] < 0) {
				f_Rs_fixed[filterCNT] = 0;
			}
			filterCNT++;
			if (f_Temp > 99) {
				f_Temp = 99;
			}
			if (f_Temp < 0) {
				f_Temp = 0;
			}
			f_Temp_fixed = f_Temp * savedata.ktemp;
			if (filterCNT > savedata.filter) {
				uint8_t i;
				for (i = 0; i < savedata.filter; i++) {
					for (filterCNT = 0; filterCNT < savedata.filter - i;
							filterCNT++) {
						if (f_Rs_fixed[filterCNT] > f_Rs_fixed[filterCNT + 1]) {
							temp = f_Rs_fixed[filterCNT + 1];
							f_Rs_fixed[filterCNT + 1] = f_Rs_fixed[filterCNT];
							f_Rs_fixed[filterCNT] = temp;
						}
					}
				}
				f_Rs_filter = f_Rs_fixed[savedata.filter / 2];
				filterCNT = 0;
				/* 刷新标志位 */
				refreshFlag = 1;
			}
		}
		memset(sensorRevBuf, 0xFF, sizeof(sensorRevBuf));
		if (HAL_UART_Receive_DMA(&huart2, (uint8_t*) sensorRevBuf,
				sizeof(sensorRevBuf)) != HAL_OK) {
//			Error_Handler();
		}
		/* 开启串口空闲中断 */
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	}
}

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen) {
	unsigned char uchCRCHi = 0xFF; /* 高CRC字节初始化 */
	unsigned char uchCRCLo = 0xFF; /* 低CRC 字节初始化 */
	unsigned uIndex; /* CRC循环中的索引 */
	while (usDataLen--) /* 传输消息缓冲区 */
	{
		uIndex = uchCRCHi ^ *puchMsg++; /* 计算CRC */
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}
void change_float_big_485rom(unsigned int j)  //修改浮点数在 rom 中的存储大小端
{
	char temp_c = 0;
	temp_c = sensorRevBuf[j + 3];
	sensorRevBuf[j + 3] = sensorRevBuf[j + 0];
	sensorRevBuf[j + 0] = temp_c;

	temp_c = sensorRevBuf[j + 2];
	sensorRevBuf[j + 2] = sensorRevBuf[j + 1];
	sensorRevBuf[j + 1] = temp_c;
}
/**
 * @功能简介 : 外部中断服务函数
 * @入口参数 : GPIO_Pin：引脚
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	/* 不用清除标志位,hal库已清除 */
//	/* 延时去抖动 */
//	HAL_Delay(20);
//	/* BTN_Config */
//	if (GPIO_Pin == GPIO_PIN_8) {
//		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET) {
//			configFlag = 1;
//		}
//	}
//	/* BTN_Cal */
//	else if (GPIO_Pin == GPIO_PIN_0) {
//
//	}
//	/* BTN_Mode */
//	else if (GPIO_Pin == GPIO_PIN_4) {
//
//	}
//	/* BTN_Right */
//	else if (GPIO_Pin == GPIO_PIN_6) {
//
//	}
//	/* BTN_Enter */
//	else if (GPIO_Pin == GPIO_PIN_14) {
//		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
//			LCD_Init();
//		}
//	}
//	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
//}
/**
 * @功能简介 : 按钮查询服务函数
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void Button_Scan(void) {
	if (!BTN_MODE()) {
		if (!BTN_CAL()) {
			Button_Cal_Flag++;
			Button_Conf_Flag = 0;
			if (Button_Cal_Flag > 500) {
				//进入校准
				calFlag = 1;
				Button_Conf_Flag = 0;
				Button_Cal_Flag = 0;
				Button_Right_Flag = 0;
				Cal_UI();
			}
		} else if (!BTN_CONFIG()) {
			Button_Conf_Flag++;
			Button_Cal_Flag = 0;
			if (Button_Conf_Flag > 500) {
				//跳转设置界面
				configFlag = 1;
				Button_Conf_Flag = 0;
				Button_Cal_Flag = 0;
				Button_Right_Flag = 0;
				Conf_UI();
			}
		} else if (!BTN_RIGHT()) {
			Button_Right_Flag++;
			Button_Cal_Flag = 0;
			Button_Conf_Flag = 0;
			if (Button_Right_Flag > 500) {
				//跳转历史界面
				historyFlag = 1;
				Button_Conf_Flag = 0;
				Button_Cal_Flag = 0;
				Button_Right_Flag = 0;
				History_UI();
			}
		}
		//只按下Mode键，不计时
		else {
			Button_Conf_Flag = 0;
			Button_Cal_Flag = 0;
			Button_Right_Flag = 0;
		}
	}
	if (!BTN_ENTER()) {
		if (!BTN_CAL()) {
			Button_Cal_Flag++;
			Button_Conf_Flag = 0;
			if (Button_Cal_Flag > 100000) {
				//恢复出厂校准值
				Button_Conf_Flag = 0;
				Button_Cal_Flag = 0;
				Button_Right_Flag = 0;
				factoryConfig(2);
				LCD_Init();
			}
		} else if (!BTN_CONFIG()) {
			Button_Conf_Flag++;
			Button_Cal_Flag = 0;
			if (Button_Conf_Flag > 100000) {
				//恢复出厂设置值
				Button_Conf_Flag = 0;
				Button_Cal_Flag = 0;
				Button_Right_Flag = 0;
				factoryConfig(1);
				LCD_Init();
			}
		}
		//只按下Mode键，不计时
		else {
			Button_Conf_Flag = 0;
			Button_Cal_Flag = 0;
			Button_Right_Flag = 0;
		}
	}
}
/**
 * @功能简介 : 读取历史数据
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void eepromReadSetting(void) {
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_Delay(1000);
	SPI_Flash_WAKEUP();
	SPI_FLASH_BufferRead((uint8_t*) &SPI_Flashed, 1, 1);
	SPI_FLASH_BufferRead((uint8_t*) &historyCNT, 10, 4);
	SPI_FLASH_BufferRead((uint8_t*) &historyStart, 20, 4);
	SPI_FLASH_BufferRead((uint8_t*) &historyEnd, 30, 4);
	SPI_FLASH_BufferRead((uint8_t*) &History_PPM, 40, 1600);
	SPI_FLASH_BufferRead((uint8_t*) &History_DATE, 1700, 1600);
	SPI_FLASH_BufferRead((uint8_t*) &History_TIME, 3400, 1200);
	SPI_Flash_PowerDown();
	HAL_Delay(1000);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}/* End eepromReadSetting() */

/**
 * @功能简介 : 写入历史数据
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void eepromWriteSetting(void) {
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
//	HAL_Delay(50);
	SPI_Flash_WAKEUP();
	SPI_FLASH_SectorErase(0);
	SPI_FLASH_SectorErase(4096);
	SPI_FLASH_BufferWrite((uint8_t*) &SPI_Flashed, 1, 1);
	SPI_FLASH_BufferWrite((uint8_t*) &historyCNT, 10, 4);
	SPI_FLASH_BufferWrite((uint8_t*) &historyStart, 20, 4);
	SPI_FLASH_BufferWrite((uint8_t*) &historyEnd, 30, 4);
	SPI_FLASH_BufferWrite((uint8_t*) &History_PPM, 40, 1600);
	SPI_FLASH_BufferWrite((uint8_t*) &History_DATE, 1700, 1600);
	SPI_FLASH_BufferWrite((uint8_t*) &History_TIME, 3400, 1200);
	SPI_Flash_PowerDown();
//	HAL_Delay(50);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}/* End eepromWriteSetting() */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
