/**
 ******************************************************************************
 * 文件名程: bsp_rtc.c
 * 作    者: 硬石嵌入式开发团队
 * 版    本: V1.0
 * 编写日期: 2015-10-04
 * 功    能: 板载调试串口底层驱动程序：默认使用USART1
 ******************************************************************************
 * 说明：
 * 本例程配套硬石stm32开发板YS-F1Pro使用。
 *
 * 淘宝：
 * 论坛：http://www.ing10bbs.com
 * 版权归硬石嵌入式开发团队所有，请勿商用。
 ******************************************************************************
 */

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_rtc.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
 * 函数功能: 从串口调试助手获取数字值(把ASCII码转换为数字)
 * 输入参数: value 用户在超级终端中输入的数值
 * 返 回 值: 输入字符的ASCII码对应的数值
 * 说    明：本函数专用于RTC获取时间，若进行其它输入应用，要修改一下
 */
//uint8_t USART_Scanf(uint32_t value)
//{
//  uint32_t index = 0;
//  uint32_t tmp[2] = {0, 0};
//  while (index < 2)
//  {
//    /* 等待直到串口接收到数据 */
//    tmp[index++] =getchar();
//    if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))   /*数字0到9的ASCII码为0x30至0x39*/
//    {
//      printf("请输入 0 到 9 之间的数字 -->:\n");
//      index--;
//    }
//  }
//  /* 计算输入字符的ASCII码转换为数字*/
//  index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
//
//  /* 检查数据有效性 */
//  if (index > value)
//  {
//    printf("请输入 0 到 %d 之间的数字\n", value);
//    return 0xFF;
//  }
//  return index;
//}
/**
 * 函数功能: 配置当前时间和日期
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 */
static void RTC_CalendarConfig(void) {
	extern RTC_TimeTypeDef sTime;
	extern RTC_DateTypeDef DateToUpdate;

	/* 配置日期 */
	/* 设置日期：2015年10月4日 星期日 */
	DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
	DateToUpdate.Month = RTC_MONTH_OCTOBER;
	DateToUpdate.Date = 0x1;
	DateToUpdate.Year = 0x19;
	HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD);

	/* 配置时间 */
	/* 时钟时间：10:50:00 */
	sTime.Hours = 0x00;
	sTime.Minutes = 0x00;
	sTime.Seconds = 0x00;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);


	/* 写入一个数值：0x32F1到RTC备份数据寄存器1 */
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F1);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, DateToUpdate.Year);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, DateToUpdate.Month);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, DateToUpdate.Date);
}

/**
 * 函数功能: RTC实时时钟初始化
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 */
//void MX_RTC_Init(void)
//{
//  /* 初始化RTC实时时钟并设置时间和日期 */
//  hrtc.Instance = RTC;
//  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
//  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
//  HAL_RTC_Init(&hrtc);
//
//#if 1
//  /* 配置RTC万年历：时间和日期 */
//  RTC_CalendarConfig();
//#else
//  /* 检测数据是否保存在RTC备份寄存器1：如果已经保存就不需要运行日期和时间设置 */
//  /* 读取备份寄存器1数据 */
//  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F1)
//  {
//    /* 配置RTC万年历：时间和日期 */
//    RTC_CalendarConfig();
//  }
//  else
//  {
//    /* 检查上电复位标志位是否为：SET */
//    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
//    {
//      printf("发生上电复位！！！\n");
//    }
//    /* 检测引脚复位标志位是否为：SET */
//    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
//    {
//      printf("发生外部引脚复位！！！\n");
//    }
//    /* 清楚复位源标志位 */
//    __HAL_RCC_CLEAR_RESET_FLAGS();
//  }
//#endif
//}
/**
 * 函数功能: RTC实时时钟初始化
 * 输入参数: hrtc：RTC外设句柄指针
 * 返 回 值: 无
 * 说    明: 该函数供HAL库内部函数调用
 */
//void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
//{
//  RCC_OscInitTypeDef        RCC_OscInitStruct;
//  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
//
//  if(hrtc->Instance==RTC)
//  {
//    /* To change the source clock of the RTC feature (LSE, LSI), You have to:
//       - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
//       - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
//         configure the RTC clock source (to be done once after reset).
//       - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
//         __HAL_RCC_BACKUPRESET_RELEASE().
//       - Configure the needed RTc clock source */
//    /* 使能PWR(电源管理外设)时钟和使能获取备份域 */
//    __HAL_RCC_PWR_CLK_ENABLE();
//    HAL_PWR_EnableBkUpAccess();
//
//    /* 使能备份时钟：备份寄存器 */
//    __HAL_RCC_BKP_CLK_ENABLE();
//
//    /* 配置外部低速时钟为RTC时钟源 */
//    RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//    HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
//    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
//
//    /* 使能RTC外设时钟 */
//    __HAL_RCC_RTC_ENABLE();
//  }
//
//}
/**
 * 函数功能: RTC实时时钟反初始化
 * 输入参数: hrtc：RTC外设句柄指针
 * 返 回 值: 无
 * 说    明: 该函数供HAL库内部函数调用
 */
//void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
//{
//
//  if(hrtc->Instance==RTC)
//  {
//    /* 禁用RTC时钟 */
//    __HAL_RCC_RTC_DISABLE();
//
//    /* 禁用PWR时钟和读取备份域 */
//    HAL_PWR_DisableBkUpAccess();
//    __HAL_RCC_PWR_CLK_DISABLE();
//
//    /* 禁用备份时钟：备份寄存器 */
//    __HAL_RCC_BKP_CLK_DISABLE();
//  }
//}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
