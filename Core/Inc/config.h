#ifndef __CONFIG_H
#define __CONFIG_H

#include "main.h"

extern ADC_HandleTypeDef hadc3;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART3_UART_Init(void);
void MX_USB_OTG_FS_PCD_Init(void);
void MX_DAC_Init(void);
void MX_TIM7_Init(void);
void MX_TIM6_Init(void);
void MX_ADC3_Init(void);

#endif
