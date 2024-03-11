/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
//#include "math_helper.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
int8_t current_row = -1, current_col = -1;
uint8_t keypad_poll = 0;
uint8_t key_pressed = 0;
uint8_t filter_en = 0;
uint8_t new_sample = 0;

const float32_t sine[SAMPLES] = {
		0,	0.130526192220052,	0.258819045102521,	0.382683432365090,	0.500000000000000,	0.608761429008721,	0.707106781186548,
		0.793353340291235,	0.866025403784439,	0.923879532511287,	0.965925826289068,	0.991444861373810,	1,	0.991444861373811,
		0.965925826289068,	0.923879532511287,	0.866025403784439,	0.793353340291235,	0.707106781186548,	0.608761429008721,
		0.500000000000000,	0.382683432365090,	0.258819045102521,	0.130526192220052,	0,	-0.130526192220051,	-0.258819045102520,
		-0.382683432365089,	-0.499999999999999,	-0.608761429008721,	-0.707106781186548,	-0.793353340291235,	-0.866025403784439,
		-0.923879532511287,	-0.965925826289068,	-0.991444861373810,	-1,	-0.991444861373811,	-0.965925826289068,	-0.923879532511287,
		-0.866025403784439,	-0.793353340291236,	-0.707106781186548,	-0.608761429008721,	-0.500000000000000,	-0.382683432365090,
		-0.258819045102522,	-0.130526192220053
};

const float32_t firCoeffs32[NUM_TAPS] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = SAMPLES/BLOCK_SIZE;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void poll_keypad(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	arm_fir_instance_f32 S;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  float32_t output = 0;
  float index = 0;
//  int32_t incr = 1;
  float incr = 1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  current_row = 0;
  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);

	float32_t  *inputF32;
	inputF32 = &sine[0];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (keypad_poll){
		  keypad_poll = 0;
		  current_row = 0;
	  	  poll_keypad();
	  }

	  if (key_pressed){
		key_pressed = 0;
		if ((current_row - 1 == 2) && (current_col == 1)){
			// double the frequency
			incr = incr*2;
		}else if ((current_row - 1 == 2) && (current_col == 0)){
			// half the frequency (haven't exactly figured out how to go below 1 kHz)
//			if (incr > 1){
//				incr = incr >> 1;
//			}
			incr = incr/2;
		}
	  }

	if (new_sample){
	  new_sample = 0;

	  if (filter_en){
		  arm_fir_f32(&S, inputF32 + (uint32_t)(index)%SAMPLES, &output, blockSize);
	  }else{
		  output = sine[(uint32_t)(index)%SAMPLES];
	  }

	  output *= 1024;
	  output += 2048;
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)output);

	  index += incr;
	}
  }
}

void poll_keypad(void){
	while (!key_pressed && (current_row < 4)) {
		if (current_row == 0) {
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_SET);
			HAL_Delay(0);
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_RESET);
		} else if (current_row == 1) {
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW1_Pin, GPIO_PIN_SET);
			HAL_Delay(0);
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
		} else if (current_row == 2) {
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW2_Pin, GPIO_PIN_SET);
			HAL_Delay(0);
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
		} else if (current_row == 3) {
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW3_Pin, GPIO_PIN_SET);
			HAL_Delay(0);
			HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
		}

		current_row++;
	}
}
