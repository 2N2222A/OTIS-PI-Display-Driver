/* USER CODE BEGIN Header */
/**
 *
 * OTIS Position Indicator display driver by JS106351 ©2023.
 *
 * This code was written for the OKI semiconductor MSM5267B-15 VFD driver.  Not tested with
 * the older OTIS displays with the National Semiconductor IC.
 * Feel free to use this code for your own application at your own risk, and if you want to share, please
 * credit the originator of the code.
 * The code was crudely written up to test a PI I had at my work and wanted to do this for some time ago.
 *
 * The clock and data lines have differential signalling so another IC must be used to communicate with the
 * device.  I ran the display at 12 volts although OTIS I believe uses 30V for their controllers and subsystems.
 *
 * The datasheet says that the driver takes 33 bits of segment data and on the 34th clock pulse latches the output,
 * but with my experimentation, I found out it takes 36 bits from OTIS weird clock design
 * with bit 0 as the output data latch, bit 1 does nothing, and the rest from bits 2 to 33 are the segment data.
 *
 *
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define clock(state) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, state)
#define data(state) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, state)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
unsigned char data_register[38];		//buffer to send out to the VFD IC shift register

// Segment to data bit mapping after experimenting around

//Main Segments
// The 'n' segment is considered a dummy segment and is used for blanking or isolating purposes
#define a1 28
#define a2 30
#define b	32
#define c	2
#define d1	4
#define d2	6
#define	 e	8
#define f	26
#define g1	9
#define g2	33
#define n	1

// Intersegments
#define h	27
#define ii	29
#define j	31
#define k	7
#define l	5
#define m	3

//#define ma1	20
//#define ma2	22
//#define mb		24
//#define mc		10
//#define md1	14
//#define md2	12
//#define	 me		16
//#define mf		18
//#define mg1	17
//#define mg2	25


unsigned char count_up = 1;
unsigned char counter = 0;


//original lookup table layout

//unsigned char digit[11][10] = {
//		{a1, a2, b, c, d1, d2, e, f, n, n},		//0
//		{n, n, b, c, n, n, n, n, n, n},			//1
//		{a1, a2, b, n, d1, d2, e, n, g1, g2},	//2
//		{a1, a2, b, c, d1, d2, n, n, g1, g2},	//3
//		{n, n, b, c, n, n, n, f, g1, g2},		//4
//		{a1, a2, n, c, d1, d2, n, f, g1, g2}, 	//5
//		{a1, a2, n, c, d1, d2, e, f, g1, g2},	//6
//		{a1, a2, b, c, n, n, n, n, n, n},		//7
//		{a1, a2, b, c, d1 ,d2, e, f, g1, g2},	//8
//		{a1, a2, b, c, d1, d2, n, f, g1, g2}, 	//9
//		{n, n, n, n, n, n, n, n, n, n}			//blank
//};


unsigned char digit[11][10] = {
		{a1, a2, b, c, d1, d2, e, f, n, n},		//0
		{b, c, n, n, n, n, n, n, n, n},			//1
		{a1, a2, b, d1, d2, e, g1, g2, n, n},	//2
		{a1, a2, b, c, d1, d2, g1, g2, n, n},	//3
		{b, c, f, g1, g2, n, n, n, n, n},		//4
		{a1, a2, c, d1, d2, f, g1, g2, n, n}, 	//5
		{a1, a2, c, d1, d2, e, f, g1, g2, n},	//6
		{a1, a2, b, c, n, n, n, n, n, n},		//7
		{a1, a2, b, c, d1 ,d2, e, f, g1, g2},	//8
		{a1, a2, b, c, d1, d2, f, g1, g2, n}, 	//9
		{n, n, n, n, n, n, n, n, n, n}			//blank
};
//unsigned char mdigit[10][10] = {
//		{ma1, ma2, mb, mc, md1, md2, me, mf, n, n},	//0
//		{n, n, mb, mc, n, n, n, n, n, n},		//1
//		{ma1, ma2, mb, n, md1, md2, me, n, mg1, mg2},	//2
//		{ma1, ma2, mb, mc, md1, md2, n, n, mg1, mg2},		//3
//		{n, n, mb, mc, n, n, n, mf, mg1, mg2},						//4
//		{ma1, ma2, n, mc, md1, md2, n, mf, mg1, mg2}, 		//5
//		{ma1, ma2, n, mc, md1, md2, me, mf, mg1, mg2},		//6
//		{ma1, ma2, mb, mc, n, n, n, n, n, n},						//7
//		{ma1, ma2, mb, mc, md1, md2, me, mf, mg1, mg2},			//8
//		{ma1, ma2, mb, mc, md1, md2, n, mf, mg1, mg2} 		//9
//};

// Initialisation test message buffer, can be changed to show other characters like elevator floors, i.e. SB, B, L, *G, EZ, MZ PH, etc.
// All that needs to be done is reference the offsets in the buffer to show the data.
unsigned char message[15][10] = {
	{n, n, n, n, n, n, n, n, n, n},			//" "
	{a1, a2, b, c, d1, d2, e, f, n, n},		//O
	{a1, a2, ii, l, n, n, n, n, n, n},		//T
	{a1, a2, ii, l, d1, d2, n, n, n, n},	//I
	{a1, a2, n, c, d1, d2, n, f, g1, g2},	//S
	{g1, g2, n, n, n, n, n, n, n, n},		//-
	{a1, a2, b, e, f, g1, g2, n, n, n},		//P
	{a1, a2, ii, l, d1, d2, n, n, n, n},	//I
	{n, n, n, n, n, n, n, n, n, n},			//" "
	{a1, a2, ii, l, n, n, n, n, n, n},		//T
	{a1, a2, d1, d2, e, f, g1, n, n, n},	//E
	{a1, a2, n, c, d1, d2, n, f, g1, g2},	//S
	{a1, a2, ii, l, n, n, n, n, n, n},		//T
	{n, n, n, n, n, n, n, n, n, n},			//" "
	{n, n, n, n, n, n, n, n, n, n},			//" "


};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


void shiftOut(void){

	//Had to add a delay to comply with timing requirements of the VFD driver IC, max frequency is 150kHz.
	for(int i = 0; i < 36; i++){
		data(data_register[i]);
		HAL_Delay(1);
		clock(1);
		HAL_Delay(5);
		clock(0);
		HAL_Delay(5);
	}
}

// Zero out the data register (except bit 0 which is the data latch) before updating it with new segment data information
void refresh(void){
	for(int i = 1; i < 38; i++){
	  data_register[i] = 0;
	}
}

void init(void){

	// For some odd reason if these sequences are not followed when initialising the display,
	// the display keeps outputting garbage

	data_register[0] = 1;
    for(int i = 1; i < 36; i++){		//38
    	data_register[i] = 0;
    }

    data_register[0] = 1;
    data(1);
    clock(1);
    HAL_Delay(20);
    clock(0);
    HAL_Delay(20);

    for(int i = 0; i < 36; i++){		//60
    	data(0);
    	HAL_Delay(1);
    	clock(1);
    	HAL_Delay(5);
    	clock(0);
    	HAL_Delay(5);
    }

    for(int i = 1; i < 36; i++){	//38
    	data_register[i] = 0;
    }

    clock(0);
    data(0);
    for(int i = 1; i < 36; i++){
    	data(data_register[i]);
    	HAL_Delay(1);
    	clock(1);
    	HAL_Delay(5);
    	clock(0);
    	HAL_Delay(5);
    }

}

void displayDigit(unsigned char number){

	unsigned char number_mod = (number / 10);		//extract most significant digit

	for(int i = 0; i < 10; i++){
		data_register[digit[number % 10][i]] = 1;	//extract and show least significant digit
	}
	if(number_mod < 1){
		for(int i = 0; i < 10; i++){
			data_register[digit[10][i]] = 1;		//blank the MSD if number is less than 10
		}
	}else{
		for(int i = 0; i < 10; i++){				//show most significant digit
			if(digit[number_mod][i] >= 26){
				data_register[digit[number_mod][i] - 8] = 1;
			}else if(digit[number_mod][i] == 1){
				data_register[digit[number_mod][i]] = 0;	//Used to prevent a stuck on segment based on offsets
			}else if(digit[number_mod][i] <= 25){
				data_register[digit[number_mod][i] + 8] = 1;
			}
		}
	}
}



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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start_IT(&htim4);

  refresh();

  init();

  __asm__("nop");

  // write scrolling message on display
  for(int nn = 0; nn < 14; nn++){

	  for(int i = 1; i < 38; i++){
		  data_register[i] = 0;
	  }

	  for(int i = 0; i < 10; i++){
		  data_register[message[nn + 1][i]] = 1;
	  }

	  for(int i = 0; i < 10; i++){

		  if(message[nn][i] >= 26){
			  data_register[message[nn][i] - 8] = 1;
		  }else if(message[nn][i] == 1){
			  data_register[message[nn][i]] = 0;
		  }else if(message[nn][i] < 25){
			  data_register[message[nn][i] + 8] = 1;
		  }

	  }
	  shiftOut();
	  HAL_Delay(100);

}

  refresh();
  shiftOut();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(count_up == 1){
		  counter++;
	  }else{
		  counter--;
	  }

	  if(counter > 98){
		  count_up = 0;
	  }else if(counter < 1){
		  count_up = 1;
	  }

	  refresh();

	  displayDigit(counter);

	  shiftOut();

	  HAL_Delay(200);
	  //}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 48000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
