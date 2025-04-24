/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "../SYSTEM/delay/delay.h"
#include "../BSP/EXTI/exti.h" 
#include "../BSP/LED/led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
  int fre = 0 ;
  int a0_amp =0;
  int a1_amp =0;
  int a2_amp =0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  uint16_t spwmcnt=0;
  uint16_t spwmcnt2=133;
  uint16_t spwmcnt3=267;
  int spwm_group[400]={
    33,99,165,231,297,362,428,494,559,624,690,755,819,884,948,1013,1076,
    1140,1203,1266,1329,1392,1454,1515,1577,1638,1698,1758,1818,1877,1936,1994,2052,
    2110,2166,2223,2278,2333,2388,2442,2495,2548,2600,2652,2703,2753,2802,2851,2899,
    2946,2993,3039,3084,3129,3172,3215,3257,3298,3339,3378,3417,3455,3492,3528,3564,
    3598,3632,3664,3696,3727,3757,3786,3814,3841,3868,3893,3917,3940,3963,3984,4005,
    4024,4042,4060,4076,4092,4106,4119,4132,4143,4153,4163,4171,4178,4184,4190,4194,
    4197,4199,4200,4200,4199,4197,4194,4190,4184,4178,4171,4163,4153,4143,4132,4119,
    4106,4092,4076,4060,4042,4024,4005,3984,3963,3940,3917,3893,3868,3841,3814,3786,
    3757,3727,3696,3664,3632,3598,3564,3528,3492,3455,3417,3378,3339,3298,3257,3215,
    3172,3129,3084,3039,2993,2946,2899,2851,2802,2753,2703,2652,2600,2548,2495,2442,
    2388,2333,2278,2223,2166,2110,2052,1994,1936,1877,1818,1758,1698,1638,1577,1515,
    1454,1392,1329,1266,1203,1140,1076,1013,948,884,819,755,690,624,559,494,
    428,362,297,231,165,99,33,-33,-99,-165,-231,-297,-362,-428,-494,-559,
    -624,-690,-755,-819,-884,-948,-1013,-1076,-1140,-1203,-1266,-1329,-1392,-1454,-1515,-1577,
    -1638,-1698,-1758,-1818,-1877,-1936,-1994,-2052,-2110,-2166,-2223,-2278,-2333,-2388,-2442,-2495,
    -2548,-2600,-2652,-2703,-2753,-2802,-2851,-2899,-2946,-2993,-3039,-3084,-3129,-3172,-3215,-3257,
    -3298,-3339,-3378,-3417,-3455,-3492,-3528,-3564,-3598,-3632,-3664,-3696,-3727,-3757,-3786,-3814,
    -3841,-3868,-3893,-3917,-3940,-3963,-3984,-4005,-4024,-4042,-4060,-4076,-4092,-4106,-4119,-4132,
    -4143,-4153,-4163,-4171,-4178,-4184,-4190,-4194,-4197,-4199,-4200,-4200,-4199,-4197,-4194,-4190,
    -4184,-4178,-4171,-4163,-4153,-4143,-4132,-4119,-4106,-4092,-4076,-4060,-4042,-4024,-4005,-3984,
    -3963,-3940,-3917,-3893,-3868,-3841,-3814,-3786,-3757,-3727,-3696,-3664,-3632,-3598,-3564,-3528,
    -3492,-3455,-3417,-3378,-3339,-3298,-3257,-3215,-3172,-3129,-3084,-3039,-2993,-2946,-2899,-2851,
    -2802,-2753,-2703,-2652,-2600,-2548,-2495,-2442,-2388,-2333,-2278,-2223,-2166,-2110,-2052,-1994,
    -1936,-1877,-1818,-1758,-1698,-1638,-1577,-1515,-1454,-1392,-1329,-1266,-1203,-1140,-1076,-1013,
    -948,-884,-819,-755,-690,-624,-559,-494,-428,-362,-297,-231,-165,-99,-33,
  };
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //中断回调函数 20k
{
    if(htim==&htim2)
    {
    TIM1->CCR1 = 4200.f + 0.02*a0_amp*spwm_group[spwmcnt];
    TIM1->CCR2 = 4200.f - 0.02*a0_amp*spwm_group[spwmcnt];
        
    TIM8->CCR1 = 4200.f + 0.02*a1_amp*spwm_group[spwmcnt2];
    TIM8->CCR2 = 4200.f - 0.02*a1_amp*spwm_group[spwmcnt2];
    
    TIM1->CCR3 = 4200.f + 0.02*a2_amp*spwm_group[spwmcnt3];
    TIM8->CCR3 = 4200.f - 0.02*a2_amp*spwm_group[spwmcnt3];
    spwmcnt++;
    spwmcnt2++;
    spwmcnt3++;
    if(spwmcnt==400)spwmcnt=0;
    if(spwmcnt2==400)spwmcnt2=0;
    if(spwmcnt3==400)spwmcnt3=0;
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

    char rx_buf[200];
    char* e_buf="error\r\n";
    int arr;
    char out[200];
    

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  /*
  HAL_TIM_Base_Start_IT(&htim2);//开启定时器2中断
  HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1);//开启四路pwm波
  HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);*/
  //char* tr_buf = "fre\r\n";
  //HAL_UART_Transmit(&huart1,tr_buf,1,0);

   
  // HAL_UART_Transmit(&huart1, (uint8_t *)tr_buf,sizeof(tr_buf),0xFFFF);
 /*if(HAL_UART_Receive(&huart1, (uint8_t *)rx_buf,1,0xFFFF) == HAL_OK)
  {
      LED0_TOGGLE();
      HAL_UART_Transmit(&huart1, (uint8_t *)rx_buf,sizeof(rx_buf),0xFFFF);
  }
  */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if(HAL_UART_Receive(&huart1, (uint8_t *)&rx_buf,sizeof("fre:00,a0:00,a1:00,a2:00\r\n")-1,0xFFFF) == HAL_OK)
    {
     int tens_digit = (int)(rx_buf[4] - '0');
     int ones_digit = (int)(rx_buf[5] - '0');
     fre = (tens_digit * 10) + ones_digit;
        
     int tens_digit_a0 = (int)(rx_buf[10] - '0');
     int ones_digit_a0 = (int)(rx_buf[11] - '0');
     a0_amp=(tens_digit_a0 * 10) + ones_digit_a0;
        
     int tens_digit_a1 = (int)(rx_buf[16] - '0');
     int ones_digit_a1 = (int)(rx_buf[17] - '0');
     a1_amp=(tens_digit_a1 * 10) + ones_digit_a1;
        
     int tens_digit_a2 = (int)(rx_buf[22] - '0');
     int ones_digit_a2 = (int)(rx_buf[23] - '0');
     a2_amp=(tens_digit_a2 * 10) + ones_digit_a2;
     
     //fre = (int)(rx_buf[5]-'0');
     if (fre == 0)
     {
     HAL_TIM_Base_Stop_IT(&htim2); //关闭定时器2
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_1);
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_1);
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_2);
     
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_1);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_1);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_2);

     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_3);
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_3);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_3);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_3);
    break;
    }
     arr =168000000/(400*fre*0.5)-1;
     
     HAL_TIM_Base_Stop_IT(&htim2); //关闭定时器2
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_1);
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_1);
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_2);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_1);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_1);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_2);
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_3);
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_3);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_3);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_3);
        
     TIM2->ARR=arr;
        
     HAL_TIM_Base_Start_IT(&htim2);//开启定时器2中断
     HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1);//开启四路pwm波
     HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);
     HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);
    
     HAL_TIM_PWM_Start (&htim8,TIM_CHANNEL_1);//开启四路pwm波
     HAL_TIMEx_PWMN_Start (&htim8,TIM_CHANNEL_1);
     HAL_TIM_PWM_Start (&htim8,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Start (&htim8,TIM_CHANNEL_2);
     
     HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_3);//开启四路pwm波
     HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_3);
     HAL_TIM_PWM_Start (&htim8,TIM_CHANNEL_3);
     HAL_TIMEx_PWMN_Start (&htim8,TIM_CHANNEL_3);
     
     out[0] = rx_buf[4];
     out[1] = rx_buf[5];
     out[2] = rx_buf[10];
     out[3] = rx_buf[11];
     out[4] = rx_buf[16];
     out[5] = rx_buf[17];
     out[6] = rx_buf[22];
     out[7] = rx_buf[23];
     
     HAL_UART_Transmit(&huart1,(uint8_t *)&out,sizeof(char)*8,0xFFFF);
     memset(rx_buf,0,sizeof("fre:00,a0:00,a1:00,a2:00\r\n")-1);
    }
    else
    {
     HAL_UART_Transmit(&huart1,(uint8_t *)e_buf,sizeof(e_buf),0xFFFF);
     HAL_TIM_Base_Stop_IT(&htim2); //关闭定时器2
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_1);//开启四路pwm波
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_1);
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_2);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_1);//开启四路pwm波
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_1);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_2);
     HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_3);//开启四路pwm波
     HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_3);
     HAL_TIM_PWM_Stop (&htim8,TIM_CHANNEL_3);
     HAL_TIMEx_PWMN_Stop (&htim8,TIM_CHANNEL_3);
     memset(rx_buf,0,sizeof("fre:00,a0:00,a1:00,a2:00\r\n")-1);
    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
