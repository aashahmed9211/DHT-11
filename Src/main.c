/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
GPIO_InitTypeDef GPIO_InitStruct;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int retry;
int set = 2;
int flag = 2;
int dat;
int buf[5];
int timer_value[40];
int timer_raw[40];
int temperature;  	    
int humidity; 
int Rh,RhDec,Temp,TempDec,ChkSum;
int tempo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DHT11_delay_us(uint32_t us);
static void MX_GPIO_Init(void);
void PSDHT11_IO_OUT(void);
void PSDHT11_IO_IN(void);
uint8_t PSDHT11_Init(void); //Init the DHT11 module
uint8_t PSDHT11_Check(void);//Check DHT11
void PSDHT11_Rst(void);//Reset DHT11
//uint8_t PSDHT11_Read_Data(int *temp,int *hum); //Read DHT11 Value
uint8_t PSDHT11_Read_Byte(void);//Read One Byte
uint8_t PSDHT11_Read_Bit(void);//Read One Bit
void PSDHT11Read(int *Rh,int *RhDec,int *Temp,int *TempDec, int *ChkSum);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

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
  MX_TIM2_Init();
	MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
	//set =!PSDHT11_Init();
	/* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		set=!PSDHT11_Init();
		PSDHT11Read(&Rh,&RhDec,&Temp,&TempDec,&ChkSum);
		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
		//DHT11_delay_us(9999);
		//set = !PSDHT11_Init();
		//flag = PSDHT11_Read_Data(&temperature,&humidity);
		//DHT11Read(&Rh,&RhDec,&Temp,&TempDec,&ChkSum);	
		HAL_Delay(3000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

	}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 84000000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USER CODE BEGIN 4 */
	void DHT11_delay_us(uint32_t us){
	HAL_TIM_Base_Start(&htim2);
	TIM2->CNT = 0;
	while((TIM2->CNT) <= us);
	HAL_TIM_Base_Stop(&htim2);
}
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
}

void PSDHT11_IO_OUT(void)
{
	
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}
void PSDHT11_IO_IN(void)
{
	//MX_GPIO_Init();
	/*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
uint8_t PSDHT11_Init(void)
{	 		    
	//TM_Delay_Init();
	PSDHT11_Rst();  
	return PSDHT11_Check();
}
void PSDHT11_Rst(void)	   
{                 
		PSDHT11_IO_OUT(); 	//SET OUTPUT
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET); 	//GPIOA.0=0  
		HAL_Delay(20);    	//Pull down Least 18ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET); 	//GPIOA.0=1 
		DHT11_delay_us(20);     	//Pull up 20~40us
		PSDHT11_IO_IN();//SET INPUT	 

}
uint8_t PSDHT11_Check(void) 	   
{   
	retry=0;
  
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==0&&retry<100)//DHT11 Pull down 40~80us
	{
		retry++;
		DHT11_delay_us(1);
	}	 
	if(retry>=100)
	{	
		return 1;
	}
	else 
	{	
		retry=0;
  }	
	//PSDHT11_IO_IN();
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==1&&retry<100)//DHT11 Pull up 40~80us
	{
		retry++;
		DHT11_delay_us(1);
	} 
	if(retry>=100)
		{
			return 1;//check error
		}
	else
	{	
		return 0;
	}
}
//uint8_t PSDHT11_Read_Bit(void) 			 
//{
//	//PSDHT11_IO_IN();//SET INPUT	 
// 	retry=0;
//	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0&&retry<100)//wait become Low level
//	{
//		retry++;
//		DHT11_delay_us(1);
//	}
//	retry=0;
//	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1&&retry<100)//wait become High level
//	{
//		retry++;
//		DHT11_delay_us(1);
//	}
//	DHT11_delay_us(40);//wait 40us
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0)
//	{
//		return 1;
//	}
//		else 
//	{
//		return 0;
//	}		
//}
//uint8_t PSDHT11_Read_Byte(void)    
//{        
//    uint8_t i;
//    dat=0;
//	for (i=0;i<8;i++) 
//	{
//   		dat<<=1; 
//	    dat|=PSDHT11_Read_Bit();
//    }						    
//   return dat;
//		//
//}
//uint8_t PSDHT11_Read_Data(int *temperature,int *humidity)    
//{        
// 	
//	uint8_t i;
//	PSDHT11_Rst();
//	if(PSDHT11_Check()==0)
//	{
//		
//		for(i=0;i<5;i++)
//		{
//			buf[i]=PSDHT11_Read_Byte();
//		}
//		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
//		{
//			*humidity=buf[0];
//			*temperature=buf[2];
//			
//		}
//	}
//	else 
//	{	
//		return 1;
//	}
//	return 0;	    
//}
void PSDHT11Read(int *Rh,int *RhDec,int *Temp,int *TempDec, int *ChkSum)
	{
 
//			uint8_t tempo;
			uint8_t j;
			uint8_t i;
			uint8_t k=0,l=0;
			uint8_t Value[5]={0x00,0x00,0x00,0x00,0x00};
 if(set==1)
 {
 
				for (j = 0; j < 5; ++j) {
				for (i = 0; i < 8; ++i) {
 
					while(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)){} 
					HAL_TIM_Base_Start(&htim2);// start the timer
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)){}
					tempo = __HAL_TIM_GET_COUNTER(&htim2);// take the counter value
						timer_raw[k]=tempo;
						timer_value[l]=(tempo-timer_raw[k-1]);
					HAL_TIM_Base_Stop(&htim2);
					if (timer_value[l]<30) 
						{
						Value[j]=Value[j]<<1;
						}
					else 
						{
						Value[j]=Value[j]<<1;
						Value[j] =Value[j]+1;
						}
						k++;						
						l++;

				}
			}
				*Rh=Value[0];
    		*RhDec=Value[1];
    		*Temp=Value[2];
    		*TempDec=Value[3];
    		*ChkSum=Value[4];	
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
