
#include "main.h"
#include "stm32f1xx_hal.h"
#include "math.h"
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

	#define address 0xD0	//dir
	#define RAD_A_DEG		57.2957
	#define A_R 16384.0//Se supone que da 16 bits en cada uno de los ejes, en la hoja de datos, divide esos 32768/2g esto es por default
	#define G_R 131.0		//32768/250
	uint8_t i, i2cBuf[8];
	int16_t AcX, AcY, AcZ, GyX, GyY,GyZ;
	float Acc[3]; 	//Angulo X --->[0]  Angulo Y---->[1]  Angulo Z---->[2]
	float Gy[3];
	char dataOut[100];
	float ADC_Data;
	
	float Angulo[3];				//Ángulo final
	float dt;
	long tiempo_Ant;
	
	float pid_p=0,pid_i=0,pid_d=0,PID=0,error=0,errorPrev=0;
	float kp;
	float ki;
	float kd;
	
	//ADC Interrupt------------------
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
		if(__HAL_ADC_GET_FLAG(hadc,ADC_FLAG_EOC)){
			ADC_Data=HAL_ADC_GetValue(hadc);
		}
	}
	//--------------------------------
int main(void)

{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	HAL_ADC_Start_IT(&hadc1);
	//kp=0.0244140625;		//Valor máximo de la cte. dado por el ADC = 100.
	//kp=kp*ADC_Data;
	//ki=ADC_Data[1]*0.003662109375;	//Valor máximo de la cte. dado por el ADC = 15.
	//kd=ADC_Data[2]*0.018310546875;	//Valor máximo de la cte. dado por el ADC = 75.
 
// //Saber dirección del dispositivo
// for(i=0;i<255;i++){
//		if(HAL_I2C_IsDeviceReady(&hi2c1,i,1,10)==HAL_OK){
//				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//			break;	
//		}
// }
 //DIRECCION ES i = 

	i2cBuf[0]=0x6B;	//POWER del MPU 6B->0 para reinicializar el sensor
	i2cBuf[1]=0x00;
 
 HAL_I2C_Master_Transmit(&hi2c2,address,i2cBuf,2,10);
 //Prueba de lectura.................
	i2cBuf[0]=28;	//POWER del MPU 6B->0 para reinicializar el sensor
	i2cBuf[1]=0x08;		//8g 16g 32g
	HAL_I2C_Master_Transmit(&hi2c2,address,i2cBuf,2,10);
 
 //leemos el sensor
 
 HAL_I2C_Master_Transmit(&hi2c2,address,i2cBuf,1,10);
 i2cBuf[1]=0;		//Si hay que borrarla par ver que si tiene un 8, para probar
 HAL_I2C_Master_Receive(&hi2c2,address,&i2cBuf[1],1,100); //en i2cBuf[1] tiene que estar los 8 bits que transmiti 
 //..................................
  while (1)
  {
		//Se escribe la dirección de los registros a leer, en este caso 3B para aceleraciones en los tres ejes
		i2cBuf[0]=0x3B;
		HAL_I2C_Master_Transmit(&hi2c2,address,i2cBuf,1,10);
		
		HAL_I2C_Master_Receive(&hi2c2,address,&i2cBuf[1],6,100); 
		AcX= i2cBuf[1]<<8|i2cBuf[2];					//Concatenación de bits, van a ser 16 bits, se declaran arriba
		AcY= i2cBuf[3]<<8|i2cBuf[4];
		AcZ= i2cBuf[5]<<8|i2cBuf[6];
		
		Acc[0]=atan(-1*(AcX/A_R)/sqrt(pow(AcY/A_R,2)+pow(AcZ/A_R,2)))*RAD_A_DEG;
		Acc[1]=atan((AcY/A_R)/sqrt(pow(AcX/A_R,2)+pow(AcZ/A_R,2)))*RAD_A_DEG;
		
		i2cBuf[0]=0x43;			//Para leer los valores del giroscopio en X Y Z
		HAL_I2C_Master_Transmit(&hi2c2,address,i2cBuf,1,10);
		HAL_I2C_Master_Receive(&hi2c2,address,&i2cBuf[1],6,10); 
		
		GyX= i2cBuf[1]<<8|i2cBuf[2];					//Concatenación de bits, van a ser 16 bits, se declaran arriba
		GyY= i2cBuf[3]<<8|i2cBuf[4];
		GyZ= i2cBuf[5]<<8|i2cBuf[6];
		Gy[0]=GyX/G_R;												//Recupero valores, esos valores están entre 250 y los guardamos aquí
		Gy[1]=GyY/G_R;												//para que me de grado por segundo directamente
		Gy[2]=GyZ/G_R;
		//Se va a crear una variable como la de ACC pero para el gyroscopio, se tiene que hacer para sacar el giro lo de una integral. 
		//*******************************Obtención del ángulo**************************************************************************
		//Se utiliza el Systick 
		dt=(HAL_GetTick()-tiempo_Ant)/1000;		//Te va a dar la cuenta en la que va, un registro que va a estar contando. Entre 1000 para que nos de en milisegundos
		tiempo_Ant=HAL_GetTick();						//Ya con esto se obtiene un diferencial, para calcular un ángulo, creamos un diferencial discreto
																				//Lleva paréntesis
//*********************Filtro complementario: calcula el ángulo con base al Acc y el Gy de cada eje
		Angulo[0]=0.98*(Angulo[0]+Gy[0]*dt)+ 0.02*Acc[0];				//Esta es la integral solita para el ángulo. Ángulo  del gyroscopio + angulo del acc = total
		Angulo[1]=0.98*(Angulo[1]+Gy[1]*dt)+ 0.02*Acc[0];
		Angulo[2]=Angulo[2]+Gy[2]*dt;
		
		//**Acc--->ang x aceleracion;
		//Angulo---->accel+gyro

		error=Angulo[0];
		if(error<65){
		pid_p = error*kp;
		if(-4 < error < 4){pid_i = pid_i+(error*ki);}
		pid_d = kd*((error-errorPrev)/dt);
		PID=pid_p+pid_i+pid_d;
		
		if(PID<-600){PID=-600;}
		if(PID>600){PID=600;}
		
		if(PID<0){//Ángulo negativo, se especifíca la dirección de los motores hacia la izquierda
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
			PID=PID*(-1);
		}else{//Ángulo positivo, se especifíca la dirección de los motores hacia la derecha.
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
		}
		
		TIM1->CCR1 = PID;
		}

		//HAL_ADC_Start(&hadc1);
		kp=0.0244140625*ADC_Data;
		sprintf(dataOut,"Angulo:%f	PID:%f	ADC:%f	Kp:%f\n",Angulo[0],PID,ADC_Data,kp);
		HAL_UART_Transmit(&huart2, dataOut,strlen(dataOut), 100);
		errorPrev=error;
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
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
