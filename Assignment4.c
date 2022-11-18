/*******************************************************************************
 * File Name          : <main.c>
 *
 * Description        : <In this program,I have used two GPS strings and parsed it
 *                      with string token function and then analysed it and print
 *                      it on the screen.>
 *
 * Author:              <Kunalsinh Gohil>
 *
 * Date:                <09/12/2018>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// FUNCTION      : printString1
// DESCRIPTION   : It will print the first GPS String.
// PARAMETERS    : term1, term2, term3, term5, term7, term8, term9, term10, term11,
//                 term12, term13, term14, term15, *secondTerm, *thirdTerm, *fourthTerm,
//				   *fifthTerm, *sixthTerm, *seventhTerm, *eighthTerm, *ninthTerm, *tenthTerm,
//				   *eleventhTerm, *twelfthTerm, *thirteenthTerm, *fourteenthTerm,
//				   *fifteenthTerm, *ptr
// RETURNS       : Nothing
void printString1(int term1, int term2, int term3, int term5, int term7,
		int term8, int term9, float term10, int term11, int term12, int term13,
		int term14, long term15, char* secondTerm, char* thirdTerm,
		char* fourthTerm, char* fifthTerm, char* sixthTerm, char* seventhTerm,
		char* eighthTerm, char* ninthTerm, char* tenthTerm, char* eleventhTerm,
		char* twelfthTerm, char* thirteenthTerm, char* fourteenthTerm,
		char* fifteenthTerm, char* ptr) {
	printf("Sentence ID = %s GP = GPS Data, GGA = Time & location data\n\r",
			term1);
	HAL_Delay(100);
	printf("UTC time\n\r");
	HAL_Delay(100);
	int x = term2 / 10000;
	int y = term2 / 100;
	y = y % 100;
	int z = term2 % 100;
	printf("%dhr", x);
	printf("%dmin", y);
	printf("%dsec\n\r", z);
	printf("Latitude\n\r");
	HAL_Delay(100);
	int p = term3 / 100;
	int q = term3 % 100;
	printf("%ddegree", p);
	printf("%dmin\n\r", q);
	printf("Latitude = %s North of the equator\n\r", fourthTerm);
	HAL_Delay(100);
	printf("Longitude\n\r");
	HAL_Delay(100);
	int m = term5 / 100;
	int n = term5 % 100;
	printf("%ddegree", m);
	printf("%dmin\n\r", n);
	printf("Longitude = %s West of Greenwich England\n\r", sixthTerm);
	HAL_Delay(100);
	printf("Position fix = %d\n\r", term7);
	HAL_Delay(100);
	printf("Satellites Used = %d\n\r", term8);
	HAL_Delay(100);
	printf("HDOP = %d\n\r", term9);
	HAL_Delay(100);
	printf("Altitude = %0.4f\n\r", term10);
	HAL_Delay(100);
	printf("Altitude Units = Meters\n\r");
	HAL_Delay(100);
	printf("Geoid separation = %d\n\r", term12);
	HAL_Delay(100);
	printf("in meters = M\n\r");
	HAL_Delay(100);
	printf("DGPS Age = %dsec\n\r", term14);
	HAL_Delay(100);
	printf("Checksum is = %lu\n\r", term15);
	HAL_Delay(100);

}

// FUNCTION      : printString2
// DESCRIPTION   : It will print the second GPS String.
// PARAMETERS    : element2, element3, element5, element7, element8, element9, element10,
//                 element12, element14, element15, elementOne, elementFour, elementSix
// RETURNS       : Nothing
void printString2(unsigned char element2, unsigned char element3,
		unsigned char element5, unsigned char element7, unsigned char element8,
		unsigned char element9, float element10, unsigned char element12,
		unsigned char element14, long element15, char* elementOne,
		char* elementFour, char* elementSix) {
	printf("Sentence ID = %s GP = GPS Data, GGA = Time & location data\n\r",
			elementOne);
	HAL_Delay(100);
	printf("UTC time\n\r");
	HAL_Delay(100);
	int l = element2 / 10000;
	int m = element2 / 100;
	m = m % 100;
	int n = element2 % 100;
	printf("%dhr", l);
	printf("%dmin", m);
	printf("%dsec\n\r", n);
	printf("Latitude\n\r");
	HAL_Delay(100);
	int r = element3 / 100;
	int s = element3 % 100;
	printf("%ddegree", r);
	printf("%dmin\n\r", s);
	printf("Latitude = %s North of the equator\n\r", elementFour);
	HAL_Delay(100);
	printf("Longitude\n\r");
	HAL_Delay(100);
	int g = element5 / 100;
	int h = element5 % 100;
	printf("%ddegree", g);
	printf("%dmin\n\r", h);
	printf("Longitude = %s West of Greenwich England\n\r", elementSix);
	HAL_Delay(100);
	printf("Position fix = %d\n\r", element7);
	HAL_Delay(100);
	printf("Satellites Used = %d\n\r", element8);
	HAL_Delay(100);
	printf("HDOP = %d\n\r", element9);
	HAL_Delay(100);
	printf("Altitude = %0.4f\n\r", element10);
	HAL_Delay(100);
	printf("Altitude Units = Meters\n\r");
	HAL_Delay(100);
	printf("Geoid separation = %d\n\r", element12);
	HAL_Delay(100);
	printf("in meters = M\n\r");
	HAL_Delay(100);
	printf("DGPS Age = %dsec\n\r", element14);
	HAL_Delay(100);
	printf("Checksum is = %lu\n\r", element15);
	HAL_Delay(100);
}

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();

	/* USER CODE BEGIN 2 */
	unsigned char GPSString1[] ="$GPGGA, 014729.10, 4303.5753, N, 08019.0810, W, 1, 6, 1.761,"
					             "214.682, M, 0, M, 0, *5D";
	unsigned char GPSString2[] = "$GPGGA, 092751.000, 5321.6802, N, 00630.3371, W, 1, 8,"
					             "1.03, 61.7, M, 55.3, M, 0, *75";
	const char *delim = ","; //a comma is a delimeter
	const char *delim2 = ",";
	char *ptr1, *ptr2;
	unsigned char term1, term2, element2, term3, element3, term5, element5,
			      term7, element7, term8, element8, term9, element9, term11,
			      element11, term12, element12, term13, element13, term14, element14;
	float term10, element10;
	long term15, element15;

	char* firstTerm = NULL;
	char* secondTerm = NULL;
	char* thirdTerm = NULL;
	char* fourthTerm = NULL;
	char* fifthTerm = NULL;
	char* sixthTerm = NULL;
	char* seventhTerm = NULL;
	char* eighthTerm = NULL;
	char* ninthTerm = NULL;
	char* tenthTerm = NULL;
	char* eleventhTerm = NULL;
	char* twelfthTerm = NULL;
	char* thirteenthTerm = NULL;
	char* fourteenthTerm = NULL;
	char* fifteenthTerm = NULL;

//String token function to parse the string
	firstTerm = strtok(GPSString1, delim);
	secondTerm = strtok(NULL, delim);
	thirdTerm = strtok(NULL, delim);
	fourthTerm = strtok(NULL, delim);
	fifthTerm = strtok(NULL, delim);
	sixthTerm = strtok(NULL, delim);
	seventhTerm = strtok(NULL, delim);
	eighthTerm = strtok(NULL, delim);
	ninthTerm = strtok(NULL, delim);
	tenthTerm = strtok(NULL, delim);
	eleventhTerm = strtok(NULL, delim);
	twelfthTerm = strtok(NULL, delim);
	thirteenthTerm = strtok(NULL, delim);
	fourteenthTerm = strtok(NULL, delim);
	fifteenthTerm = strtok(NULL, delim);

	term2 = atoi(secondTerm); //string argument to integer type
	term3 = atoi(thirdTerm);
	term5 = atoi(fifthTerm);
	term7 = atoi(seventhTerm);
	term8 = atoi(eighthTerm);
	term9 = atoi(ninthTerm);
	term10 = atof(tenthTerm); //string argument to float type
	term11 = atoi(eleventhTerm);
	term12 = atoi(twelfthTerm);
	term13 = atoi(thirteenthTerm);
	term14 = atoi(fourteenthTerm);
	term15 = strtoul(fifteenthTerm, &ptr1, 10); //converts into unsigned long int value

	char* elementOne = NULL;
	char* elementTwo = NULL;
	char* elementThree = NULL;
	char* elementFour = NULL;
	char* elementFive = NULL;
	char* elementSix = NULL;
	char* elementSeven = NULL;
	char* elementEight = NULL;
	char* elementNine = NULL;
	char* elementTen = NULL;
	char* elementEleven = NULL;
	char* elementTwelve = NULL;
	char* elementThirteen = NULL;
	char* elementFourteen = NULL;
	char* elementFifteen = NULL;

	elementOne = strtok(GPSString2, delim2);
	elementTwo = strtok(NULL, delim2);
	elementThree = strtok(NULL, delim2);
	elementFour = strtok(NULL, delim2);
	elementFive = strtok(NULL, delim2);
	elementSix = strtok(NULL, delim2);
	elementSeven = strtok(NULL, delim2);
	elementEight = strtok(NULL, delim2);
	elementNine = strtok(NULL, delim2);
	elementTen = strtok(NULL, delim2);
	elementEleven = strtok(NULL, delim2);
	elementTwelve = strtok(NULL, delim2);
	elementThirteen = strtok(NULL, delim2);
	elementFourteen = strtok(NULL, delim2);
	elementFifteen = strtok(NULL, delim2);

	element2 = atoi(elementTwo);
	element3 = atoi(elementThree);
	element5 = atoi(elementFive);
	element7 = atoi(elementSeven);
	element8 = atoi(elementEight);
	element9 = atoi(elementNine);
	element10 = atof(elementTen);
	element11 = atoi(elementEleven);
	element12 = atoi(elementTwelve);
	element13 = atoi(elementThirteen);
	element14 = atoi(elementFourteen);
	element15 = strtoul(elementFifteen, &ptr2, 10);

	printf("GPS String1 is %s\n\r", GPSString1);
	HAL_Delay(1000);
	printString1(term1, term2, term3, term5, term7, term8, term9, term10,
			term11, term12, term13, term14, term15, secondTerm, thirdTerm,
			fourthTerm, fifthTerm, sixthTerm, seventhTerm, eighthTerm,
			ninthTerm, tenthTerm, eleventhTerm, twelfthTerm, thirteenthTerm,
			fourteenthTerm, fifteenthTerm, ptr1);

	printf("GPS String2 is %s\n\r", GPSString2);
	HAL_Delay(1000);
	printString2(element2, element3, element5, element7, element8, element9,
			element10, element12, element14, element15, elementOne, elementFour,
			elementSix);
	/* USER CODE END 2 */
}
/* USER CODE END 3 */

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
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

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9090;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 4045;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
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

/*****END OF FILE*****/
