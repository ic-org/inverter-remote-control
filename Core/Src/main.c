/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UART controller that sends requests to either adjust or
  *                   collect information about PFC parameters.
  ******************************************************************************
  * @attention
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

typedef struct{
  volatile int16_t Vflycap;
  volatile int16_t Vdcbus;
  volatile int16_t Vbias;
  volatile int16_t Vac_rms;
  volatile int32_t Iac_rms_mA;
  volatile int16_t Pactive;
  volatile uint32_t Fsw;
  volatile int16_t Kc;
} ReadData_Typedef;

ReadData_Typedef InverterOpCondition ={0,0,0,0,0,0,0,120};

typedef struct{
  volatile uint8_t Inverter_Enable;
  volatile uint16_t Vdcbus;
  volatile uint16_t Pactive_G2B;
  volatile uint16_t Pactive_B2G;
} WriteData_Typedef;

WriteData_Typedef InverterAdCondition = {0,400,3500,3500};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 4
#define TX_BUFFER_SIZE 4
#define FUNCTION_NUM 11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[RX_BUFFER_SIZE];
uint8_t TxBuffer[TX_BUFFER_SIZE];

int16_t VbulkSet_8 = 3200;

uint16_t static PFC_Database[100] = {0};
  // 00 - 24  Adjustment 
  // 25 - 49  Information
  // 50 - 74  Error
  // 75 - 99  Error

static const uint8_t FUNCTION_CODE[FUNCTION_NUM] = 
{
  1,  // PFC output voltage adjustment
  2,  // Active power G2B
  3,  // Active power B2G
  26, // flying capacitor voltage
  27, // PFC output voltage
  28, // bias voltage
  29, // AC line peak voltage
  30, // AC line peak current
  31, // AC active power
  32, // boost leg switching period
  33  // Current sense gain
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void FillBuffer(uint8_t *buffer, uint8_t function, int16_t data);
int CompareBuffer(uint8_t *buf1, uint8_t *buf2, uint16_t bufSize);
void CommunicateToNode(uint8_t function, int16_t data);
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
  uint8_t index;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    for (index = 0; index < FUNCTION_NUM; index++)
    {
      if(InverterAdCondition.Inverter_Enable > 0){
        HAL_GPIO_WritePin(INVERTER_ENABLE_GPIO_Port, INVERTER_ENABLE_Pin, GPIO_PIN_SET);
      }
      else{
        HAL_GPIO_WritePin(INVERTER_ENABLE_GPIO_Port, INVERTER_ENABLE_Pin, GPIO_PIN_RESET);
      }
      CommunicateToNode(FUNCTION_CODE[index], PFC_Database[index+1]);
    }    
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|INVERTER_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USART2_TX_Pin USART2_RX_Pin */
  GPIO_InitStruct.Pin = USART2_TX_Pin|USART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin INVERTER_ENABLE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|INVERTER_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Fill buffer with function code and additional data
void FillBuffer(uint8_t *buffer, uint8_t function, int16_t data)
{
  buffer[0] = function;
  buffer[1] = (uint8_t)((data & 0xFF00) >> 8);
  buffer[2] = (uint8_t)(data & 0x00FF);
  buffer[3] = buffer[0]^buffer[1]^buffer[2];  
}

// check if two buffers match in size
int CompareBuffer(uint8_t *buf1, uint8_t *buf2, uint16_t bufSize)
{
  uint16_t i = 0;
  for (i = 0; i < bufSize; i++)
  {
    if (buf1[i] != buf2[i])
    {
      // one of the elements did not match!
      return 0;
    }
  }
  return 1;   // both buffers match!
}

// communicate to UART node
void CommunicateToNode(uint8_t function, int16_t data)
{
  uint8_t parityCheckRX = 0;
  
  FillBuffer(TxBuffer, function, data);
  
    if (HAL_UART_Transmit(&huart1, TxBuffer, TX_BUFFER_SIZE, 5) != HAL_OK)            // 1. send request based on function type 
    {
      //Error_Handler();                                                              // transmission was not successful [HAL_TIMEOUT? HAL_ERROR?]
    }
    if (HAL_UART_Receive(&huart1, RxBuffer, RX_BUFFER_SIZE, 20) != HAL_OK)            // 2. wait for 15 ms until response is received 
    {                                                                                 // in case of failed transmission HAL_UART_Receive returns HAL_ERROR
      //Error_Handler();                                                              // if no response detected - retransmit the same message again
    }                                                                                 // 3. assess received data
    parityCheckRX = RxBuffer[0]^RxBuffer[1]^RxBuffer[2];
    if(parityCheckRX == RxBuffer[3])                                                  // The received message is good
    {
      if ( RxBuffer[0]<= 25)                                                          // adjustment request
      {
        

      }    
      else if (RxBuffer[0] <= 75)                                                     // information request
      {
        PFC_Database[RxBuffer[0]] = (RxBuffer[1] << 8) | RxBuffer[2];                 // store received PFC variable in a coresponding database location                                                                        
      }
      else
      {
        // error handling [function code above 50]
        // additional check could also be carried out for invalid function code

      }
    }
    InverterOpCondition.Vflycap = PFC_Database[26]>>3;
    InverterOpCondition.Vdcbus = PFC_Database[27]>>3;
    InverterOpCondition.Vbias = PFC_Database[28]/140;
    if(PFC_Database[30] < (1<<15)){
      InverterOpCondition.Vac_rms = (PFC_Database[29]*25)/141;
		}
		else{
      InverterOpCondition.Vac_rms = ((PFC_Database[29]-(1<<16))*25)/141;
		}
    if(PFC_Database[30] < (1<<15)){
      InverterOpCondition.Iac_rms_mA = (PFC_Database[30]*1000)/(1.41*(InverterOpCondition.Kc));
    }
		else{
      InverterOpCondition.Iac_rms_mA = ((PFC_Database[30]-(1<<16))*1000)/(1.41*(InverterOpCondition.Kc));
		}
    if(PFC_Database[31] < (1<<15)){             // Data sent by the slave node is both positive and negative but the master can only see the positve number 
      InverterOpCondition.Pactive = PFC_Database[31]>>3;                  // So this step is required to converter unsigned integer to signed integer data
    }
    else{
      InverterOpCondition.Pactive = (PFC_Database[31]-(1<<16))>>3;
    }      
    InverterOpCondition.Fsw = 46e6/PFC_Database[32];
    InverterOpCondition.Kc = PFC_Database[33];
    if(PFC_Database[51] == 255){                                                      // Reset InverterAdCondition variable when receiving the special 51 
      InverterAdCondition.Inverter_Enable = 0;                                        // request from the slave node
      InverterAdCondition.Vdcbus = 400;
      InverterAdCondition.Pactive_B2G = 3500;
      InverterAdCondition.Pactive_G2B = 3500;
      PFC_Database[51] = 0;
    }

    PFC_Database[1] = InverterAdCondition.Vdcbus<<3;
    PFC_Database[2] = InverterAdCondition.Pactive_G2B<<3;
    PFC_Database[3] = InverterAdCondition.Pactive_B2G<<3;
 
}

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
