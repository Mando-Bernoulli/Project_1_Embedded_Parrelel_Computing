/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME280_ADDR          (0x77 << 1)
#define BME280_I2C_TIMEOUT   100

/* BME280 registers */
#define BME280_REG_CALIB_T   0x88   /* dig_T1..dig_T3 (6 bytes) */
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_TEMP_MSB  0xFA   /* temp_msb, temp_lsb, temp_xlsb */

#define BME280_CHIP_ID       0x60   /* BMP280 would be 0x58 */
#define BME280_RESET_CMD     0xB6
#define TEMP_SETPOINT 26

#define FAN_TEMP_OFF       26
#define FAN_TEMP_MEDIUM    27
#define FAN_TEMP_HIGH      28
#define FAN_TEMP_MAX       29

#define FAN_PWM_MAX        3200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
static uint16_t dig_T1;
static int16_t  dig_T2;
static int16_t  dig_T3;

osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "SensorReadTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "ControlReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t actuatorTaskHandle;
const osThreadAttr_t actuatorTask_attributes = {
  .name = "ActuatorControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


osThreadId_t loggingTaskHandle;
const osThreadAttr_t loggingTask_attributes = {
  .name = "LoggingControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

QueueHandle_t sensorQueue;
QueueHandle_t actuatorQueue;

/* Sprint 5: logging message structure */
typedef struct
{
    char msg[80];
} LogMessage;

QueueHandle_t LogQueue;
SemaphoreHandle_t LogSemaphore;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);


/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef BME280_Init(void);
static HAL_StatusTypeDef BME280_ReadTemp(int32_t *temp_centi);

void SensorReadTask(void *argument);
void ControlReadTask(void *argument);
void ActuatorControlTask(void *argument);

static void Fan_SetSpeed(uint8_t duty_percent);
static void Fan_ControlTemperature(int32_t temp, uint8_t *duty);

void LoggingTask(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Print(const char *str)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

static void Log_Send(const char *message)
{
    LogMessage logMsg;

    snprintf(logMsg.msg, sizeof(logMsg.msg), "%s", message);

    if (xQueueSend(LogQueue, &logMsg, 0) == pdPASS)
    {
        /* Wake the logging task after a successful queue send. */
        xSemaphoreGive(LogSemaphore);
    }
}

/* Check chip ID, reset, load calibration, start continuous temperature conversion */
static HAL_StatusTypeDef BME280_Init(void)
{
	HAL_StatusTypeDef ret;
	uint8_t id = 0;
	uint8_t cal[6];
	uint8_t val;

	ret = HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, BME280_REG_ID,
			I2C_MEMADD_SIZE_8BIT, &id, 1, BME280_I2C_TIMEOUT);
	if (ret != HAL_OK) return ret;
	if (id != BME280_CHIP_ID) return HAL_ERROR;

	/* Soft reset, then wait for NVM copy to finish */
	val = BME280_RESET_CMD;
	ret = HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, BME280_REG_RESET,
			I2C_MEMADD_SIZE_8BIT, &val, 1, BME280_I2C_TIMEOUT);
	if (ret != HAL_OK) return ret;
	osDelay(10);

	/* Temperature calibration: 0x88..0x8D (little endian) */
	ret = HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, BME280_REG_CALIB_T,
			I2C_MEMADD_SIZE_8BIT, cal, 6, BME280_I2C_TIMEOUT);
	if (ret != HAL_OK) return ret;
	dig_T1 = (uint16_t)(cal[1] << 8 | cal[0]);
	dig_T2 = (int16_t) (cal[3] << 8 | cal[2]);
	dig_T3 = (int16_t) (cal[5] << 8 | cal[4]);

	/* ctrl_hum: humidity measurement skipped (must be written before ctrl_meas) */
	val = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, BME280_REG_CTRL_HUM,
			I2C_MEMADD_SIZE_8BIT, &val, 1, BME280_I2C_TIMEOUT);
	if (ret != HAL_OK) return ret;

	/* config: standby 0.5 ms, filter off, I2C only */
	val = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, BME280_REG_CONFIG,
			I2C_MEMADD_SIZE_8BIT, &val, 1, BME280_I2C_TIMEOUT);
	if (ret != HAL_OK) return ret;

	/* ctrl_meas: osrs_t = x1 (001), osrs_p = skip (000), mode = normal (11) */
	val = (0x01 << 5) | (0x00 << 2) | 0x03;
	ret = HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, BME280_REG_CTRL_MEAS,
			I2C_MEMADD_SIZE_8BIT, &val, 1, BME280_I2C_TIMEOUT);
	return ret;
}

/* Reads temperature in hundredths of a degree Celsius (e.g. 2534 = 25.34 C).
   Uses the integer compensation formula from the Bosch datasheet. */
static HAL_StatusTypeDef BME280_ReadTemp(int32_t *temp_centi)
{
	HAL_StatusTypeDef ret;
	uint8_t raw[3];
	int32_t adc_T, var1, var2, t_fine;

	ret = HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, BME280_REG_TEMP_MSB,
			I2C_MEMADD_SIZE_8BIT, raw, 3, BME280_I2C_TIMEOUT);
	if (ret != HAL_OK) return ret;

	/* 20-bit raw value: msb[7:0] lsb[7:0] xlsb[7:4] */
	adc_T = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
	if (adc_T == 0x80000) return HAL_ERROR;   /* measurement skipped / not ready */

	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
	          ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;

	*temp_centi = (t_fine * 5 + 128) >> 8;

	return HAL_OK;
}


static void Fan_ControlTemperature(int32_t temp, uint8_t *duty)
{
    if (temp < FAN_TEMP_OFF)
    {
        *duty = 0;
    }
    else if (temp < FAN_TEMP_MEDIUM)
    {
        *duty = 25;
    }
    else if (temp < FAN_TEMP_HIGH)
    {
        *duty = 50;
    }
    else if (temp < FAN_TEMP_MAX)
    {
        *duty = 100;
    }
    else
    {
        *duty = 100;
    }
}


static void Fan_SetSpeed(uint8_t duty_percent)
{
    uint32_t pulse;

    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    pulse = ((uint32_t)duty_percent *
             (FAN_PWM_MAX - 1U)) / 100U;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}



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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Start PWM on TIM3 Channel 2 */

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
      Error_Handler();
  }

  /* Fan initially off */
  Fan_SetSpeed(0);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  LogSemaphore = xSemaphoreCreateBinary();

  if (LogSemaphore == NULL)
  {
      Error_Handler();
  }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  sensorQueue = xQueueCreate(5, sizeof(int32_t));
  actuatorQueue = xQueueCreate(5, sizeof(uint8_t));
  LogQueue = xQueueCreate(10, sizeof(LogMessage));

  if (sensorQueue == NULL || actuatorQueue == NULL || LogQueue == NULL)
  {
      UART_Print("Queue creation failed\r\n");
      Error_Handler();
  }

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  char msg[80];

  snprintf(msg, sizeof(msg),
           "Heap before sensor: %lu bytes\r\n",
           (unsigned long)xPortGetFreeHeapSize());
  UART_Print(msg);

  sensorTaskHandle =
      osThreadNew(SensorReadTask, NULL, &sensorTask_attributes);

  snprintf(msg, sizeof(msg),
           "Heap before control: %lu bytes\r\n",
           (unsigned long)xPortGetFreeHeapSize());
  UART_Print(msg);

  controlTaskHandle =
      osThreadNew(ControlReadTask, NULL, &controlTask_attributes);

  snprintf(msg, sizeof(msg),
           "Heap before actuator: %lu bytes\r\n",
           (unsigned long)xPortGetFreeHeapSize());
  UART_Print(msg);

  actuatorTaskHandle =
      osThreadNew(ActuatorControlTask, NULL, &actuatorTask_attributes);

  snprintf(msg, sizeof(msg),
           "Heap before actuator: %lu bytes\r\n",
           (unsigned long)xPortGetFreeHeapSize());
  UART_Print(msg);

  loggingTaskHandle =
      osThreadNew(LoggingTask, NULL, &loggingTask_attributes);


  UART_Print(sensorTaskHandle != NULL
             ? "Sensor creation OK\r\n"
             : "Sensor creation FAILED\r\n");

  UART_Print(controlTaskHandle != NULL
             ? "control creation OK\r\n"
             : "control creation FAILED\r\n");

  UART_Print(actuatorTaskHandle != NULL
             ? "Actuator creation OK\r\n"
             : "Actuator creation FAILED\r\n");

  UART_Print(loggingTaskHandle != NULL
             ? "Logging creation OK\r\n"
             : "Logging creation FAILED\r\n");

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 913;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5: onboard LED */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA7: TIM3_CH2 (fan PWM) */
    /* PA7: TIM3_CH2 - PC fan PWM control */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
void SensorReadTask(void *argument)
{
    int32_t t;
    int32_t a;
    int32_t temp;
    char msg[64];

    while (BME280_Init() != HAL_OK)
    {
        UART_Print("BME280 INIT FAILED\r\n");
        osDelay(1000);
    }

    UART_Print("BME280 INITIALIZED\r\n");

    for (;;)
    {
    	if (BME280_ReadTemp(&t) == HAL_OK)
    	{
    		a = (t < 0) ? -t : t;
    		temp = t / 100;
    		/* Integer formatting: no need for float printf support */
    		snprintf(msg, sizeof(msg), "Temperature: %s%ld.%02ld C\r\n",
    					(t < 0) ? "-" : "", (long)(a / 100), (long)(a % 100));
    	}
    	else
    	{
    		snprintf(msg, sizeof(msg), "BME280 read error\r\n");
    	}
    	Log_Send(msg);


        if (xQueueSend(sensorQueue, &temp, 0) != pdPASS)
        {
            UART_Print("Queue full\r\n");
        }

        osDelay(1000);
    }

}


void ControlReadTask(void *argument)
{
    int32_t temp;
    uint8_t fanDuty;

    for (;;)
    {
        if (xQueueReceive(sensorQueue, &temp,
                          portMAX_DELAY) == pdPASS)
        {
            /* Control the onboard GPIO output */
            if (temp >= TEMP_SETPOINT)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
                                  GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
                                  GPIO_PIN_RESET);
            }

            /* Calculate fan speed from temperature */
            Fan_ControlTemperature(temp, &fanDuty);

            /* Send the requested fan speed to actuator task */
            if (xQueueSend(actuatorQueue, &fanDuty,
                           portMAX_DELAY) != pdPASS)
            {
                UART_Print("Actuator queue send failed\r\n");
            }
        }
    }
}


void ActuatorControlTask(void *argument)
{
    uint8_t fanDuty;
    char msg[48];

    UART_Print("Actuator task started\r\n");

    for (;;)
    {
        if (xQueueReceive(actuatorQueue, &fanDuty,
                          portMAX_DELAY) == pdPASS)
        {
            Fan_SetSpeed(fanDuty);

            snprintf(msg, sizeof(msg),
                     "Fan PWM set to %u%%\r\n",
                     (unsigned int)fanDuty);

            Log_Send(msg);
        }
    }
}

void LoggingTask(void *argument)
{
    LogMessage logMsg;

    (void)argument;

    for (;;)
    {
        /* Wait until a producer signals that a log is available. */
        if (xSemaphoreTake(LogSemaphore, portMAX_DELAY) == pdTRUE)
        {
            /* Read all available messages from the log queue. */
            while (xQueueReceive(LogQueue, &logMsg, 0) == pdPASS)
            {
                UART_Print(logMsg.msg);
                UART_Print("\r\n");
            }
        }
    }
}


/* USER CODE END Header_StartDefaultTask */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  __disable_irq();
	  while (1)
	  {
	  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
