/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "string.h"
#include "calibration.h"
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes =
		{ .name = "SensorTask", .stack_size = 1024 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal, };
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = { .name = "ControlTask",
		.stack_size = 1024 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = { .name = "DebugTask", .stack_size =
		2048 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for dataMutex */
osMutexId_t dataMutexHandle;
const osMutexAttr_t dataMutex_attributes = { .name = "dataMutex" };
/* Definitions for ledMutex */
osMutexId_t ledMutexHandle;
const osMutexAttr_t ledMutex_attributes = { .name = "ledMutex" };
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = { .name = "uartMutex" };
/* USER CODE BEGIN PV */
SensorData_t g_sensorData;
FlashCalibrationBlock_t g_flashCalib;
volatile uint8_t g_debug_enabled = 0;	//debug mode ki-be kapcsolására
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartSensorTask(void *argument);
void StartControlTask(void *argument);
void StartDebugTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {

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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	App_Calibration_Init();
	HAL_UART_Transmit(&huart2, (uint8_t*) "START\r\n", 7, 100);	//csak debug miatt, induláskor egyetlen üzenet
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of dataMutex */
	dataMutexHandle = osMutexNew(&dataMutex_attributes);

	/* creation of ledMutex */
	ledMutexHandle = osMutexNew(&ledMutex_attributes);

	/* creation of uartMutex */
	uartMutexHandle = osMutexNew(&uartMutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of SensorTask */
	SensorTaskHandle = osThreadNew(StartSensorTask, NULL,
			&SensorTask_attributes);

	/* creation of ControlTask */
	ControlTaskHandle = osThreadNew(StartControlTask, NULL,
			&ControlTask_attributes);

	/* creation of DebugTask */
	DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_HALF_Pin | LED_FULL_Pin | CHARGE_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MOTION_Pin */
	GPIO_InitStruct.Pin = MOTION_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MOTION_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_HALF_Pin LED_FULL_Pin CHARGE_Pin */
	GPIO_InitStruct.Pin = LED_HALF_Pin | LED_FULL_Pin | CHARGE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/*Configure GPIO pins : LIGHT_Pin BATTERY_Pin */
	GPIO_InitStruct.Pin = LIGHT_Pin | BATTERY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief  Function implementing the SensorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument) {
	/* USER CODE BEGIN 5 */
	// Nyers ADC értékek ide kerülnek beolvasáskor
	uint16_t raw_light = 0;
	uint16_t raw_battery = 0;

	// Kalibrált (interpolált) értékek ide kerülnek
	float calibrated_light = 0.0f;
	float calibrated_battery = 0.0f;
	// ADC csatorna konfigurációs struktúra (halhoz tartozik)
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/* Infinite loop */
	for (;;) {
		// 1.: FÉNYÉRZÉKELŐ ADC (ADC_CHANNEL_6)
		sConfig.Channel = ADC_CHANNEL_6;      // ADC bemenet kiválasztása (első)
		sConfig.Rank = 1; // első a konverziós sorban (de az önálló mérések miatt mindíg rank1)
		sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; // hosszú mintavételi idő, stabilabb

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK) {
			HAL_ADC_Start(&hadc1);  // mérés indítása
			if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
				raw_light = HAL_ADC_GetValue(&hadc1); // nyers ADC érték kiolvasása
			} else {
				cli_printf("SensorTask: light ADC conversion timeout!\r\n");
				raw_light = 0;
			}
			HAL_ADC_Stop(&hadc1); // konverzió leállítása
		} else {
			cli_printf("SensorTask: light ADC config error!\r\n");
			raw_light = 0;
		}

		// 2.: AKKUFESZÜLTSÉG ADC (ADC_CHANNEL_7)
		sConfig.Channel = ADC_CHANNEL_7;        // második ADC bemenet
		sConfig.Rank = 1;                       // önálló mérések, mindig rank 1
		sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK) {
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
				raw_battery = HAL_ADC_GetValue(&hadc1);
			} else {
				cli_printf("SensorTask: battery ADC conversion timeout!\r\n");
				raw_battery = 0;
			}
			HAL_ADC_Stop(&hadc1);
		}
		else {
			cli_printf("SensorTask: battery ADC config error!\r\n");
			raw_battery = 0;
		}

		// 3.: KALIBRÁCIÓ – interpolált értékek kiszámítása a calibration.c-ben
		//így garantáltan nem marad érvénytelen vagy értelmezhetetlen érték a mérés után!
		if (!CalibrateValue(&g_flashCalib.light, raw_light,
				&calibrated_light)) {
			calibrated_light = 0.0f;
		}
		if (!CalibrateValue(&g_flashCalib.battery, raw_battery,
				&calibrated_battery)) {
			calibrated_battery = 0.0f;
		}
		// Ha a kalibrációs adatok nem érvényesek, az eredmény 0 marad

		// 4.: ADATOK MENTÉSE A GLOBÁLIS STRUKTÚRÁBA (mutex védelem mellett)
		if (osMutexAcquire(dataMutexHandle, 100) == osOK) {
			// Kritikus szakasz – csak 1 task férhet hozzá egyszerre
			g_sensorData.lightLevel = calibrated_light;
			g_sensorData.batteryVoltage = calibrated_battery;
			g_sensorData.raw_light = raw_light;
			g_sensorData.raw_battery = raw_battery;
			osMutexRelease(dataMutexHandle);
		}
		if (g_debug_enabled) {
					cli_printf("SensorTask: raw_light=%u, raw_battery=%u\r\n",
							raw_light, raw_battery);
				}
		osDelay(500);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief Function implementing the ControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument) {
	/* USER CODE BEGIN StartControlTask */
	//feltételezés: fényérzékelő 0-3.3V; akkumulátor 3.2-4.2V (standard 3.7V-os cella)
	{
		float light = 0.0f;
		float voltage = 0.0f;
		GPIO_PinState motion = GPIO_PIN_RESET;
		uint8_t led_half = 0, led_full = 0, charging = 0;

		uint8_t was_motion = 0;
		uint32_t full_brightness_timer = 0; // ms-ben számlál, de osDelay(500) miatt ciklusszámlálóként használjuk

		/* Infinite loop */
		for (;;) {
			// 1. Szenzoradatok mutex-el
			if (osMutexAcquire(dataMutexHandle, 100) == osOK) {
				light = g_sensorData.lightLevel;
				voltage = g_sensorData.batteryVoltage;
				osMutexRelease(dataMutexHandle);
			}

			// 2. Mozgásérzékelő olvasása
			motion = HAL_GPIO_ReadPin(MOTION_GPIO_Port, MOTION_Pin);

			// 3. Logika: alapértelmezés (mindent kikapcsol)
			led_half = 0;
			led_full = 0;
			charging = 0;

			// 4. Akku < 15% → minden kikapcsol
			if (voltage < 3.35f) {	//3.2+0.15=3.35
				// Nincs világítás, töltés se
				led_half = 0;
				led_full = 0;
				charging = 0;

				if (g_debug_enabled) {
					cli_printf("Battery depleted! Everything off.\r\n");
				}
			}
			// 5. Fény > 50% (1,65V) → töltés bekapcsol, LED-ek off
			else if (light > 1.65f) {	//3.3*0,5=1.65
				if (voltage < 4.15f) { // csak 95% battery alatt (3.2+0.95=4.15)
					charging = 1;
					led_half = 0;
					led_full = 0;

					if (g_debug_enabled) {
						cli_printf("Bright. Charging on.\r\n");
					}

				} else {
					// Akku már elérte a 95%-ot → töltés leáll
					charging = 0;
					led_half = 0;
					led_full = 0;

					if (g_debug_enabled) {
						cli_printf(
								"Battery has reached 95%%, charging stops.\r\n");
					}
				}
			}
			// 6. Fény < 15% (0,495V) és akku >= 35% (3,55V) → fél fény
			else if ((light < 0.495f) && (voltage >= 3.55f)) {
				// Mozgás kezelés
				if (motion == GPIO_PIN_SET) {
					was_motion = 1;
					full_brightness_timer = 10; // debug miatt rövidebb: 10ciklus=5sec; (25s/0.5s = 50 ciklus a feladat szerint)
				}

				if (was_motion && full_brightness_timer > 0) {
					led_half = 0;
					led_full = 1;
					full_brightness_timer--;
					// Ha új mozgás, időzítő újraindul
				} else {
					led_half = 1;
					led_full = 0;
				}

				if (motion == GPIO_PIN_RESET && full_brightness_timer == 0) {
					was_motion = 0; // visszaáll, ha letelt az időzítő és nincs mozgás
				}
			}
			// 7. Egyéb helyzetek: alapból minden lekapcsol
			else {
				led_half = 0;
				led_full = 0;
				charging = 0;
			}

			// 8. GPIO kimenetek beállítása (mutex védelem!)
			if (osMutexAcquire(ledMutexHandle, 50) == osOK) {
				HAL_GPIO_WritePin(LED_HALF_GPIO_Port, LED_HALF_Pin,
						led_half ? GPIO_PIN_SET : GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_FULL_GPIO_Port, LED_FULL_Pin,
						led_full ? GPIO_PIN_SET : GPIO_PIN_RESET);
				HAL_GPIO_WritePin(CHARGE_GPIO_Port, CHARGE_Pin,
						charging ? GPIO_PIN_SET : GPIO_PIN_RESET);
				osMutexRelease(ledMutexHandle);
			}

			// 9. DEBUG kiírás (opcionális, csak parancsra indul, cli.c-ben "debug")

			if (g_debug_enabled) {
				// Első sor: csak float
				cli_printf("ControlTask: Battery=%.2f V\r\n", voltage);
				cli_printf("ControlTask: Light=%.2f V\r\n", light);

				// Második sor: csak int
				cli_printf(
						"ControlTask: Motion=%d, LED_HALF=%d, LED_FULL=%d, CHARGE=%d\r\n",
						(motion == GPIO_PIN_SET) ? 1 : 0, led_half, led_full,
						charging);
			}
			osDelay(500); // 500 ms periódus (2 Hz)
		}
		/* USER CODE END StartControlTask */
	}
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
 * @brief Function implementing the DebugTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument) {
	/* USER CODE BEGIN StartDebugTask */

	char buffer[64];
	uint8_t idx = 0;
	uint8_t ch = 0;

	//cli_printf("DebugTask started!\r\n");	//debug miatt

	/* Infinite loop */
	for (;;) {
		idx = 0;
		memset(buffer, 0, sizeof(buffer));

		// Karakteres beolvasás
		while (1) {
			if (HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) == HAL_OK) {
				HAL_UART_Transmit(&huart2, &ch, 1, 10); // Echo

				// Sorvégződés CR vagy LF
				if (ch == '\r' || ch == '\n') {
					buffer[idx] = '\0';
					break;
				} else if (idx < sizeof(buffer) - 1) {
					buffer[idx++] = ch;
				}
			}
		}
		//CleanInput(buffer);		//cli-c-ben megírva, helper fgv.
		if (!CLI_ProcessCommand(buffer)) {
			cli_printf("Command processing failed.\r\n");
		}
	}
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM11 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM11) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
