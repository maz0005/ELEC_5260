/**
*Marco A. Zuniga
*ELEC 5260: Final Project
*Description: Utilize both the accelerometer and magnetometer on a 
*             STM32L476G Discovery Board using a RTOS OS
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "stm32l476g_discovery.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l476g_discovery_gyroscope.h"
#include "stm32l476g_discovery_compass.h"

enum component{ACCEL = 0, MAGNE = 1};
enum component state = ACCEL;

/*LED related*/
uint8_t led_4_state = 0; 
uint8_t led_5_state = 0;

/*Accelerometer related*/
int16_t accel_average = 0;
osMutexId_t accel_lock;
osMutexId_t read_lock; 

/*Will display either acceleromter or magnetmometer values*/
int16_t* x_display;
int16_t* y_display;
int16_t* z_display;

int16_t* display_value;

int16_t accel_buffer[3] = {0};
int16_t mag_buffer[3] = {0};

uint8_t accel_print[2] = {'A', 'C'};
uint8_t magne_print[2] = {'M', 'A'};

osMessageQueueId_t message_queue;
osEventFlagsId_t LED_flag;

//void app_main(void *argument);
void thread1(void *argument);
void thread2(void *argument);
void thread3(void *argument);
void thread4(void *argument);
void thread5(void *argument);
void thread6(void *argument);
void thread7(void *argument);
uint32_t HAL_GetTick (void);
void SystemClock_Config(void);
void EXTI0_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI9_5_IRQHandler(void);


/*Threads*/
osThreadId_t tid_1;
osThreadId_t tid_2;
osThreadId_t tid_3;
osThreadId_t tid_4;
osThreadId_t tid_5;
osThreadId_t tid_6;
osThreadId_t tid_7;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();
	SystemCoreClockUpdate();  /*Set GLOBAL VARIABLE SystemCoreClock*/
	//if (SysTick_Config(SystemCoreClock/1000)) while(1); /*Catch error*/	
  
	/*Initialize all components*/
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	
	BSP_JOY_Init(JOY_MODE_EXTI);
	BSP_LCD_GLASS_Init();
	
	BSP_COMPASS_Init();
	BSP_GYRO_Init();
	
	osKernelInitialize();

	LED_flag = osEventFlagsNew(NULL);
	accel_lock = osMutexNew(NULL);
	read_lock = osMutexNew(NULL);
	message_queue = osMessageQueueNew(1, sizeof(int16_t), NULL);
	
	/*Instantiate 5 threads*/
	tid_1 = osThreadNew(thread1, NULL, NULL);
	tid_2 = osThreadNew(thread2, NULL, NULL);
	tid_3 = osThreadNew(thread3, NULL, NULL);
	tid_4 = osThreadNew(thread4, NULL, NULL);
	tid_5 = osThreadNew(thread5, NULL, NULL);
	tid_6 = osThreadNew(thread6, NULL, NULL);
	tid_7 = osThreadNew(thread7, NULL, NULL);
		
	osKernelStart(); /*Turn over control to the operating system*/
	
  while (1); /*Should never return here*/

}


/**
* @brief Handle when user presses joystick. START/STOP  
*/
void EXTI0_IRQHandler() {
	for (int i = 0; i < 100; i++) i++; /*Debounce*/

	if (state = ACCEL) {
		display_value = &mag_buffer[0];
		BSP_LCD_GLASS_DisplayChar(&magne_print[0], POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_1); /*Update display to magnetometer*/
		BSP_LCD_GLASS_DisplayChar(&magne_print[1], POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_2);
	}

	else if (state = MAGNE) {
		display_value = &accel_buffer[0];
		BSP_LCD_GLASS_DisplayChar(&accel_print[0], POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_1); /*Update display to accelerometer*/
		BSP_LCD_GLASS_DisplayChar(&accel_print[1], POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_2);
	}


	EXTI->PR1 |= SEL_JOY_PIN;
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
}

/**
*@brief RIGHT. Update to y axis
*/
void EXTI2_IRQHandler(void) { 
	for (int i = 0; i < 100; i++) i++; /*Debounce*/

	if (state = ACCEL) display = &accel_buffer[1];
   /*                      +                        */
	else if (state = MAGNE) display = &magne_buffer[1];

  
	EXTI->PR1 |= RIGHT_JOY_PIN;
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

/**
*	@brief  UP. Update to z axis
*/
void EXTI3_IRQHandler() {
	for (int i = 0; i < 100; i++) i++; /*Debounce*/

	if (state = ACCEL) display = &accel_buffer[2];
   /*                      +                        */
	else if (state = MAGNE) display = &magne_buffer[2];

  
	EXTI->PR1 |= RIGHT_JOY_PIN;
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

/**
* @brief  DOWN. Update x axis
*/
void EXTI9_5_IRQHandler(void) { 
	for (int i = 0; i < 100; i++) i++; /*Debounce*/

	if (state = ACCEL) display = &accel_buffer[0];
   /*                      +                        */
	else if (state = MAGNE) display = &magne_buffer[0];

  
	EXTI->PR1 |= RIGHT_JOY_PIN;
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
}


/**
*@brief Thread 1 of 7
* Blink LED4(red) on and off. Signals thread 2 with event flag when done changing state.
*
*/
void thread1(void *argument) {
	while (1) {
    
		osEventFlagsWait(LED_flag, 0x0001, osFlagsWaitAll, osWaitForever); /*Wait for thread 2*/
		
		if (!led_4_state) {
			BSP_LED_On(LED4);
			led_4_state = 1;
		}
		
		else {
			BSP_LED_Off(LED4);
			led_4_state = 0;
		}
		
		osDelay(1000);
    osEventFlagsSet(LED_flag, 0x0002); /*Signal thread 2*/
		
	}
}

/**
*@brief Thread 2 of 7
*Blink LED5(green) on and off. Signals thread 1 with event flag when done changing state.
*	
*/
void thread2(void *argument) {
	
		while (1) {
      osEventFlagsSet(LED_flag, 0x0001); /*Signal thread 1*/
			osEventFlagsWait(LED_flag, 0x0002, osFlagsWaitAll, osWaitForever); /*Wait for thread 1*/
			
			if (!led_5_state) {
				BSP_LED_On(LED5);
				led_5_state = 1;
			}
		
			else {
				BSP_LED_Off(LED5);
				led_5_state = 0;
			}

			osDelay(1000); 
	}
}

/**
*@brief Thread 3 of 7
*Read accelerometer
*
*/
void thread3(void *argument) {
	while (1) {
		osMutexAcquire(read_lock, osWaitForever); /*Just to assure the magnetometer isn't being read. SAME CHIP*/
		BSP_COMPASS_AccGetXYZ((int16_t *) accel_buffer);
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_3); 
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_4);
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_5); 
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_6);
		osDelay(15);
		osMutexRelease(read_lock);
	}
}

/**
*@brief Thread 4 of 7
* Read magnetometer
*
*/
void thread4(void *argument) {
	while (1) {
		osMutexAcquire(read_lock, osWaitForever); /*Just to assure the accelerometer isn't being read. SAME CHIP*/
		BSP_COMPASS_MagGetXYZ((int16_t *) mag_buffer);
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_3); 
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_4);
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_5); 
		BSP_LCD_GLASS_DisplayChar(, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_6);
		osDelay(15);
		osMutexRelease(read_lock);
	}
	
}


/**
*@brief Thread 5 of 7
* Fill message queue with 'z' acceleration if it's 4 times higher than previous
*
*/
void thread5(void *argument) {
uint16_t last_z = 0;
uint16_t new_z = 0;
	
	while(1) {
		new_z = accel_buffer[2];
		if (((new_z) > (last_z << 2))) { /*Check if new acceleration is 4 times higher than previous*/
			osMessageQueuePut(message_queue, &new_z, 0, osWaitForever); /*Place in buffer*/
		}
		last_z = new_z;
	}
		
}

/**
*@brief Thread 6 of 7
* Will wait until a there has been a drastic change in acceleration in the z axis
* and find the average amongst the last five recordings.
*
*/
void thread6(void *argument) {
uint16_t temp = 0;
uint16_t array_index = 0;
int16_t z_buffer[5] = {0}; /*Hold all samples*/
	
	while(1) {
		
		osMessageQueueGet(message_queue, &z_buffer[array_index], 0, osWaitForever);
		
		if (array_index == 5) array_index = 0;
		else array_index++;
			
		for (int i = 0; i < 5; i++) {
			temp += accel_buffer[i];
		}

		temp = (uint16_t) (temp / 5); /*Take average*/
		
		osMutexAcquire(accel_lock, osWaitForever); /*Wait until thread 7 is done with its computation*/
		accel_average = temp; /*Update accel_average*/
		temp = 0;
		osMutexRelease(accel_lock);
	}
		
}

/**
*@brief Thread 7 of 7
* Take updated average and do something with it
*
*/
void thread7(void *argument) {
	while(1) {
		osMutexAcquire(accel_lock, osWaitForever); /*Wait until thread 6 releases lock*/
		
		osDelay(1000); /*This is to account for a long computation period with variable 'accel_average'. Needs to remain unchanged.*/
		
		osMutexRelease(accel_lock); /*Allow thread 6 to update the accel_average. Done with computation*/
		
	}
		
}



/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}
	
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
