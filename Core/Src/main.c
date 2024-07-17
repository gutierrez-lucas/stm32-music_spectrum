
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"

#include "display.h"
#include "led-matrix.h"
#include "data_process.h"
#include "menu.h"

#include "task_utils.h"
#include "serial.h"

extern xTaskHandle ledmatrix_task_handle;
extern xTaskHandle process_task_handle;
extern xTaskHandle display_task_handle;
extern xTaskHandle menu_task_handle;

void main_task(void *pvParameters);
xTaskHandle main_task_handle = NULL;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

int main(void){

	HAL_Init();
	SystemClock_Config();

	serial_init();
	MX_GPIO_Init();
	MX_DMA_Init();

	printf("\r\n\r\nSpectrum Analyzer\r\n");

	if(my_display_init() == false){
		printf("Display task error\r\n");
	}
	if(data_process_init() == false){
		printf("Data process task error\r\n");
	}
	if(menu_init() == false){
		printf("Menu task error\r\n");
	}
	if(ledmatrix_init() == false){
		printf("Led-Matrix task error\r\n");
	}

	xTaskCreate(main_task, "main_task", 128, NULL, tskIDLE_PRIORITY+1, &main_task_handle) != pdPASS ? printf("MAIN: TASK ERR\r\n") : printf("MAIN: TASK OK\r\n");
	vTaskStartScheduler();

	while (1){
	}
}

void main_task(void *pvParameters){
	printf("MAIN - TASK\r\n");

	if( display_init() == false){
		printf("Display task error\r\n");
	}

	printf("MAIN: display ready\r\n");
	printf("MAIN: waiting for menu\r\n");
	xTaskNotifyGive(menu_task_handle);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	printf("MAIN: menu ready\r\n");
	printf("MAIN: Unlocking display task\r\n");
	xTaskNotifyGive(display_task_handle);
	printf("MAIN: waiting for led-matrix task\r\n");
	xTaskNotifyGive(ledmatrix_task_handle);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	printf("MAIN: waiting for process\r\n");
	xTaskNotifyGive(process_task_handle);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	printf("MAIN: process ready\r\n");
	vTaskPrioritySet(process_task_handle, tskIDLE_PRIORITY+2);

	printf("MAIN: TASK END\r\n");
	vTaskDelete(main_task_handle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM1) { HAL_IncTick(); }
}

void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
															|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
		Error_Handler();
	}
}

static void MX_DMA_Init(void){
	__HAL_RCC_DMA1_CLK_ENABLE();
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = display_rst_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	HAL_GPIO_WritePin(GPIOB, encoder_button_pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = encoder_button_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, trace_1_Pin|trace_2_Pin|trace_3_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = trace_1_Pin|trace_2_Pin|trace_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, trace_4_Pin|trace_5_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = trace_4_Pin|trace_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; // this is for the encoder .. just in case
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
	__disable_irq();
	while (1){	}
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
