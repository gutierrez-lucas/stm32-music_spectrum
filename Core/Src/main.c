
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "event_groups.h"

#include "display.h"
#include "data_process.h"
#include "task_utils.h"
#include "serial.h"
#include "fft.h"
#include "menu.h"

#include "../display/ssd1306.h"
#include "../led_matrix/led_matrix.h"

static void led_matrix_DMA();

extern uint8_t rgbw_arr[NUM_OF_LEDS * BYTES_PER_LED * 8 + 1];//every pixel colour info is 24 bytes long

void ledmatrix_task(void *pvParameters);
xTaskHandle ledmatrix_task_handle = NULL;
extern xTaskHandle process_task_handle;
extern xTaskHandle display_task_handle;
extern xTaskHandle menu_task_handle;

void main_task(void *pvParameters);
xTaskHandle main_task_handle = NULL;

extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_tim2_ch1;
TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);

QueueHandle_t led_matrix_queue;

int main(void){

	HAL_Init();
	SystemClock_Config();

	serial_init();
	MX_GPIO_Init();
	MX_DMA_Init();
	// led_matrix_DMA();
	// MX_TIM2_Init();

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
	// xTaskCreate(ledmatrix_task, "led_task", 200, NULL, tskIDLE_PRIORITY+1, &ledmatrix_task_handle) != pdPASS ? printf("LED-M: TASK ERR\r\n") : printf("LED-M: TASK OK\r\n");
	// vTaskSetApplicationTaskTag( ledmatrix_task_handle, ( void * ) LEDMATRIX_TASK_TAG );

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
	printf("MAIN: waiting for process\r\n");
	xTaskNotifyGive(process_task_handle);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	printf("MAIN: process ready\r\n");
	vTaskPrioritySet(process_task_handle, tskIDLE_PRIORITY+2);

	printf("MAIN: TASK END\r\n");
	vTaskDelete(main_task_handle);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	htim2.Instance->CCR1 = 0;
}

bool ledmatrix_init(){
	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);

	return true;
}

void ledmatrix_task(void *pvParameters){

    printf("LED-M: TASK LOCK\r\n");
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    printf("LED-M: TASK UNLOCK");

	uint8_t lock = 0;
		
	led_matrix_queue = xQueueCreate(8, sizeof(uint8_t));
	if(led_matrix_queue == NULL){ printf("LMATRIX queue err\r\n"); } 

	uint8_t matrix_value;

	ledmatrix_init();
	xTaskNotifyGive(display_task_handle);

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(lock++ == 3){
			lock = 0;
			rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
		}
		for(uint8_t i = 1; i <= 8; i++){
			xQueueReceive(led_matrix_queue, &matrix_value, pdMS_TO_TICKS(1));
			matrix_draw_vertical_line(i, 0, matrix_value, ROTATION_0);
		}
		HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &rgbw_arr, sizeof(rgbw_arr));
		xTaskNotifyGive(process_task_handle);
	}

	printf("Destroying LMatrix task\r\n");
	vTaskDelete(ledmatrix_task_handle);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){
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

static void led_matrix_DMA(){
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

static void MX_DMA_Init(void){
	__HAL_RCC_DMA1_CLK_ENABLE();
}

static void MX_TIM2_Init(void){
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 90 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK){
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK){
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK){
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK){
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1 != HAL_OK)) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim2, &sBreakDeadTimeConfig)!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);

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
