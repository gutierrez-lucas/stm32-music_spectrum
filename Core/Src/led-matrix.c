#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "main.h"
#include "menu.h"
#include "../led-ws2812/ws2812.h"
#include "led-matrix.h"
#include "serial.h"

TIM_HandleTypeDef htim2;
static void MX_TIM2_Init(void);
DMA_HandleTypeDef hdma_tim2_ch1;

extern xTaskHandle main_task_handle;
extern xTaskHandle menu_task_handle;
extern xTaskHandle process_task_handle;

void ledmatrix_task(void *pvParameters);
xTaskHandle ledmatrix_task_handle = NULL;
void led_matrix_DMA();
QueueHandle_t led_matrix_queue;

extern uint8_t rgbw_arr[NUM_OF_LEDS * BYTES_PER_LED * 8 + 1];//every pixel colour info is 24 bytes long

void ledmatrix_task(void *pvParameters);

bool ledmatrix_init(){

	led_matrix_DMA();
	MX_TIM2_Init();
	xTaskCreate(ledmatrix_task, "led_task", 250, NULL, tskIDLE_PRIORITY+1, &ledmatrix_task_handle) != pdPASS ? printf("LED-M: TASK ERR\r\n") : printf("LED-M: TASK OK\r\n");
	vTaskSetApplicationTaskTag( ledmatrix_task_handle, ( void * ) LEDMATRIX_TASK_TAG );

	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);

	return true;
}

bool rgb_ready = false;

void ledmatrix_task(void *pvParameters){

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	notification_union notify;
	uint8_t lock = 0;
		
	led_matrix_queue = xQueueCreate(8, sizeof(uint8_t));
	if(led_matrix_queue == NULL){ printf("LMATRIX queue err\r\n"); } 

	uint8_t matrix_value;

	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &rgbw_arr, sizeof(rgbw_arr));

	xTaskNotifyGive(main_task_handle);

	while(1){
		xTaskNotifyWait(0, 0, &notify.stream, portMAX_DELAY);
		if(check_use_matrix(notify.section.configuration)){
			if(lock++ == 5){
				rgb_matrix_clear_buffer(rgbw_arr, sizeof(rgbw_arr));
				lock = 0;
			}
			for(uint8_t i = 1; i <= 8; i++){ 
				xQueueReceive(led_matrix_queue, &matrix_value, pdMS_TO_TICKS(1));
				matrix_draw_vertical_line(i, 1, matrix_value, ROTATION_270);
				while( rgb_ready != true ){ vTaskDelay(pdMS_TO_TICKS(1)); }
				// vTaskDelay(pdMS_TO_TICKS(1));
				rgb_ready = false;
				HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, rgbw_arr, sizeof(rgbw_arr));
			}
		}
		xTaskNotifyGive(menu_task_handle);
	}

	vTaskDelete(ledmatrix_task_handle);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	htim2.Instance->CCR1 = 0;
	rgb_ready = true;
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

void led_matrix_DMA(){
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}
