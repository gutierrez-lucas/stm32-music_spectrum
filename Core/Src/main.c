
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
// printf
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "fft.h"
#include "task_utils.h"

#include "../display/ssd1306.h"
#include "../display/test.h"

void process_task(void *pvParameters);
void display_task(void *pvParameters);

xTaskHandle process_task_handle = NULL;
xTaskHandle display_task_handle = NULL;

UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3;
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

int _write(int file, char *data, int len){
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)){
				errno = EBADF;
				return -1;
	}
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);

	return (status == HAL_OK ? len : 0);
}

// 10k
// #define FFT_MAX_FREQ_PRELOAD 600 // 10kHz
// #define FFT_BAND_RESOLUTION 78.12 // 128 bands, 10kHz each
// 20k
#define FFT_BAND_RESOLUTION 156.25 // 128 bands, 20kHz each
#define FFT_MAX_FREQ_PRELOAD 300 // 20kHz

#define BUFFER_SIZE 256

QueueHandle_t display_queue;

uint16_t adc_buffer[BUFFER_SIZE];
bool adc_conversion_done = false;

struct cmpx complex_samples[BUFFER_SIZE];

const uint8_t adc_to_point[409] = {1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,7,7,8,8,8,8,8,8,9,9,9,9,9,9,10,10,10,10,10,10,10,11,11,11,11,11,11,12,12,12,12,12,12,12,13,13,13,13,13,13,14,14,14,14,14,14,15,15,15,15,15,15,15,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,19,19,20,20,20,20,20,20,20,21,21,21,21,21,21,22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,25,25,25,25,25,25,25,26,26,26,26,26,26,27,27,27,27,27,27,28,28,28,28,28,28,28,29,29,29,29,29,29,30,30,30,30,30,30,30,31,31,31,31,31,31,32,32,32,32,32,32,33,33,33,33,33,33,33,34,34,34,34,34,34,35,35,35,35,35,35,35,36,36,36,36,36,36,37,37,37,37,37,37,38,38,38,38,38,38,38,39,39,39,39,39,39,40,40,40,40,40,40,40,41,41,41,41,41,41,42,42,42,42,42,42,43,43,43,43,43,43,43,44,44,44,44,44,44,45,45,45,45,45,45,46,46,46,46,46,46,46,47,47,47,47,47,47,48,48,48,48,48,48,48,49,49,49,49,49,49,50,50,50,50,50,50,51,51,51,51,51,51,51,52,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,54,55,55,55,55,55,55,56,56,56,56,56,56,56,57,57,57,57,57,57,58,58,58,58,58,58,58,59,59,59,59,59,59,60,60,60,60,60,60,61,61,61,61,61,61,61,62,62,62,62,62,62,63,63,63,63,63,63};

int main(void)
{

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();

	printf("\r\n\r\nACD system\r\n");

	// printf("----\r\n");
	// for(uint16_t i=0; i<409; i++){
	// 	printf("%d,", i*64/409);
	// }
	// printf("-----\r\n");

	display_queue = xQueueCreate(BUFFER_SIZE, sizeof(uint16_t));
	if(display_queue == NULL){
		printf("Error creating display queue\r\n");
	}

	HAL_ADC_Start_DMA(&hadc1, (uint16_t*)adc_buffer, BUFFER_SIZE); //Link DMA to ADC1
	HAL_TIM_Base_Start(&htim3);

	if(xTaskCreate(display_task, "display_task", 500, NULL, tskIDLE_PRIORITY+1, &display_task_handle) != pdPASS){
		printf("Error creating display task\r\n");
	}
	if(xTaskCreate(process_task, "process_task", 200, NULL, tskIDLE_PRIORITY+1, &process_task_handle) != pdPASS){
		printf("Error creating process task\r\n");
	}

	vTaskSetApplicationTaskTag( display_task_handle, ( void * ) DISPLAY_TASK_TAG );
	vTaskSetApplicationTaskTag( process_task_handle, ( void * ) PROCESS_TASK_TAG );

	vTaskStartScheduler();

	while (1){
	}
}

UBaseType_t task_watermark;
// uint8_t display_buffer[128];
float max_frequency = 0;
uint16_t max_amplitude = 0;
uint8_t max_index = 0;

void display_task(void *pvParameters){

	printf("Display task\r\n");

	char str_buff[10];
	static int connected = 0;
	uint16_t display_buffer;

	while(1){
		if(connected == 0){
			printf("Reseting display.. \r\n");

			HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_RESET); // reset display
			vTaskDelay(pdMS_TO_TICKS(100));
			HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_SET);

			HAL_StatusTypeDef res = SSD1306_Init(0x78); 
			if( res != HAL_OK){
				printf("Display connection err: %d\r\n", res);
			}else{
				connected = 1;
				printf("Display connected.\r\n" );
				SSD1306_Clear();
				SSD1306_Putc("Connected", &Font_7x10, 1);
				SSD1306_UpdateScreen();
			}
		}else{
			SSD1306_Clear();
			for(uint8_t i = 0; i < 128; i++){
				xQueueReceive(display_queue, &display_buffer, pdMS_TO_TICKS(1));
				SSD1306_DrawLine(i, 64, i, 64-display_buffer, 1);
			}

			SSD1306_GotoXY(80,10);
			sprintf(str_buff, "%4d", (uint16_t)max_frequency);
			SSD1306_Puts(str_buff, &Font_7x10, 1);
			SSD1306_UpdateScreen();
		}
		
		xTaskNotifyGive(process_task_handle);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
	printf("Destroying Display task 1 \r\n");
	vTaskDelete(display_task_handle);
}

void process_task(void *pvParameters){

	printf("PRINT task\r\n");

	char str_buff[10];
	uint16_t auxiliar; 
	uint16_t display_value;

	printf("Waiting for display task .. \r\n");
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	printf("Display connected\r\n");

	while(1){
		if( adc_conversion_done == true){
			adc_conversion_done = false;
			for(int i = 0; i < BUFFER_SIZE; i++){
				complex_samples[i].real = (float)adc_buffer[i];
				complex_samples[i].imag = 0;
			}
			FFT(&complex_samples, BUFFER_SIZE);


			max_frequency = 0;
			max_index = 0;
			max_amplitude = 0;

			for(int i = 1; i <= BUFFER_SIZE/2; i++){
				auxiliar = (uint16_t)sqrt((uint16_t)complex_samples[i].real*(uint16_t)complex_samples[i].real + (uint16_t)complex_samples[i].imag*(uint16_t)complex_samples[i].imag);
				if(auxiliar > max_amplitude){
					max_amplitude = auxiliar;
					max_index = i;
				}

				display_value = adc_to_point[auxiliar/100];
				xQueueSend(display_queue, (uint16_t*)&display_value, 0);
			}
			max_frequency = max_index*FFT_BAND_RESOLUTION;
			// printf("freq: %d | amp: %d | index: %d\r\n", max_frequency, max_amplitude, max_index);
			xTaskNotifyGive(display_task_handle);
		}
		// ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(35));
	}
	printf("Destroying print task \r\n");
	vTaskDelete(process_task_handle);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	// trace_toggle(AUXILIAR_TAG_2);
	adc_conversion_done = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
// 	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	// }else if (htim->Instance == TIM3) {
	}
}

void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}	
}

static void MX_ADC1_Init(void){

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}


static void MX_DMA_Init(void){
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_TIM3_Init(void)
{

	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 6;
	htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim3.Init.Period = FFT_MAX_FREQ_PRELOAD;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
	{
	Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_I2C1_Init(void){
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK){
		Error_Handler();
	}

}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */

	HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = display_rst_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, trace_3_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = trace_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, trace_1_Pin|trace_2_Pin|trace_4_Pin|trace_5_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = trace_1_Pin|trace_2_Pin|trace_4_Pin|trace_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

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
