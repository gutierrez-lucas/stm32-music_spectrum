
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "event_groups.h"

// printf
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "fft.h"

#include "task_utils.h"

#include "../display/ssd1306.h"
#include "../led_matrix/led_matrix.h"

extern uint8_t rgbw_arr[NUM_OF_LEDS * BYTES_PER_LED * 8 + 1];//every pixel colour info is 24 bytes long

void process_task(void *pvParameters);
void display_task(void *pvParameters);
void ledmatrix_task(void *pvParameters);

xTaskHandle process_task_handle = NULL;
xTaskHandle display_task_handle = NULL;
xTaskHandle ledmatrix_task_handle = NULL;

UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_tim2_ch1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
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
QueueHandle_t led_matrix_queue;

uint16_t adc_buffer[BUFFER_SIZE];
bool adc_conversion_done = false;

struct cmpx complex_samples[BUFFER_SIZE];

const uint8_t adc_to_point[409] = {1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,7,7,8,8,8,8,8,8,9,9,9,9,9,9,10,10,10,10,10,10,10,11,11,11,11,11,11,12,12,12,12,12,12,12,13,13,13,13,13,13,14,14,14,14,14,14,15,15,15,15,15,15,15,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,19,19,20,20,20,20,20,20,20,21,21,21,21,21,21,22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,25,25,25,25,25,25,25,26,26,26,26,26,26,27,27,27,27,27,27,28,28,28,28,28,28,28,29,29,29,29,29,29,30,30,30,30,30,30,30,31,31,31,31,31,31,32,32,32,32,32,32,33,33,33,33,33,33,33,34,34,34,34,34,34,35,35,35,35,35,35,35,36,36,36,36,36,36,37,37,37,37,37,37,38,38,38,38,38,38,38,39,39,39,39,39,39,40,40,40,40,40,40,40,41,41,41,41,41,41,42,42,42,42,42,42,43,43,43,43,43,43,43,44,44,44,44,44,44,45,45,45,45,45,45,46,46,46,46,46,46,46,47,47,47,47,47,47,48,48,48,48,48,48,48,49,49,49,49,49,49,50,50,50,50,50,50,51,51,51,51,51,51,51,52,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,54,55,55,55,55,55,55,56,56,56,56,56,56,56,57,57,57,57,57,57,58,58,58,58,58,58,58,59,59,59,59,59,59,60,60,60,60,60,60,61,61,61,61,61,61,61,62,62,62,62,62,62,63,63,63,63,63,63};
const uint8_t adc_to_matrix_point[409] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7};

void print_lut(void){
	printf("----\r\n");
	double aux;
	for(uint16_t i=0; i<409; i++){
		aux = i*8/409;
		printf("%d,", (uint8_t)aux);
	}
	printf("-----\r\n");
}

int main(void){

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();

	printf("\r\n\r\nSpectrum Analyzer\r\n");

	printf("Creating display task: ");
	xTaskCreate(display_task, "display_task", 500, NULL, tskIDLE_PRIORITY+1, &display_task_handle) != pdPASS ? printf(" ERR\r\n") : printf(" OK\r\n");

	printf("Creating process task: ");
	xTaskCreate(process_task, "process_task", 128, NULL, tskIDLE_PRIORITY+2, &process_task_handle) != pdPASS ? printf(" ERR\r\n") : printf(" OK\r\n");

	printf("Creating led matrix task: ");
	xTaskCreate(ledmatrix_task, "led_task", 200, NULL, tskIDLE_PRIORITY+1, &ledmatrix_task_handle) != pdPASS ? printf(" ERR\r\n") : printf(" OK\r\n");

	vTaskSetApplicationTaskTag( display_task_handle, ( void * ) DISPLAY_TASK_TAG );
	vTaskSetApplicationTaskTag( process_task_handle, ( void * ) PROCESS_TASK_TAG );
	vTaskSetApplicationTaskTag( ledmatrix_task_handle, ( void * ) LEDMATRIX_TASK_TAG );

	vTaskStartScheduler();

	while (1){
	}
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	htim2.Instance->CCR1 = 0;
}

uint8_t lock = 0;

bool ledmatrix_init(){
	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);

	return true;
}

void ledmatrix_task(void *pvParameters){
	printf("Led matrix task\r\n");

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

void display_rst(void){
	HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(100));
	HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(1000));
}

bool display_connect(){
	HAL_StatusTypeDef res = SSD1306_Init(0x78);
	if( res != HAL_OK){
		printf("Display connection err: %d\r\n", res);
		return false;
	}else{
		printf("Display connected.\r\n" );
		SSD1306_Clear();
		SSD1306_Putc("Spectrum Analyzer", &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}
	return true;
}

bool display_init(){
	HAL_ADC_Start_DMA(&hadc1, (uint16_t*)adc_buffer, BUFFER_SIZE); //Link DMA to ADC1
	HAL_TIM_Base_Start(&htim3);
	return true;
}

void display_task(void *pvParameters){

	uint32_t notification_message = 0;
	char str_buff[10];
	bool res = false;
	uint16_t display_buffer;

	printf("Display task\r\n");

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	display_queue = xQueueCreate(BUFFER_SIZE, sizeof(uint16_t));
	if(display_queue == NULL){ printf("DISPLAY queue err\r\n"); } 

	display_init();

	while(1){
		if(res == false){

			display_rst();
			res = display_connect();
		}else{
			SSD1306_Clear();
			for(uint8_t i = 0; i < 128; i++){
				xQueueReceive(display_queue, &display_buffer, pdMS_TO_TICKS(1));
				SSD1306_DrawLine(i, 64, i, 64-display_buffer, 1);
			}

			SSD1306_GotoXY(80,10);
			sprintf(str_buff, "%4d", (uint16_t)notification_message);
			SSD1306_Puts(str_buff, &Font_7x10, 1);
			SSD1306_UpdateScreen();
		}
		xTaskNotifyGive(ledmatrix_task_handle);
		// ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		xTaskNotifyWait(0, 0, &notification_message, portMAX_DELAY);
	}
	printf("Destroying Display\r\n");
	vTaskDelete(display_task_handle);
}

void process_task(void *pvParameters){

	printf("PRINT task\r\n");

	char str_buff[10];
	uint16_t display_value;

	uint32_t auxiliar, aux1;
	uint16_t max_amplitude = 0;
	uint8_t max_index = 0;

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while(1){
		if( adc_conversion_done == true){
			adc_conversion_done = false;
			for(int i = 0; i < BUFFER_SIZE; i++){
				complex_samples[i].real = (float)adc_buffer[i];
				complex_samples[i].imag = 0;
			}
			FFT(&complex_samples, BUFFER_SIZE);

			max_index = 0;
			max_amplitude = 0;

			uint8_t aux_counter = 0, aux2_counter=0;
			uint32_t led_matrix_acum = 0;
			for(int i = 1; i <= BUFFER_SIZE/2; i++){
// #define CONFIG_USE_MATH_ABS
#ifdef CONFIG_USE_MATH_ABS
				auxiliar = (uint16_t)sqrt((uint16_t)complex_samples[i].real*(uint16_t)complex_samples[i].real + (uint16_t)complex_samples[i].imag*(uint16_t)(uint16_t)complex_samples[i].imag);
#else
				auxiliar = (uint32_t)(complex_samples[i].real)<<1;
				aux1 = (uint32_t)(complex_samples[i].imag)<<1;
				auxiliar = (auxiliar+aux1)>>1;
#endif
				if(auxiliar > max_amplitude){
					max_amplitude = auxiliar;
					max_index = i;
				}
				display_value = adc_to_point[auxiliar/100];
				xQueueSend(display_queue, (uint16_t*)&display_value, 0);

				if(aux2_counter < 8){
					led_matrix_acum += auxiliar;
					aux_counter++;
					if(aux_counter == 4){
						led_matrix_acum /= 400;
						if(led_matrix_acum > 408){
							led_matrix_acum = 8;
						}else{
							led_matrix_acum = adc_to_matrix_point[led_matrix_acum];
						}
						xQueueSend(led_matrix_queue, (uint8_t*)&led_matrix_acum, 0);
						led_matrix_acum = 0;
						aux_counter = 0;
						aux2_counter++;
					}
				}

			}
			xTaskNotify(display_task_handle, (uint32_t)(max_index*FFT_BAND_RESOLUTION), eSetValueWithOverwrite);
		}
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
		// vTaskDelay(pdMS_TO_TICKS(500));
	}
	printf("Destroying print task \r\n");
	vTaskDelete(process_task_handle);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	adc_conversion_done = true;
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

static void MX_ADC1_Init(void){

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK){
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
		Error_Handler();
	}
}


static void MX_DMA_Init(void){
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK){
		Error_Handler();
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK){
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK){
		Error_Handler();
	}
}

static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK){
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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = display_rst_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
