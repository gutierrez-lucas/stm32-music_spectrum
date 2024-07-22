#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "data_process.h"
#include "menu.h"
#include "serial.h"
#include "fft.h"

void process_task(void *pvParameters);

xTaskHandle process_task_handle = NULL;
extern xTaskHandle main_task_handle;
extern xTaskHandle display_task_handle;

static void data_process_DMA();
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

extern QueueHandle_t led_matrix_queue;
extern QueueHandle_t display_queue;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_adc1;
ADC_HandleTypeDef hadc1;
uint16_t adc_buffer[BUFFER_SIZE];
bool adc_conversion_done = false;
// 10k
// #define FFT_MAX_FREQ_PRELOAD 600 // 10kHz
// #define FFT_BAND_RESOLUTION 78.12 // 128 bands, 10kHz each
// 20k
#define FFT_BAND_RESOLUTION 156.25 // 128 bands, 20kHz each
#define FFT_MAX_FREQ_PRELOAD 300 // 20kHz

struct cmpx complex_samples[BUFFER_SIZE];

const uint8_t adc_to_point[409] = {1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,7,7,8,8,8,8,8,8,9,9,9,9,9,9,10,10,10,10,10,10,10,11,11,11,11,11,11,12,12,12,12,12,12,12,13,13,13,13,13,13,14,14,14,14,14,14,15,15,15,15,15,15,15,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,19,19,20,20,20,20,20,20,20,21,21,21,21,21,21,22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,25,25,25,25,25,25,25,26,26,26,26,26,26,27,27,27,27,27,27,28,28,28,28,28,28,28,29,29,29,29,29,29,30,30,30,30,30,30,30,31,31,31,31,31,31,32,32,32,32,32,32,33,33,33,33,33,33,33,34,34,34,34,34,34,35,35,35,35,35,35,35,36,36,36,36,36,36,37,37,37,37,37,37,38,38,38,38,38,38,38,39,39,39,39,39,39,40,40,40,40,40,40,40,41,41,41,41,41,41,42,42,42,42,42,42,43,43,43,43,43,43,43,44,44,44,44,44,44,45,45,45,45,45,45,46,46,46,46,46,46,46,47,47,47,47,47,47,48,48,48,48,48,48,48,49,49,49,49,49,49,50,50,50,50,50,50,51,51,51,51,51,51,51,52,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,54,55,55,55,55,55,55,56,56,56,56,56,56,56,57,57,57,57,57,57,58,58,58,58,58,58,58,59,59,59,59,59,59,60,60,60,60,60,60,61,61,61,61,61,61,61,62,62,62,62,62,62,63,63,63,63,63,63};
const uint8_t adc_to_matrix_point[409] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7};

bool data_process_init(){
    data_process_DMA();
	MX_TIM3_Init();
	MX_ADC1_Init();

	xTaskCreate(process_task, "process_task", 128, NULL, tskIDLE_PRIORITY+1, &process_task_handle) != pdPASS ? printf("PROCESS: TASK ERR\r\n") : printf("PROCESS: TASK OK\r\n");
	vTaskSetApplicationTaskTag( process_task_handle, ( void * ) PROCESS_TASK_TAG );
    return true;
}

void process_task(void *pvParameters){

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	HAL_ADC_Start_DMA(&hadc1, (uint16_t*)adc_buffer, BUFFER_SIZE); //Link DMA to ADC1
	HAL_TIM_Base_Start(&htim3);

	char str_buff[10];
	uint16_t display_value;
	notification_union notify;

	uint32_t auxiliar, aux1;
	uint16_t max_amplitude = 0;
	uint8_t max_index = 0;

	uint32_t menu_notification;
	bool selected_use_matrix = true, selected_use_display = true;
	uint8_t selected_plot = SELECTION_FREQ_PLOT;

	xTaskNotifyGive(main_task_handle);

	while(1){
		if( adc_conversion_done == true ){
			adc_conversion_done = false;
			for(int i = 0; i < BUFFER_SIZE; i++){
				complex_samples[i].real = (float)adc_buffer[i];
				complex_samples[i].imag = 0;
			}
			if(selected_plot == SELECTION_TIME_PLOT){
				for(int i = 0; i < BUFFER_SIZE; i+=2){
					display_value = adc_to_point[(uint16_t)complex_samples[i].real/100];
					xQueueSend(display_queue, (uint16_t*)&display_value, 0);
				}
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
				if(selected_plot == SELECTION_FREQ_PLOT){
					display_value = adc_to_point[auxiliar/100];
					xQueueSend(display_queue, (uint16_t*)&display_value, 0);
				}

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
			if(selected_use_display == false){
				notify.configuration |= 0xf0;
			}else{
				notify.configuration &= ~0xf0;
			}
			if(selected_use_matrix == false){
				notify.configuration |= 0x0f;
			}else{
				notify.configuration &= ~0x0f;
			}
			notify.payload = max_index*FFT_BAND_RESOLUTION;
			xTaskNotify(display_task_handle, notify.stream, eSetValueWithOverwrite);
		}
		xTaskNotifyWait(0, 0, &menu_notification, portMAX_DELAY);
		if(menu_notification != 0){
			if(menu_notification == SELECTION_FREQ_PLOT){
				selected_plot = SELECTION_FREQ_PLOT;
			}else if(menu_notification == SELECTION_TIME_PLOT){
				selected_plot = SELECTION_TIME_PLOT;
			}else if(menu_notification == SELECTION_POWER_PLOT){
				selected_plot = SELECTION_POWER_PLOT;
			}else if(menu_notification == SELECION_MATRIX_OFF){
				selected_use_matrix = false;
			}else if(menu_notification == SELECION_MATRIX_ON){
				selected_use_matrix = true;
			}else if(menu_notification == SELECTION_DISPLAY_OFF){
				selected_use_display = false;
			}else if(menu_notification == SELECTION_DISPLAY_ON){
				selected_use_display = true;
			}
			menu_notification = 0;
		}
	}
	printf("Destroying print task \r\n");
	vTaskDelete(process_task_handle);
}

void print_lut(void){
	printf("----\r\n");
	double aux;
	for(uint16_t i=0; i<409; i++){
		aux = i*8/409;
		printf("%d,", (uint8_t)aux);
	}
	printf("-----\r\n");
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	adc_conversion_done = true;
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

static void data_process_DMA(){
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_TIM3_Init(void){

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