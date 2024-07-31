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

extern uint8_t adc_to_matrix_point[409];
extern uint8_t linear_to_linear_display_y[1000];
extern uint8_t adc_to_point[409];

void process_task(void *pvParameters);

xTaskHandle process_task_handle = NULL;
extern xTaskHandle main_task_handle;
extern xTaskHandle display_task_handle;

static void data_process_DMA();
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

extern QueueHandle_t led_matrix_queue;
extern QueueHandle_t display_queue;

#define AVERAGE_LED_SAMPLES_AMOUNT 3

#define PERIOD_20KHZ 256
#define PERIOD_10KHZ 512
#define PERIOD_5KHZ 1018

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_adc1;
ADC_HandleTypeDef hadc1;
uint16_t adc_buffer[BUFFER_SIZE];
bool adc_conversion_done = false;

float band_resolution = 156.25;

void change_max_freq(uint8_t freq){
	uint16_t preload = 0;

	switch(freq){
		case USE_20K:
			preload = PERIOD_20KHZ;
			band_resolution = 156.25;
			break;
		case USE_10K:
			preload = PERIOD_10KHZ;
			band_resolution = 78.12;
			break;
		case USE_5K:
			preload = PERIOD_5KHZ;
			band_resolution = 39.06;
			break;
		default: printf("Invalid frequency\r\n"); return;
	}
	__HAL_TIM_SET_AUTORELOAD(&htim3, preload);
}

struct cmpx complex_samples[BUFFER_SIZE];

bool data_process_init(){
    data_process_DMA();
	MX_TIM3_Init();
	MX_ADC1_Init();

	xTaskCreate(process_task, "process_task", 250, NULL, tskIDLE_PRIORITY+1, &process_task_handle) != pdPASS ? printf("PROCESS: TASK ERR\r\n") : printf("PROCESS: TASK OK\r\n");
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

	uint32_t amplitude, auxiliar;
	uint16_t max_amplitude = 0;
	uint8_t max_index = 0;
	uint8_t matrix_value = 0;

	notification_union menu_notification;
	load_default_configuration(&menu_notification.section.configuration);

	xTaskNotifyGive(main_task_handle);

	while(1){
		if( adc_conversion_done == true ){
			adc_conversion_done = false;
			for(uint16_t n = 0; n < BUFFER_SIZE; n++){
				complex_samples[n].real = (float)adc_buffer[n];
				complex_samples[n].imag = 0;
			}
			if(check_plot_mode_time(menu_notification.section.configuration) == true){
				for(uint16_t i = 0; i < BUFFER_SIZE; i+=2){
					display_value = adc_to_point[(uint16_t)complex_samples[i].real/10];
					xQueueSend(display_queue, (uint16_t*)&display_value, 0);
				}
			}
			FFT(&complex_samples, BUFFER_SIZE);

			max_index = 0;
			max_amplitude = 0;

			uint8_t average_led_samples = 0, samples_to_ledmatrix=0;
			uint32_t average_led_acum = 0;
			for(uint16_t k = 1; k <= BUFFER_SIZE/2; k++){
// #define CONFIG_USE_MATH_ABS
#ifdef CONFIG_USE_MATH_ABS
				amplitude = (uint16_t)sqrt((uint16_t)complex_samples[k].real*(uint16_t)complex_samples[k].real + (uint16_t)complex_samples[k].imag*(uint16_t)(uint16_t)complex_samples[k].imag);
#else
				amplitude = (uint32_t)(complex_samples[k].real)<<1;
				auxiliar = (uint32_t)(complex_samples[k].imag)<<1;
				amplitude = (amplitude+auxiliar)>>1;
#endif
				if(amplitude > max_amplitude){
					max_amplitude = amplitude;
					max_index = k;
				}
				if(check_plot_mode_freq(menu_notification.section.configuration)){
					if(amplitude > 30000){
						display_value = 63;
					}else{
						display_value = linear_to_linear_display_y[amplitude/30];
					}
					xQueueSend(display_queue, (uint16_t*)&display_value, 0);
				}

				if(samples_to_ledmatrix < 8 && k>1){
					average_led_acum += amplitude;
					average_led_samples++;
					if(average_led_samples == AVERAGE_LED_SAMPLES_AMOUNT){
						average_led_acum /= AVERAGE_LED_SAMPLES_AMOUNT; 
						average_led_acum /= 10;
						if(average_led_acum > 408){ // size of the LUT
							matrix_value = 8;		// max led matrix value
						}else{
							matrix_value = adc_to_matrix_point[average_led_acum];
						}
						xQueueSend(led_matrix_queue, (uint8_t*)&matrix_value, 0);
						average_led_acum = 0;
						average_led_samples = 0;
						samples_to_ledmatrix++;
					}
				}
			}
			menu_notification.section.payload = max_index*band_resolution;
			xTaskNotify(display_task_handle, menu_notification.stream, eSetValueWithOverwrite);
		}
		xTaskNotifyWait(0, 0, &menu_notification, portMAX_DELAY);
	}
	vTaskDelete(process_task_handle);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	trace_toggle(AUXILIAR_TAG_1);
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
	htim3.Init.Period = PERIOD_20KHZ; // 20khz
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
