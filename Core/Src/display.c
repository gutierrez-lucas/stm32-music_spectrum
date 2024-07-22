#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "display.h"
#include "../display/ssd1306.h"
#include "serial.h"
#include "menu.h"

void display_task(void *pvParameters);
static void MX_I2C1_Init(void);
bool display_connect();
void display_rst(void);

extern xTaskHandle ledmatrix_task_handle;
extern xTaskHandle menu_task_handle;
extern xTaskHandle main_task_handle;
extern xTaskHandle process_task_handle;
xTaskHandle display_task_handle = NULL;

I2C_HandleTypeDef hi2c1;
QueueHandle_t display_queue;

bool my_display_init(){
	MX_I2C1_Init();
	xTaskCreate(display_task, "display_task", 500, NULL, tskIDLE_PRIORITY+1, &display_task_handle) != pdPASS ? printf("DISPLAY: TASK ERR\r\n") : printf("DISPLAY: TASK OK\r\n");
	vTaskSetApplicationTaskTag( display_task_handle, ( void * ) DISPLAY_TASK_TAG );

	display_queue = xQueueCreate(BUFFER_SIZE, sizeof(uint16_t));
	if(display_queue == NULL){ printf("DISPLAY: QUEUE ERR\r\n"); } 

    return true;
}

void display_task(void *pvParameters){

	uint32_t notification_message = 0;
	char str_buff[10];
	bool res = false;
	uint16_t display_buffer;

	notification_union notify;

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while(1){
		xTaskNotifyWait(0, 0, &notify.stream, portMAX_DELAY);
		if(!(notify.configuration &= 0xf0)){
			SSD1306_Clear();
			for(uint8_t i = 0; i < 128; i++){
				xQueueReceive(display_queue, &display_buffer, pdMS_TO_TICKS(1));
				SSD1306_DrawLine(i, 64, i, 64-display_buffer, 1);
			}

			SSD1306_GotoXY(80,10);
			sprintf(str_buff, "%4d", notify.payload);
			SSD1306_Puts(str_buff, &Font_7x10, 1);
		}else{
			SSD1306_GotoXY(0,24);
			sprintf(str_buff, "NO DISP", notify.payload);
			SSD1306_Puts(str_buff, &Font_16x26, 1);
		}
		SSD1306_UpdateScreen();
		xTaskNotify(ledmatrix_task_handle, notify.stream, eSetValueWithOverwrite);
	}
	printf("DISPLAY: TASK END\r\n");
	vTaskDelete(display_task_handle);
}

bool display_init(){
	printf("SSD1306: INIT\r\n");

	uint8_t retry = 1;
	if(retry < 3){
		display_rst();
		if(display_connect()){
			return true;
		}else{
			retry++;
		}
	} 
	return false;
}

void display_rst(void){
	printf("SSD1306: RST\r\n");
	HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(100));
	HAL_GPIO_WritePin(GPIOB, display_rst_pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(1000));
}

bool display_connect(){
	HAL_StatusTypeDef res = SSD1306_Init(0x78);
	if( res != HAL_OK){
		printf("SSD1306: CONN ERR: %d\r\n", res);
		return false;
	}else{
		printf("SSD1306: CONN OK\r\n");
		SSD1306_Clear();
		SSD1306_Putc("Spectrum Analyzer", &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}
	return true;
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

static void led_matrix_DMA(){
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}