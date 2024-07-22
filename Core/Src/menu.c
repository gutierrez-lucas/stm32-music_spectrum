#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "menu.h"
#include "../display/bitmap.h"
#include "../display/ssd1306.h"
#include "serial.h"

xTaskHandle menu_task_handle = NULL;
extern xTaskHandle main_task_handle;
extern xTaskHandle process_task_handle;

int16_t get_encoder();
void menu_task(void *pvParameters);

static void MX_TIM4_Init(void);
TIM_HandleTypeDef htim4;

////////////
uint8_t current_selected_element = 0, last_selected_element = 99;
uint8_t current_menu = 0;
int16_t idle_encoder_position = 0xff, current_encoder_position = 0;

#define ENCODER_MENU_STEP 4
#define MAX_MENU_ELEMENTS 3
#define FIRST_MENU_ELEMENT 1
////////////
typedef enum action_t{
	UNUSED = 99,
	PRESSED = 1,
	UNPRESSED = 0
}action_t;

int16_t new_encoder_position = 0, last_encoder_position = 0, difference_position = 0;
bool using_menu;
uint8_t menu_id = 0;

#define MAIN_MENU_ID 0
#define PLOT_MENU_ID 10
#define CONF_MENU_ID 20
#define METRICS_MENU_ID 30

#define FONT_HEIGHT 10
#define FONT_WIDTH 7
#define STARTING_LEFT 10
#define STARTING_TOP 5
#define BOX_3_SEPARATOR_HIGH 3
// #define BOX_3_HEIGHT (uint8_t)((SSD1306_HEIGHT-4*BOX_3_SEPARATOR_HIGH)/3)
#define BOX_3_HEIGHT 16 
#define BOX_3_HEIGHT_DISTANCE_NEXT (BOX_3_HEIGHT+BOX_3_SEPARATOR_HIGH)
#define BOX_1_WIDTH (SSD1306_WIDTH-STARTING_LEFT)
#define STRING_3_TOP (STARTING_TOP+4)
#define STRING_3_LEFT (STARTING_LEFT+3)

bool menu_init(){
	MX_TIM4_Init();
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	xTaskCreate(menu_task, "menu_task", 250, NULL, tskIDLE_PRIORITY+1, &menu_task_handle) != pdPASS ? printf("MENU: TASK ERR\r\n") : printf("MENU: TASK OK\r\n");
	vTaskSetApplicationTaskTag( menu_task_handle, ( void * ) MENU_TASK_TAG );
    return true;
}

void return_to_main_menu(){
	SSD1306_Clear();
	menu_id = MAIN_MENU_ID;
	current_selected_element = 0;
	current_menu = 0;
	using_menu = false;
	idle_encoder_position = get_encoder();
}

void menu_task(void *pvParameters){

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	display_draw_menu();
	uint8_t current_action = UNPRESSED, last_action = UNUSED;
	uint8_t button_lock = 0;
	uint32_t notify_to_process = 0;
	bool use_display = true;
	bool use_matrix = true;

	using_menu = true;

	xTaskNotifyGive(main_task_handle);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	idle_encoder_position = get_encoder();

	while(1){
		if (using_menu == false){
			current_encoder_position = get_encoder();
			if( current_encoder_position != idle_encoder_position ){
				using_menu = true;
			}else{
				xTaskNotifyGive(process_task_handle);
				xTaskNotify(process_task_handle, notify_to_process, eSetValueWithOverwrite);
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}
			idle_encoder_position = current_encoder_position;
		}else{
			if(current_selected_element != last_selected_element || current_action != last_action){
				if(current_action == PRESSED){
					current_menu = current_selected_element + menu_id;
					current_selected_element = 0;
				}
				switch(current_menu){
					case 0:
						display_menu_main(current_selected_element);
						menu_id = MAIN_MENU_ID;
						break;
					case 1:
						display_menu_plot(current_selected_element);
						menu_id = PLOT_MENU_ID;
						break;
					case 11:
						notify_to_process = SELECTION_FREQ_PLOT;
						return_to_main_menu();
						break;
					case 12:
						notify_to_process = SELECTION_TIME_PLOT;
						return_to_main_menu();
						break;
					case 13:
					    notify_to_process = SELECTION_POWER_PLOT;		
						return_to_main_menu();
						break;
					case 2:
						display_menu_conf(current_selected_element, use_display, use_matrix);
						menu_id = CONF_MENU_ID;
						break;
					case 21:
						if(use_matrix == true){
							notify_to_process = SELECION_MATRIX_OFF;
							use_matrix = false;
						}else{
							notify_to_process = SELECION_MATRIX_ON;
							use_matrix = true;
						}
						return_to_main_menu();
						break;
					case 22:
						if(use_display == true){
							notify_to_process = SELECTION_DISPLAY_OFF;
							use_display = false;
						}else{
							notify_to_process = SELECTION_DISPLAY_ON;
							use_display = true;
						}
						return_to_main_menu();
						break;
					case 23:
						notify_to_process = SELECTION_CHANGE_FREQ;
						return_to_main_menu();
						break;
					case 3:
						return_to_main_menu();
						break;
					default: break;
				}
			}
			last_selected_element = current_selected_element;
			last_action = current_action;

			new_encoder_position = get_encoder();
			difference_position = new_encoder_position - last_encoder_position;
			last_encoder_position = new_encoder_position;
			if(difference_position > ENCODER_MENU_STEP){
				current_selected_element++;
				if(current_selected_element > MAX_MENU_ELEMENTS){
					current_selected_element = FIRST_MENU_ELEMENT;
				}
			}else if(difference_position < -ENCODER_MENU_STEP){
				current_selected_element--;
				if(current_selected_element < FIRST_MENU_ELEMENT){
					current_selected_element = MAX_MENU_ELEMENTS;
				}
			}

			if(button_lock == 0){
				if(HAL_GPIO_ReadPin(encoder_button_GPIO_Port, encoder_button_pin) == GPIO_PIN_RESET){
					current_action = PRESSED;
					button_lock++;
				}
			}else{
				current_action = UNPRESSED;
				button_lock++;
				if(button_lock > 1){
					button_lock = 0;
				}
			}
			vTaskDelay(pdMS_TO_TICKS(250));
		}
    }
}


void display_draw_menu(){
	bool mode = 0;
	uint8_t iterations = 0;

	while(iterations++!=2){
		SSD1306_Clear();
		if(mode == 1){
		SSD1306_DrawBitmap(0,0,&dick_bitmap,128,64,1);
		SSD1306_GotoXY(2,2);
		SSD1306_Puts("UNLP,", &Font_7x10, 1);

			mode = 0;
		}else{
		SSD1306_DrawBitmap(0,0,&dick_bitmap_2,128,64,1);
		SSD1306_GotoXY(2,2);
		SSD1306_Puts("UNLP,", &Font_7x10, 1);
			mode = 1;
		}
		SSD1306_UpdateScreen();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void display_menu_main(uint8_t selected){
	SSD1306_Fill (0);
	if( selected == 1){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
		SSD1306_Puts("PLOTS", &Font_7x10, 0);
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
		SSD1306_Puts("PLOTS", &Font_7x10, 1);
	}
	if( selected == 2){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
		SSD1306_Puts("CONFIG", &Font_7x10, 0);
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
		SSD1306_Puts("CONFIG", &Font_7x10, 1);
	}
	if( selected == 3){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
		SSD1306_Puts("METRICS", &Font_7x10, 0);
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
		SSD1306_Puts("METRICS", &Font_7x10, 1);
	}
	SSD1306_UpdateScreen();
}

void display_menu_plot(uint8_t selected){
	SSD1306_Fill (0);
	if( selected == 1){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
		SSD1306_Puts("FREQUENCY PLOT", &Font_7x10, 0);
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
		SSD1306_Puts("FREQUENCY PLOT", &Font_7x10, 1);
	}
	if( selected == 2){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
		SSD1306_Puts("TEMPORAL PLOT ", &Font_7x10, 0);
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
		SSD1306_Puts("TEMPORAL PLOT ", &Font_7x10, 1);
	}
	if( selected == 3){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
		SSD1306_Puts("POWER PLOT", &Font_7x10, 0);
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
		SSD1306_Puts("POWER_PLOT", &Font_7x10, 1);
	}
	SSD1306_UpdateScreen();
}

void display_menu_conf(uint8_t selected, bool use_display, bool use_matrix){
	SSD1306_Fill (0);
	if( selected == 1){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
		if( use_matrix == true){
			SSD1306_Puts("MATRIX ON", &Font_7x10, 0);
		}else{
			SSD1306_Puts("MATRIX OFF", &Font_7x10, 0);
		}
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
		if( use_matrix == true){
			SSD1306_Puts("MATRIX ON", &Font_7x10, 1);
		}else{
			SSD1306_Puts("MATRIX OFF", &Font_7x10, 1);
		}
	}
	if( selected == 2){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
		if( use_display == true ){
			SSD1306_Puts("DISPLAY ON", &Font_7x10, 0);
		}else{
			SSD1306_Puts("DISPLAY OFF", &Font_7x10, 0);
		}
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
		if( use_display == true ){
			SSD1306_Puts("DISPLAY ON", &Font_7x10, 1);
		}else{
			SSD1306_Puts("DISPLAY OFF", &Font_7x10, 1);
		}
	}
	if( selected == 3){
		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
		SSD1306_Puts("CHANGE FREQ", &Font_7x10, 0);
	}else{
		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
		SSD1306_Puts("CHANGE FREQ", &Font_7x10, 1);
	}
	SSD1306_UpdateScreen();
}

int16_t get_encoder(){
	uint16_t value = TIM4->CNT;
	if(value > 32767){
		return (int16_t)(-1)*(65535-value);
	}else{
		return (int16_t)value;
	}
}

static void MX_TIM4_Init(void){

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}
// 
void trace_toggle(int tag){
	switch(tag){
		case(IDLE_TASK_TAG):
			HAL_GPIO_TogglePin(trace_1_GPIO_Port, trace_1_Pin);
			break;
		case(DISPLAY_TASK_TAG):
            HAL_GPIO_TogglePin(trace_5_GPIO_Port, trace_5_Pin);
			break;
		case(PROCESS_TASK_TAG):
			HAL_GPIO_TogglePin(trace_3_GPIO_Port, trace_3_Pin);
			break;
		case(LEDMATRIX_TASK_TAG):
			HAL_GPIO_TogglePin(trace_4_GPIO_Port, trace_4_Pin);
			break;
        case(AUXILIAR_TAG_1):
			HAL_GPIO_TogglePin(trace_2_GPIO_Port, trace_2_Pin);
            break;
        case(AUXILIAR_TAG_2):
            // HAL_GPIO_TogglePin(trace_5_GPIO_Port, trace_5_Pin);
            break;
		default: 
			break;
	}
}

void trace_on(int tag){
	switch(tag){
		case(IDLE_TASK_TAG):
			HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_SET);
			break;
		case(DISPLAY_TASK_TAG):
            HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_SET);
			break;
		case(PROCESS_TASK_TAG):
			HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_SET);
			break;
		case(LEDMATRIX_TASK_TAG):
			HAL_GPIO_WritePin(trace_4_GPIO_Port, trace_4_Pin, GPIO_PIN_SET);
			break;
        case(AUXILIAR_TAG_1):
			HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_SET);
            break;
        case(AUXILIAR_TAG_2):
            // HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_SET);
            break;
		default: 
			break;
	}
}

void trace_off(int tag){
	switch(tag){
		case(IDLE_TASK_TAG):
			HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_RESET);
			break;
		case(DISPLAY_TASK_TAG):
            HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_RESET);
			break;
		case(PROCESS_TASK_TAG):
			HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_RESET);
			break;
		case(LEDMATRIX_TASK_TAG):
			HAL_GPIO_WritePin(trace_4_GPIO_Port, trace_4_Pin, GPIO_PIN_RESET);
			break;
        case(AUXILIAR_TAG_1):
			HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_RESET);
            break;
        case(AUXILIAR_TAG_2):
            // HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_RESET);
            break;
		default: 
			break;
	}
}
