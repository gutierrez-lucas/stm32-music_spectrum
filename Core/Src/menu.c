#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "menu.h"
#include "data_process.h"
// #include "../display/bitmap.h"
#include "../display/ssd1306.h"
#include "serial.h"

xTaskHandle menu_task_handle = NULL;
extern xTaskHandle main_task_handle;
extern xTaskHandle process_task_handle;

extern void change_max_freq(uint8_t freq);

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

#define MENU_ELEMENTS_MAIN 3
#define MENU_ELEMENTS_PLOT 3
#define MENU_ELEMENTS_CONF 4
#define MENU_ELEMENTS_FREQ 3

char *menu_elements[MENU_ELEMENTS_MAIN] = {'\0'};
char *plot_elements[MENU_ELEMENTS_PLOT] = {'\0'};
char *config_elements[MENU_ELEMENTS_CONF] = {'\0'};
char *freq_elements[MENU_ELEMENTS_FREQ] = {'\0'};

int16_t get_encoder();
void display_menu_plot(uint8_t selected);
void draw_menu_elements(uint8_t selected, char** elements);
bool menu_init();

int16_t new_encoder_position = 0, last_encoder_position = 0, difference_position = 0;
bool using_menu;
uint8_t menu_id = 0;

#define MENU_SELECTION_MAIN 0
#define MENU_SELECTION_PLOT 1
#define MENU_SELECTION_CONF 2 
#define MENU_SELECTION_FREQUENCY 3 

#define MENU_ID_MAIN 0
#define MENU_ID_PLOT 10
#define MENU_ID_CONF 20
#define MENU_ID_FREQUENCY 30

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

	menu_elements[0] = "PLOT";
	menu_elements[1] = "CONFIG";
	menu_elements[2] = "MAX FREQ";
	config_elements[0] = "MATRIX";
	config_elements[1] = "DISPLAY";
	config_elements[2] = "MAX FREQ";
	config_elements[3] = "BACK";
	freq_elements[0] = "20kHZ";
	freq_elements[1] = "10kHZ";
	freq_elements[2] = "5kHZ";
	plot_elements[0] = "FREQUENCY";
	plot_elements[1] = "TEMPORAL";
	plot_elements[2] = "POWER";
	MX_TIM4_Init();
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	xTaskCreate(menu_task, "menu_task", 250, NULL, tskIDLE_PRIORITY+1, &menu_task_handle) != pdPASS ? printf("MENU: TASK ERR\r\n") : printf("MENU: TASK OK\r\n");
	vTaskSetApplicationTaskTag( menu_task_handle, ( void * ) MENU_TASK_TAG );
    return true;
}

void return_to_main_menu(){
	SSD1306_Clear();
	menu_id = MENU_ID_MAIN;
	current_selected_element = 0;
	current_menu = 0;
	using_menu = false;
	idle_encoder_position = get_encoder();
}

void load_default_configuration(notification_union* foo){
	foo->stream = 0;
	set_max_freq_20k(foo->section.configuration);
	set_plot_mode_freq(foo->section.configuration);
	toggle_use_matrix(foo->section.configuration);
	toggle_use_display(foo->section.configuration);
	toggle_show_max_freq(foo->section.configuration);
}

uint8_t get_amount_of_menu_elements(uint8_t menu_id){
	switch(menu_id){
		case MENU_ID_MAIN: return MENU_ELEMENTS_MAIN;
		case MENU_ID_PLOT: return MENU_ELEMENTS_PLOT;
		case MENU_ID_CONF: return MENU_ELEMENTS_CONF;
		case MENU_ID_FREQUENCY: return MENU_ELEMENTS_FREQ;
		default: return 0;
	}
}

void menu_task(void *pvParameters){

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	uint8_t current_action = UNPRESSED, last_action = UNUSED;
	uint8_t button_lock = 0;

	notification_union menu_selection;
	load_default_configuration(&menu_selection);

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
				xTaskNotify(process_task_handle, menu_selection.stream, eSetValueWithOverwrite);
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
					case MENU_ID_MAIN+MENU_SELECTION_MAIN:
						draw_menu_elements(current_selected_element, menu_elements);
						menu_id = MENU_ID_MAIN;
						break;
					case MENU_ID_MAIN+MENU_SELECTION_PLOT:
						draw_menu_elements(current_selected_element, plot_elements);
						menu_id = MENU_ID_PLOT;
						break;
					case MENU_ID_PLOT+1:
						set_plot_mode_freq(menu_selection.section.configuration);
						return_to_main_menu();
						break;
					case MENU_ID_PLOT+2:
						set_plot_mode_time(menu_selection.section.configuration);
						return_to_main_menu();
						break;
					case MENU_ID_PLOT+3:
						set_plot_mode_power(menu_selection.section.configuration);
						return_to_main_menu();
						break;
					case MENU_ID_MAIN+MENU_SELECTION_CONF:
						draw_menu_elements(current_selected_element, config_elements);
						menu_id = MENU_ID_CONF;
						break;
					case MENU_ID_CONF+1:
					    toggle_use_matrix(menu_selection.section.configuration);
						return_to_main_menu();
						break;
					case MENU_ID_CONF+2:
						toggle_use_display(menu_selection.section.configuration);
						return_to_main_menu();
						break;
					case MENU_ID_CONF+3:
						toggle_show_max_freq(menu_selection.section.configuration);
						return_to_main_menu();
						break;
					case MENU_ID_CONF+4:
						return_to_main_menu();
						break;
					case MENU_ID_MAIN+MENU_SELECTION_FREQUENCY:
						draw_menu_elements(current_selected_element, freq_elements);
						menu_id = MENU_ID_FREQUENCY;
						break;
					case MENU_ID_FREQUENCY+1:
						set_max_freq_20k(menu_selection.section.configuration);
						change_max_freq(USE_20K);
						return_to_main_menu();
						break; 
					case MENU_ID_FREQUENCY+2:
						set_max_freq_10k(menu_selection.section.configuration);
						change_max_freq(USE_10K);
						return_to_main_menu();
						break;
					case MENU_ID_FREQUENCY+3:
						set_max_freq_5k(menu_selection.section.configuration);
						change_max_freq(USE_5K);
						return_to_main_menu();
						break;
					default:  printf("MENU: ERR unknown selection: %d\r\n", current_menu); break;
				}
			}
			last_selected_element = current_selected_element;
			last_action = current_action;

			new_encoder_position = get_encoder();
			difference_position = new_encoder_position - last_encoder_position;
			last_encoder_position = new_encoder_position;
			if(difference_position > ENCODER_MENU_STEP){
				current_selected_element++;
				if(current_selected_element > get_amount_of_menu_elements(menu_id)){
					current_selected_element = FIRST_MENU_ELEMENT;
				}
			}else if(difference_position < -ENCODER_MENU_STEP){
				current_selected_element--;
				if(current_selected_element < FIRST_MENU_ELEMENT){
					current_selected_element = get_amount_of_menu_elements(menu_id);
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
			vTaskDelay(pdMS_TO_TICKS(100));
		}
    }
}

void draw_menu_elements(uint8_t selected, char** elements){
	bool highligth = false;
	uint8_t difference_from_max = 0;
	if(selected > MAX_MENU_ELEMENTS){
		difference_from_max = selected - MAX_MENU_ELEMENTS;
	}

	SSD1306_Fill (0);
	for(uint8_t menu_index = 0; menu_index <= 2; menu_index++){
		if( menu_index + difference_from_max + 1 == selected){
			highligth = false;
			SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*menu_index, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		}else{
			highligth = true;
			SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*menu_index, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
		}
		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*menu_index);
		SSD1306_Puts(elements[menu_index + difference_from_max], &Font_7x10, highligth);
	}
	SSD1306_UpdateScreen();
}

// void display_menu_conf(uint8_t selected, notification_union foo){
// 	SSD1306_Fill (0);
// 	if( selected == 1){
// 		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
// 		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
// 		if( check_use_matrix(foo.section.configuration) ){
// 			SSD1306_Puts("MATRIX ON", &Font_7x10, 0);
// 		}else{
// 			SSD1306_Puts("MATRIX OFF", &Font_7x10, 0);
// 		}
// 	}else{
// 		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
// 		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP);
// 		if( check_use_matrix(foo.section.configuration) ){
// 			SSD1306_Puts("MATRIX ON", &Font_7x10, 1);
// 		}else{
// 			SSD1306_Puts("MATRIX OFF", &Font_7x10, 1);
// 		}
// 	}
// 	if( selected == 2){
// 		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
// 		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
// 		if( check_use_display(foo.section.configuration) ){
// 			SSD1306_Puts("DISPLAY ON", &Font_7x10, 0);
// 		}else{
// 			SSD1306_Puts("DISPLAY OFF", &Font_7x10, 0);
// 		}
// 	}else{
// 		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
// 		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT);
// 		if( check_use_display(foo.section.configuration) ){
// 			SSD1306_Puts("DISPLAY ON", &Font_7x10, 1);
// 		}else{
// 			SSD1306_Puts("DISPLAY OFF", &Font_7x10, 1);
// 		}
// 	}
// 	if( selected == 3){
// 		SSD1306_DrawFilledRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
// 		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
// 		if( check_show_max_freq(foo.section.configuration) ){
// 			SSD1306_Puts("MAX-FREQ ON", &Font_7x10, 0);
// 		}else{
// 			SSD1306_Puts("MAX-FREQ OFF", &Font_7x10, 0);
// 		}
// 	}else{
// 		SSD1306_DrawRectangle(STARTING_LEFT,STARTING_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2, BOX_1_WIDTH, BOX_3_HEIGHT, 1);
// 		SSD1306_GotoXY(STRING_3_LEFT,STRING_3_TOP+BOX_3_HEIGHT_DISTANCE_NEXT*2);
// 		if ( check_show_max_freq(foo.section.configuration) ){
// 			SSD1306_Puts("MAX-FREQ ON", &Font_7x10, 1);
// 		}else{
// 			SSD1306_Puts("MAX-FREQ OFF", &Font_7x10, 1);
// 		}
// 	}
// 	SSD1306_UpdateScreen();
// }

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
