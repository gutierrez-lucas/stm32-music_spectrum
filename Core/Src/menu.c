#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "menu.h"
#include "menu_internals.h"
//
#include "data_process.h"
#include "../display/ssd1306.h"
#include "serial.h"

xTaskHandle menu_task_handle = NULL;
extern xTaskHandle main_task_handle;
extern xTaskHandle process_task_handle;

extern void change_max_freq(uint8_t freq);

TIM_HandleTypeDef htim4;

char *menu_elements[MENU_ELEMENTS_MAIN] = {'\0'};
char *plot_elements[MENU_ELEMENTS_PLOT] = {'\0'};
char *config_elements[MENU_ELEMENTS_CONF] = {'\0'};
char *freq_elements[MENU_ELEMENTS_FREQ] = {'\0'};
char *indicator_elements[MENU_ELEMENTS_FREQ] = {'\0'};

static int16_t get_encoder();
static void draw_menu_elements(uint8_t selected, char** elements);
static uint8_t get_selected_element(menu_internals_t* foo);
static uint8_t get_button_status();
static void load_menu_elements();
static void return_to_main_menu(menu_internals_t* foo);
static uint8_t get_amount_of_menu_elements(uint8_t menu_id);

static void menu_task(void *pvParameters);
static void MX_TIM4_Init(void);
bool menu_init();

bool menu_init(){

	MX_TIM4_Init();
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	load_menu_elements();

	xTaskCreate(menu_task, "menu_task", 250, NULL, tskIDLE_PRIORITY+1, &menu_task_handle) != pdPASS ? printf("MENU: TASK ERR\r\n") : printf("MENU: TASK OK\r\n");
	vTaskSetApplicationTaskTag( menu_task_handle, ( void * ) MENU_TASK_TAG );
    return true;
}

static void menu_task(void *pvParameters){

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	menu_internals_t menu_internals = {
		.current_selected_element = 0,
		.last_selected_element = 99,
		.current_menu = 0,
		.idle_encoder_position = get_encoder(),
		.current_encoder_position = 0, 
		.menu_id = true,
		.using_menu = true 
	};

	uint8_t current_action = UNPRESSED, last_action = UNUSED;

	notification_union menu_selection;
	load_default_configuration(&menu_selection);

	xTaskNotifyGive(main_task_handle);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while(1){
		if (menu_internals.using_menu == false){
			menu_internals.current_encoder_position = get_encoder();
			if( menu_internals.current_encoder_position != menu_internals.idle_encoder_position ){
				menu_internals.using_menu = true;
			}else{
				xTaskNotify(process_task_handle, menu_selection.stream, eSetValueWithOverwrite);
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}
			menu_internals.idle_encoder_position = menu_internals.current_encoder_position;
		}else{
			if(menu_internals.current_selected_element != menu_internals.last_selected_element || current_action != last_action){
				if(current_action == PRESSED){
					menu_internals.current_menu = menu_internals.current_selected_element + menu_internals.menu_id;
					menu_internals.current_selected_element = 0;
				}
				switch(menu_internals.current_menu){
					case MENU_ID_MAIN+MENU_SELECTION_MAIN:
						draw_menu_elements(menu_internals.current_selected_element, menu_elements);
						menu_internals.menu_id = MENU_ID_MAIN;
						break;
					case MENU_ID_MAIN+MENU_SELECTION_PLOT:
						draw_menu_elements(menu_internals.current_selected_element, plot_elements);
						menu_internals.menu_id = MENU_ID_PLOT;
						break;
					case MENU_ID_PLOT+MENU_SELECTION_PLOT_FREQ:
						set_plot_mode_freq(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_PLOT+MENU_SELECTION_PLOT_TEMP:
						set_plot_mode_time(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_PLOT+MENU_SELECTION_PLOT_POWER:
						set_show_max_power(menu_selection.section.configuration);
						set_plot_mode_power(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_MAIN+MENU_SELECTION_CONF:
						draw_menu_elements(menu_internals.current_selected_element, config_elements);
						menu_internals.menu_id = MENU_ID_CONF;
						break;
					case MENU_ID_CONF+MENU_SELECTION_CONF_MATRIX:
					    toggle_use_matrix(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_CONF+MENU_SELECTION_CONF_DISPLAY:
						toggle_use_display(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_CONF+MENU_SELECTION_CONF_FREQUENCY:
						draw_menu_elements(menu_internals.current_selected_element, freq_elements);
						menu_internals.menu_id = MENU_ID_FREQUENCY;
						break;
					case MENU_ID_CONF+MENU_SELECTION_CONF_HIGHLIGHT_MAX:
						toogle_highlight_max(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_CONF+MENU_SELECTION_CONF_USE_LOG_SCALE:
						toggle_use_log_scale(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_FREQUENCY+MENU_SELECTION_FREQ_20K:
						set_max_freq_20k(menu_selection.section.configuration);
						change_max_freq(USE_20K);
						return_to_main_menu(&menu_internals);
						break; 
					case MENU_ID_FREQUENCY+MENU_SELECTION_FREQ_10K:
						set_max_freq_10k(menu_selection.section.configuration);
						change_max_freq(USE_10K);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_FREQUENCY+MENU_SELECTION_FREQ_5K:
						set_max_freq_5k(menu_selection.section.configuration);
						change_max_freq(USE_5K);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_MAIN+MENU_SELECTION_INDICATOR:
						draw_menu_elements(menu_internals.current_selected_element, indicator_elements);
						menu_internals.menu_id = MENU_ID_INDICATOR;
						break;
					case MENU_ID_INDICATOR+MENU_SELECTION_INDICATOR_MAIN_FREQ:
						set_show_max_freq(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_INDICATOR+MENU_SELECTION_INDICATOR_TOTAL_POW:
						set_show_max_power(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					case MENU_ID_INDICATOR+MENU_SELECTION_INDICATOR_NONE:
						set_show_none(menu_selection.section.configuration);
						return_to_main_menu(&menu_internals);
						break;
					default:  printf("MENU: ERR unknown selection: %d\r\n", menu_internals.current_menu); break;
				}
			}
			menu_internals.last_selected_element = menu_internals.current_selected_element;
			last_action = current_action;

			menu_internals.current_selected_element = get_selected_element(&menu_internals);
			current_action = get_button_status();

			vTaskDelay(pdMS_TO_TICKS(100));
		}
    }
}

static void draw_menu_elements(uint8_t selected, char** elements){
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

static int16_t get_encoder(){
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

static void load_menu_elements(){
	menu_elements[0] = "PLOT";
	menu_elements[1] = "CONFIG";
	menu_elements[2] = "INDICATOR";

	indicator_elements[0] = "MAIN FREQ";
	indicator_elements[1] = "TOTAL POW";
	indicator_elements[2] = "NONE";
	
	config_elements[0] = "MATRIX";
	config_elements[1] = "DISPLAY";
	config_elements[2] = "MAX FREQ";
	config_elements[3] = "HIGHLIGHT MAX";
	config_elements[4] = "USE LOG SCALE";

	freq_elements[0] = "20kHZ";
	freq_elements[1] = "10kHZ";
	freq_elements[2] = "5kHZ";

	plot_elements[0] = "FREQUENCY";
	plot_elements[1] = "TEMPORAL";
	plot_elements[2] = "POWER";
}

static uint8_t get_selected_element(menu_internals_t* foo){
	static int16_t new_encoder_position = 0, last_encoder_position = 0, difference_position = 0;

	new_encoder_position = get_encoder();
	difference_position = new_encoder_position - last_encoder_position;
	last_encoder_position = new_encoder_position;
	if(difference_position > ENCODER_MENU_STEP){
		foo->current_selected_element++;
		if(foo->current_selected_element > get_amount_of_menu_elements(foo->menu_id)){
			foo->current_selected_element = FIRST_MENU_ELEMENT;
		}
	}else if(difference_position < -ENCODER_MENU_STEP){
		if(foo->current_selected_element == 0){
			foo->current_selected_element = get_amount_of_menu_elements(foo->menu_id);
		}else{
			foo->current_selected_element--;
		}
	}
	return(foo->current_selected_element);
}

static uint8_t get_button_status(){
	static uint8_t button_lock = 0;
	if(button_lock == 0){
		if(HAL_GPIO_ReadPin(encoder_button_GPIO_Port, encoder_button_pin) == GPIO_PIN_RESET){
			button_lock++;
			return(PRESSED);
		}
	}else{
		button_lock++;
		if(button_lock > 1){
			button_lock = 0;
		}
	}
	return(UNPRESSED);
}

static void return_to_main_menu(menu_internals_t* foo){
	SSD1306_Clear();
	foo->menu_id = MENU_ID_MAIN;
	foo->current_selected_element = 0;
	foo->current_menu = 0;
	foo->using_menu = false;
	foo->idle_encoder_position = get_encoder();
}

void load_default_configuration(notification_union* foo){
	foo->stream = 0;
	set_max_freq_20k(foo->section.configuration);
	set_plot_mode_freq(foo->section.configuration);
	toggle_use_matrix(foo->section.configuration);
	toggle_use_display(foo->section.configuration);
	set_show_max_freq(foo->section.configuration);
	toogle_highlight_max(foo->section.configuration);
	toggle_use_log_scale(foo->section.configuration);
}

static uint8_t get_amount_of_menu_elements(uint8_t menu_id){
	switch(menu_id){
		case MENU_ID_MAIN: return MENU_ELEMENTS_MAIN;
		case MENU_ID_PLOT: return MENU_ELEMENTS_PLOT;
		case MENU_ID_CONF: return MENU_ELEMENTS_CONF;
		case MENU_ID_FREQUENCY: return MENU_ELEMENTS_FREQ;
		case MENU_ID_INDICATOR: return MENU_ELEMENTS_IND;
		default: return 0;
	}
}