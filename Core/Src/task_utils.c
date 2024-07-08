#include "task_utils.h"

void trace_toggle(int tag){
	switch(tag){
		case(IDLE_TASK_TAG):
			HAL_GPIO_TogglePin(trace_1_GPIO_Port, trace_1_Pin);
			break;
		case(DISPLAY_TASK_TAG):
			HAL_GPIO_TogglePin(trace_2_GPIO_Port, trace_2_Pin);
			break;
		case(PROCESS_TASK_TAG):
			HAL_GPIO_TogglePin(trace_3_GPIO_Port, trace_3_Pin);
			break;
        case(AUXILIAR_TAG_1):
            HAL_GPIO_TogglePin(trace_4_GPIO_Port, trace_4_Pin);
            break;
        case(AUXILIAR_TAG_2):
            HAL_GPIO_TogglePin(trace_5_GPIO_Port, trace_5_Pin);
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
			HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_SET);
			break;
		case(PROCESS_TASK_TAG):
			HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_SET);
			break;
        case(AUXILIAR_TAG_1):
            HAL_GPIO_WritePin(trace_4_GPIO_Port, trace_4_Pin, GPIO_PIN_SET);
            break;
        case(AUXILIAR_TAG_2):
            HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_SET);
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
			HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_RESET);
			break;
		case(PROCESS_TASK_TAG):
			HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_RESET);
			break;
        case(AUXILIAR_TAG_1):
            HAL_GPIO_WritePin(trace_4_GPIO_Port, trace_4_Pin, GPIO_PIN_RESET);
            break;
        case(AUXILIAR_TAG_2):
            HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_RESET);
            break;
		default: 
			break;
	}
}