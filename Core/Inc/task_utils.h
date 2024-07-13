#ifndef ____TASK_UTILS_H____
#define ____TASK_UTILS_H____

#include "main.h"

void trace_off(int tag);
void trace_on(int tag);
void trace_toggle(int tag);

#define trace_1_Pin GPIO_PIN_9
#define trace_1_GPIO_Port GPIOA
#define trace_2_Pin GPIO_PIN_10
#define trace_2_GPIO_Port GPIOA
#define trace_3_Pin GPIO_PIN_15
#define trace_3_GPIO_Port GPIOB
#define trace_4_Pin GPIO_PIN_8
#define trace_4_GPIO_Port GPIOA
#define trace_5_Pin GPIO_PIN_11
#define trace_5_GPIO_Port GPIOA

#define display_rst_pin GPIO_PIN_5
#define display_rst_pin_GPIO_Port GPIOB

#define IDLE_TASK_TAG	    0
#define DISPLAY_TASK_TAG    1
#define PROCESS_TASK_TAG	2
#define LEDMATRIX_TASK_TAG	6 
#define AUXILIAR_TAG_1	    3
#define AUXILIAR_TAG_2	    4


#endif