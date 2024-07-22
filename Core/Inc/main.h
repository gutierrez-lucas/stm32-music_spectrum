#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

void Error_Handler(void);

void trace_off(int tag);
void trace_on(int tag);
void trace_toggle(int tag);

#define BUFFER_SIZE 256

#define trace_1_Pin GPIO_PIN_13
#define trace_1_GPIO_Port GPIOB
#define trace_2_Pin GPIO_PIN_14
#define trace_2_GPIO_Port GPIOB
#define trace_3_Pin GPIO_PIN_15
#define trace_3_GPIO_Port GPIOB
#define trace_4_Pin GPIO_PIN_8
#define trace_4_GPIO_Port GPIOA
#define trace_5_Pin GPIO_PIN_11
#define trace_5_GPIO_Port GPIOA

#define display_rst_pin GPIO_PIN_5
#define display_rst_pin_GPIO_Port GPIOB

#define encoder_button_pin GPIO_PIN_0
#define encoder_button_GPIO_Port GPIOB

#define IDLE_TASK_TAG	    0
#define DISPLAY_TASK_TAG    1
#define PROCESS_TASK_TAG	2
#define LEDMATRIX_TASK_TAG	6 
#define MENU_TASK_TAG       7
#define AUXILIAR_TAG_1	    3
#define AUXILIAR_TAG_2	    4

typedef union notification_union{
    uint32_t stream;
    struct {
        uint16_t configuration;
        uint16_t payload;
    }section;
}notification_union;

void print_binary_32(uint32_t number);

#endif /* __MAIN_H */
