#ifndef ___MENU_H___
#define ___MENU_H___

#include <stdint.h>
#include <stdbool.h>

#define SELECTION_FREQ_PLOT      1	
#define SELECTION_TIME_PLOT      2
#define SELECTION_POWER_PLOT     3
#define SELECION_MATRIX_OFF      4
#define SELECTION_DISPLAY_OFF    5
#define SELECTION_CHANGE_FREQ    6

int16_t get_encoder();
void display_menu_plot(uint8_t selected);
void display_menu_main(uint8_t selected);
void display_draw_menu();
bool menu_init();

#endif