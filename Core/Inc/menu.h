#ifndef ___MENU_H___
#define ___MENU_H___

#include <stdint.h>
#include <stdbool.h>

#define SELECTION_FREQ_PLOT      1	
#define SELECTION_TIME_PLOT      2
#define SELECTION_POWER_PLOT     3
#define SELECION_MATRIX_OFF      4
#define SELECION_MATRIX_ON       7 
#define SELECTION_DISPLAY_OFF    5
#define SELECTION_DISPLAY_ON     8
#define SELECTION_CHANGE_FREQ    6

#define FREQ_1      0b0000000000000001
#define FREQ_2      0b0000000000000010
#define FREQ_3      0b0000000000000100
#define PLOT_POWER  0b0000000000001000
#define PLOT_TIME   0b0000000000010000
#define PLOT_FREQ   0b0000000000100000
#define PLOT_PART   0b0000000000111000
#define USE_MATRIX  0b0000000001000000
#define USE_DISPLAY 0b0000000010000000

#define set_plot_mode_freq(x) x &= ~PLOT_PART; x |= PLOT_FREQ
#define set_plot_mode_time(x) x &= ~PLOT_PART; x |= PLOT_TIME
#define set_plot_mode_power(x) x &= ~PLOT_PART; x |= PLOT_POWER
#define check_use_display(x) x&USE_DISPLAY ? true : false
#define check_use_matrix(x) x&USE_MATRIX ? true : false
#define check_plot_mode_freq(x) x&PLOT_FREQ ? true : false
#define check_plot_mode_time(x) x&PLOT_TIME ? true : false
#define check_plot_mode_power(x) x&PLOT_POWER ? true : false
#define toggle_use_display(x) x ^= USE_DISPLAY
#define toggle_use_matrix(x) x ^= USE_MATRIX

int16_t get_encoder();
void display_menu_plot(uint8_t selected);
void display_menu_main(uint8_t selected);
void display_draw_menu();
bool menu_init();

#endif