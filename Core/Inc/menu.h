#ifndef ___MENU_H___
#define ___MENU_H___

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

void load_default_configuration(notification_union* foo);

#define FREQ_1          0b0000000000000001
#define FREQ_2          0b0000000000000010
#define FREQ_3          0b0000000000000100
#define FREQ_PART       0b0000000000000111
#define PLOT_POWER      0b0000000000001000
#define PLOT_TIME       0b0000000000010000
#define PLOT_FREQ       0b0000000000100000
#define PLOT_PART       0b0000000000111000
#define USE_MATRIX      0b0000000001000000
#define USE_DISPLAY     0b0000000010000000
#define SHOW_MAX_FREQ   0b0000000100000000

#define set_max_freq_20k(x) x&=~FREQ_PART; x |= FREQ_1
#define set_max_freq_10k(x) x&=~FREQ_PART; x |= FREQ_2
#define set_max_freq_5k(x) x&=~FREQ_PART; x |= FREQ_3
#define check_max_freq_20k(x) x&FREQ_1 ? true : false
#define check_max_freq_10k(x) x&FREQ_2 ? true : false
#define check_max_freq_5k(x) x&FREQ_3 ? true : false
#define set_plot_mode_freq(x) x&=~PLOT_PART; x |= PLOT_FREQ
#define set_plot_mode_time(x) x&=~PLOT_PART; x |= PLOT_TIME
#define set_plot_mode_power(x) x&=~PLOT_PART; x |= PLOT_POWER
#define check_use_display(x) x&USE_DISPLAY ? true : false
#define check_use_matrix(x) x&USE_MATRIX ? true : false
#define check_show_max_freq(x) x&SHOW_MAX_FREQ ? true : false
#define check_plot_mode_freq(x) x&PLOT_FREQ ? true : false
#define check_plot_mode_time(x) x&PLOT_TIME ? true : false
#define check_plot_mode_power(x) x&PLOT_POWER ? true : false
#define toggle_use_display(x) x^=USE_DISPLAY
#define toggle_use_matrix(x) x^=USE_MATRIX
#define toggle_show_max_freq(x) x^=SHOW_MAX_FREQ


#endif