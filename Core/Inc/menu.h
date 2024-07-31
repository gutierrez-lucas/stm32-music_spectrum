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
#define SHOW_MAX_POW    0b0000001000000000
#define SHOW_NONE       0b0000010000000000
#define SHOW_PART       0b0000011100000000
#define HIGHLIGHT_MAX   0b0000100000000000
#define USE_LOG_SCALE   0b0001000000000000

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
#define set_show_max_freq(x) x&=~SHOW_PART; x|=SHOW_MAX_FREQ
#define set_show_max_power(x) x&=~SHOW_PART; x|=SHOW_MAX_POW
#define set_show_none(x) x&=~SHOW_PART; x|=SHOW_NONE
#define toogle_highlight_max(x) x^=HIGHLIGHT_MAX
#define check_highlight_max(x) x&HIGHLIGHT_MAX ? true : false
#define check_use_log_scale(x) x&USE_LOG_SCALE ? true : false
#define toggle_use_log_scale(x) x^=USE_LOG_SCALE

#endif