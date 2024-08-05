#ifndef ___MENU_H___
#define ___MENU_H___

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "menu_elements.h"

void load_default_configuration(notification_union* foo);

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
#define check_show_max_power(x) x&SHOW_MAX_POW ? true : false
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