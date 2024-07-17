#ifndef ___MENU_H___
#define ___MENU_H___

#include <stdint.h>

int16_t get_encoder();
void display_menu_2(uint8_t selected);
void display_menu_1(uint8_t selected);
void display_draw_menu();
bool menu_init();

#endif