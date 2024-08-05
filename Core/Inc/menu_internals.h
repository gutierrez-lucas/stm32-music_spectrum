#ifndef ____MENU_INTERNALS_H____
#define ____MENU_INTERNALS_H____

#include <stdint.h>
#include <stdbool.h>

typedef struct menu_internals_t{
	uint8_t current_selected_element;
	uint8_t last_selected_element;
	uint8_t current_menu;
	int16_t idle_encoder_position;
	int16_t current_encoder_position;
	uint8_t menu_id;
	bool using_menu;
}menu_internals_t;

#define ENCODER_MENU_STEP 4
#define MAX_MENU_ELEMENTS 3
#define FIRST_MENU_ELEMENT 1
////////////
typedef enum action_t{
	UNUSED = 99,
	PRESSED = 1,
	UNPRESSED = 0
}action_t;


#define FONT_HEIGHT 10
#define FONT_WIDTH 7
#define STARTING_LEFT 10
#define STARTING_TOP 5
#define BOX_3_SEPARATOR_HIGH 3
// #define BOX_3_HEIGHT (uint8_t)((SSD1306_HEIGHT-4*BOX_3_SEPARATOR_HIGH)/3)
#define BOX_3_HEIGHT 16 
#define BOX_3_HEIGHT_DISTANCE_NEXT (BOX_3_HEIGHT+BOX_3_SEPARATOR_HIGH)
#define BOX_1_WIDTH (SSD1306_WIDTH-STARTING_LEFT)
#define STRING_3_TOP (STARTING_TOP+4)
#define STRING_3_LEFT (STARTING_LEFT+3)

#endif