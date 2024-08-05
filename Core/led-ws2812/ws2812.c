#include "../led-ws2812/ws2812.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define CONFIG_MIRROR_MATRIX  // comment if want the led secuence be from right to left

uint8_t rgbw_arr[NUM_OF_LEDS * BYTES_PER_LED * 8 + 1];//every pixel colour info is 24 bytes long

void rgb_matrix_clear_buffer(uint8_t *buffer, uint16_t bytenumber) {
	for (uint32_t i = 0; i < bytenumber-1; ++i) {
		buffer[i] = LOW_BIT;
	}
	buffer[bytenumber] = 0; 
}

void rgb_process_colors(rgb_mode_t mode, uint8_t* colors, uint8_t amplitude){
	switch(mode){
		case(COLOR_LIMITS):
			switch(amplitude){
				case 1:  colors[0] =   0; colors[1] = 250; colors[2] =   0; break;
				case 2:  colors[0] =  10; colors[1] = 200; colors[2] = 100; break;
				case 3:  colors[0] =  10; colors[1] =  75; colors[2] =  80; break;
				case 4:  colors[0] =  10; colors[1] =   0; colors[2] = 200; break;
				case 5:  colors[0] =  10; colors[1] =   0; colors[2] =  80; break;
				case 6:  colors[0] = 200; colors[1] =   0; colors[2] =  80; break;
				case 7:  colors[0] = 250; colors[1] =  80; colors[2] =   0; break;
				case 8:  colors[0] = 250; colors[1] =   0; colors[2] =   0; break;
				default: colors[0] =  80; colors[1] =  10; colors[2] =   0; break;
			}
			break;
		case(COLOR_OFF):
				 colors[0] =   0; colors[1] =   0; colors[2] =   0; break;
		case(COLOR_FULL):
				 colors[0] = 255; colors[1] = 255; colors[2] = 255; break;
		case(COLOR_RED):
				 colors[0] = 255; colors[1] =   0; colors[2] =   0; break;
		case(COLOR_GREEN):
				 colors[0] =   0; colors[1] = 255; colors[2] =   0; break;
		default: 
				 colors[0] =   0; colors[1] =   0; colors[2] =   0; break;
	}
}

void rgb_process_mirror(uint8_t* coordinates){
	coordinates[0] = MATRIX_SIZE - coordinates[0];
}

void rgb_process_rotation(rotation_t rotation, uint8_t* coordinates, uint8_t x, uint8_t y){
	switch (rotation) {
		case ROTATION_0:
			coordinates[0] = x;
			coordinates[1] = y;
			break;
		case ROTATION_90:
			coordinates[0] = MATRIX_SIZE - y + 1;
			coordinates[1] = x;
			break;
		case ROTATION_180:
			coordinates[0] = MATRIX_SIZE - x + 1;
			coordinates[1] = MATRIX_SIZE - y + 1;
			break;
		case ROTATION_270:
			coordinates[0] = y;
			coordinates[1] = MATRIX_SIZE - x + 1;
			break;
		default: coordinates[0] = x; coordinates[1] = y;  break;
	}
}

uint32_t rgb_matrix_set_pixel(uint8_t *buffer, rgb_mode_t mode, uint8_t x, uint8_t y, rotation_t rotation) {

	if(x*y>NUM_OF_LEDS){return -1;}//in case we mess up

	uint8_t y_rot, x_rot;
	uint8_t colors[3] = {100,0,0};
	uint8_t coordinates[2] = {1,1}; // x,y

	rgb_process_colors(mode, colors, y);
	rgb_process_rotation(rotation, coordinates, x, y);
#ifdef CONFIG_MIRROR_MATRIX
	rgb_process_mirror(coordinates);
#endif

	uint8_t led_index = (coordinates[1]-1)*MATRIX_SIZE + coordinates[0];

	for (uint32_t i = 0; i < BYTES_PER_LED * 8; i++) { //we need to store every bit
	
		if (i < 8) { //this means first byte G
			if (colors[1] & (0x80 >> i)) { //this is a mask for reading every bit inside the byte R
				buffer[i + led_index * BYTES_PER_LED * 8] = HIGH_BIT;
			} else {
				buffer[i + led_index * BYTES_PER_LED * 8] = LOW_BIT;
			}
		}

		if ((i >= 8) & (i < 16)) { //this means second byte R
			if (colors[0] & (0x80 >> (i - 8))) {
				buffer[i + led_index * BYTES_PER_LED * 8] = HIGH_BIT;
			} else {
				buffer[i + led_index * BYTES_PER_LED * 8] = LOW_BIT;
			}
		}

		if ((i >= 16) & (i < 24)) { //this means third byte B
			if (colors[2] & (0x80 >> (i - 16))) {
				buffer[i + led_index * BYTES_PER_LED * 8] = HIGH_BIT;
			} else {
				buffer[i + led_index * BYTES_PER_LED * 8] = LOW_BIT;
			}
		}

	}
	return 1;
}

void matrix_test_secuential(rotation_t rotation){
	static uint8_t x_counter = 1, y_counter = 1;

	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	rgb_matrix_set_pixel(&rgbw_arr, COLOR_LIMITS, x_counter, y_counter, rotation);
	if(x_counter == 8){
		x_counter = 1;
		if(y_counter == 8){
			y_counter = 1;
		}else{
			y_counter++;
		}
	}else{
		x_counter++;
	}
}

void matrix_draw_vertical_line(uint8_t x, uint8_t y1, uint8_t y2, rotation_t rotation, bool highlight){
	static uint8_t max_y[8] = {0,0,0,0,0,0,0,0};
	static uint8_t iteration = 0;

	for(uint8_t i=y1; i<=y2; i++){
		rgb_matrix_set_pixel(rgbw_arr, COLOR_LIMITS, x, i, rotation);
	}
	if(highlight == true){
		if(max_y[x-1] < y2){
			max_y[x-1] = y2;
		}
		if (x == 8){
			iteration++;
		}
		if(iteration == 2){  // if a faster movement down wanted change 2 to 1
			iteration = 0;
			for(uint8_t i=0; i<8; i++){
				if(max_y[i] > 0){
					max_y[i]--;
				}
			}
		}
		if(max_y[x-1] > 0){
			rgb_matrix_set_pixel(rgbw_arr, COLOR_OFF, x, max_y[x-1]+1, rotation);
			if(max_y[x-1] == 1){
				rgb_matrix_set_pixel(rgbw_arr, COLOR_GREEN, x, max_y[x-1], rotation);
			}else{
				rgb_matrix_set_pixel(rgbw_arr, COLOR_RED, x, max_y[x-1], rotation);
			}
		}
	}
}

void matrix_test_pyramid(rotation_t rotation){
	static uint8_t rotation_change = 0;
	static uint8_t iteration_x = 1;
	static uint8_t iteration_y = 1;

	matrix_draw_vertical_line(iteration_x, 1, iteration_y, rotation+rotation_change, false);
	if(iteration_x >= 5){
		iteration_y--;
	}else{
		iteration_y++;
	}
	iteration_x++;
	if(iteration_x == 9){
		rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
		rotation_change++;
		if(rotation_change == 4){
			rotation_change = 0;
		}
		iteration_x = 1;
		iteration_y = 1;
	}
}

