#include "../led_matrix/led_matrix.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

uint8_t rgbw_arr[NUM_OF_LEDS * BYTES_PER_LED * 8 + 1];//every pixel colour info is 24 bytes long

char test_matrix[9][8] = {
	{0, 1, 1, 0, 0, 1, 0, 1},
	{0, 1, 2, 3, 3, 2, 1, 2},
	{1, 2, 3, 4, 4, 2, 2, 2},
	{2, 4, 6, 5, 7, 6, 6, 3},
	{1, 3, 4, 4, 6, 5, 5, 3},
	{0, 1, 2, 3, 4, 4, 2, 1},
	{0, 0, 1, 1, 1, 2, 1, 0},
	{0, 0, 0, 1, 1, 1, 0, 0},
	{0, 0, 0, 1, 0, 0, 0, 0}
};

void rgb_matrix_clear_buffer(uint8_t *buffer, uint16_t bytenumber) {
	for (uint32_t i = 0; i < bytenumber-1; ++i) {
		buffer[i] = LOW_BIT;
	}
	buffer[bytenumber] = 0;//needs to be 0 to silent PWM at the end of transaction
}

void rgb_process_colors(rgb_mode_t mode, uint8_t* colors, uint8_t amplitude){
	switch(mode){
		case(OFF):
			colors[0] = 0;colors[1] = 0;colors[2] = 0; break;
		case(LIMITS):
			switch(amplitude){
				case 0: colors[0] = 0; colors[1] = 200; colors[2] = 0; break;
				case 1: colors[0] = 10; colors[1] = 200; colors[2] = 100; break;
				case 2: colors[0] = 10; colors[1] = 75; colors[2] = 80; break;
				case 3: colors[0] = 10; colors[1] = 0; colors[2] = 80; break;
				case 4: colors[0] = 300; colors[1] = 0; colors[2] = 80; break;
				case 5: colors[0] = 80; colors[1] = 0; colors[2] = 80; break;
				case 6: colors[0] = 80; colors[1] = 60; colors[2] = 10; break;
				case 7: colors[0] = 0; colors[1] = 0; colors[2] = 10; break;
				default: colors[0] = 80; colors[1] = 10; colors[2] = 0; break;
			}
			break;
		default: colors[0] = 0;colors[1] = 0;colors[2] = 0; break;
	}
}

void rgb_process_rotation(rotation_t rotation, uint8_t* coordinates, uint8_t x, uint8_t y){
	switch (rotation) {
		case ROTATION_0:
			coordinates[1] = y;
			coordinates[0] = x;
			break;
		case ROTATION_90:
			coordinates[1] = x;
			coordinates[0] = MATRIX_SIZE - y + 1;
			break;
		case ROTATION_180:
			coordinates[1] = MATRIX_SIZE - y + 1;
			coordinates[0] = MATRIX_SIZE - x + 1;
			break;
		case ROTATION_270:
			coordinates[1] = MATRIX_SIZE - x + 1;
			coordinates[0] = y;
			break;
		default: coordinates[1] = y; coordinates[0] = x; break;
	}
}

uint32_t rgb_matrix_set_pixel(uint8_t *buffer, rgb_mode_t mode, uint8_t x, uint8_t y, rotation_t rotation) {

	if(x*y>NUM_OF_LEDS){return -1;}//in case we mess up

	uint8_t y_rot, x_rot;
	uint8_t colors[3] = {100,0,0};
	uint8_t coordinates[2] = {1,1};

	rgb_process_colors(mode, colors, y);
	rgb_process_rotation(rotation, coordinates, x, y);

	uint8_t led_index = (coordinates[1]-1)*MATRIX_SIZE + coordinates[0] -1;

	for (uint32_t i = 0; i < BYTES_PER_LED * 8; ++i) { //we need to store every bit

		if (i < 8) { //this means first byte R
			if (colors[1] & (0x80 >> i)) { //this is a mask for reading every bit inside the byte R
				buffer[i + led_index * BYTES_PER_LED * 8] = HIGH_BIT;
			} else {
				buffer[i + led_index * BYTES_PER_LED * 8] = LOW_BIT;
			}
		}

		if ((i >= 8) & (i < 16)) { //this means second byte G
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

bool done = false;

void matrix_test_secuential(rotation_t rotation){
	static uint8_t x_counter = 1, y_counter = 1;

	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	rgb_matrix_set_pixel(&rgbw_arr, LIMITS, x_counter, y_counter, rotation);
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

void matrix_draw_vertical_line(uint8_t x, uint8_t y1, uint8_t y2, rotation_t rotation){
	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	for(uint8_t i=y1; i<=y2; i++){
		rgb_matrix_set_pixel(&rgbw_arr, LIMITS, x, i, rotation);
	}
}

void matrix_test_pyramid(rotation_t rotation){
	static uint8_t iteration_x = 1, iteration_y = 1;
	static uint8_t rotation_diff = 0;

	matrix_draw_vertical_line(iteration_x, 1, iteration_y, rotation+rotation_diff);
	if(iteration_x >= 5){
		iteration_y--;
	}else{
		iteration_y++;
	}
	iteration_x++;
	if(iteration_x == 9){
		iteration_x = 1;
		iteration_y = 1;
		rotation_diff ++;
		if(rotation_diff == 4){
			rotation_diff = 0;
		}
	}
}

void matrix_test_vertical_levels(rotation_t rotation, show_mode_t mode){
	static uint8_t table = 0;
	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	for(uint8_t i=1; i<=MATRIX_SIZE; i++){
		if(mode == POINTS){
			rgb_matrix_set_pixel(&rgbw_arr, LIMITS, i, test_matrix[table][i-1]+1, rotation);
		}else if(mode == LINES){
			for(uint8_t j=1; j<=test_matrix[table][i-1]; j++){
				rgb_matrix_set_pixel(&rgbw_arr, LIMITS, i, j, rotation);
			}
		}
	}
	table++;
	if(table == 9){
		table = 0;
	}
}