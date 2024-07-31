#ifndef ___LED_MATRIX_H
#define ___LED_MATRIX_H

#include <stdint.h>
#include <stdbool.h>

#define NUM_OF_LEDS 64 
#define BYTES_PER_LED 3 
#define MATRIX_SIZE 8

#define HIGH_BIT 58 //if our pwm period is 90, 64%(90)=57.6 close to 58
#define LOW_BIT 29  //if our pwm period is 90, 32%(90)=28.8 close to 29

typedef enum rotation_t {
	ROTATION_0 = 0,
	ROTATION_90 = 1,
	ROTATION_180 = 2,
	ROTATION_270 = 3
} rotation_t;

typedef enum rgb_mode_t {
	OFF = 0,
	LIMITS = 1
} rgb_mode_t;

typedef enum show_mode_t{
	POINTS = 0,
	LINES = 1
}show_mode_t;

void rgb_matrix_clear_buffer(uint8_t *buffer, uint16_t bytenumber); 
void rgb_process_colors(rgb_mode_t mode, uint8_t* colors, uint8_t amplitude);
void rgb_process_rotation(rotation_t rotation, uint8_t* coordinates, uint8_t x, uint8_t y);
uint32_t rgb_matrix_set_pixel(uint8_t *buffer, rgb_mode_t mode, uint8_t x, uint8_t y, rotation_t rotation);
void matrix_test_secuential(rotation_t rotation);
void matrix_test_pyramid(rotation_t rotation);
void matrix_draw_vertical_line(uint8_t x, uint8_t y1, uint8_t y2, rotation_t rotation);
void matrix_test_vertical_levels(rotation_t rotation, show_mode_t mode);

#endif