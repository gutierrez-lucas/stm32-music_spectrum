#ifndef __SERIAL__H
#define __SERIAL__H
#include <stdbool.h>
void serial_init();
int _write(int file, char *data, int len);
#endif