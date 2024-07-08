#ifndef ____TASK_UTILS_H____
#define ____TASK_UTILS_H____

#include "main.h"

void trace_off(int tag);
void trace_on(int tag);
void trace_toggle(int tag);

#define IDLE_TASK_TAG	 0 
#define DISPLAY_TASK_TAG 1
#define PROCESS_TASK_TAG	 2
#define AUXILIAR_TAG_1	 3
#define AUXILIAR_TAG_2	 4

#endif