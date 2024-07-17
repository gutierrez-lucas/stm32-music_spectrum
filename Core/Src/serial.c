#include <stdio.h>
#include "serial.h"
#include "main.h"
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

UART_HandleTypeDef huart1;

int _write(int file, char *data, int len){
	 if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)){
			errno = EBADF;
			return -1;
	 }
	 HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);

	 return (status == HAL_OK ? len : 0);
}

void serial_init(){
    MX_USART1_UART_Init();
}

void MX_USART1_UART_Init(void){
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart1) != HAL_OK){
		Error_Handler();
	}
	
}