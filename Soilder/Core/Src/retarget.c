#include "stm32f4xx_hal.h"

__asm(".global __use_no_semihosting\n\t");

extern UART_HandleTypeDef huart6;

void stdout_putchar(char ch)
{	
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1, 10);
}

void ttywrch(int ch)
{
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1, 10);
}

void _sys_exit(int x)
{
  while (1);
}