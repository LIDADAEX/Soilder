#include "usbd_cdc_if.h"
#include "stdint.h"
__asm(".global __use_no_semihosting\n\t");

static uint8_t putchar_buff[128];
static uint16_t putchar_length = 0;
static uint32_t putchar_startTime;

void stdout_putchar(char ch)
{

  if (putchar_length == 0)
  {
    putchar_startTime = HAL_GetTick();
  }

  putchar_buff[putchar_length] = ch;

  putchar_length++;

  if (putchar_length >= 128)
  {
    while (CDC_Transmit_FS(putchar_buff, putchar_length) == USBD_BUSY);
    putchar_length = 0;
  }
}

void check_putchar()
{
  if (putchar_length != 0 && (HAL_GetTick() - putchar_startTime >= 10))
  {
    if (CDC_Transmit_FS(putchar_buff, putchar_length) != USBD_BUSY)
      putchar_length = 0;
  }
}

void ttywrch(int ch)
{
  uint8_t data = (uint8_t)ch;
  while (CDC_Transmit_FS(&data, 1) == USBD_BUSY);
}

void _sys_exit(int x)
{
  while (1);
}