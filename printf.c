#include <libopencm3/stm32/usart.h>
#include "hwdefs.h"
#include <stdio.h>
#include <errno.h>

int _write(int file, char *ptr, int len)
{
   int i;

   if (file == 1) {
      for (i = 0; i < len; i++)
         usart_send_blocking(TERM_USART, ptr[i]);
      return i;
   }

   errno = EIO;
   return -1;
}