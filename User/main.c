#include <stdio.h>

#include <stm8s.h>
#include "stm8s_it.h"

void main(void)
{
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	UART_SimpleInit(UART_BAUD_115200);

	while(1);
}
