#include "stm8s.h"
#include <stdio.h>
#include "bootstrap.h"

void main(void)
{
	SimpleInit();

	// clear screen, hide cursor
	printf("\033c\033[?25lClock freq = \033[32m%lu Hz\033[0m\r\n", CLK_GetClockFreq());

	while (1) {
		LED_Toggle();
		printf("Time = %lu ms\r", time_ms);
		Delay(500);
	}
}
