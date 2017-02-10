#include "stm8s.h"
#include <stdlib.h>

void Delay(uint16_t nCount) {
	uint8_t i;
	for (; nCount != 0; nCount--) {
		for (i = 255; i != 0; i--) {}
	}
}

void putchar(char c) {
	while ((UART1->SR & UART1_SR_TXE) == 0);
	UART1->DR = (u8)c;
}

void puts(const char *ch) {
	char c;
	while ((c = *ch++) != 0)
		putchar(c);
}

void puts_itoa(int32_t n, unsigned char radix) {
	char s[10], i, c;
	_ltoa(n, s, radix);
	i = 0;
	while((c = s[i++]) != 0) {
		putchar(c);
	}
}

void main(void)
{
	// Disable div8
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

	// LED for blinking
	GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);

	// minimal uart init
	UART_SimpleInit(UART_BAUD_115200);

	// irq conf
	UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
	enableInterrupts();

	// Clear screen & print system frequency
	puts("\033c\033[?25lClock freq = "); // cls
	puts_itoa(CLK_GetClockFreq(), 10);
	puts(" Hz"); // cls

	// echo & blinking
	while (1) {
		Delay(2000);
		GPIOB->ODR ^= GPIO_PIN_5;
	}
}

/**
  * @brief UART1 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
	if (UART1->SR & UART1_SR_RXNE)
		UART1->DR = (u8) (UART1->DR); // echo

	if (UART1->SR & UART1_SR_OR)
		UART1->SR &= ~UART1_SR_OR; // clear OR flag
}
