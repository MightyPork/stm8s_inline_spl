//
// Created by MightyPork on 2017/02/10.
//

#ifndef STM8S_STDINIT_H
#define STM8S_STDINIT_H

/** Global timebase */
extern volatile uint32_t time_ms;

/** Uart IRQ handler */
void UART1_RX_IRQHandler(void) INTERRUPT(18);

/** SysTick handler */
void TIM4_UPD_OVF_IRQHandler(void) INTERRUPT(23);

/** putchar, used by the SDCC stdlib */
void putchar(char c);

/**
 * Simple init (UART, LED, timebase)
 */
void SimpleInit(void);

/**
 * Millisecond delay
 *
 * @param ms - nr of milliseconds
 */
void Delay(uint16_t ms);

/**
 * User UART rx handler
 *
 * If adding custom handler, comment out the defualt echo impl in bootstrap.c
 *
 * @param c
 */
extern void UART_HandleRx(char c);

/** Toggle indicator LED */
inline void LED_Toggle(void)
{
	GPIOB->ODR ^= GPIO_PIN_5;
}

/** Set indicator LED */
inline void LED_Set(bool state)
{
	if (state) {
		GPIOB->ODR &= ~GPIO_PIN_5;
	} else {
		GPIOB->ODR |= GPIO_PIN_5;
	}
}


#endif //STM8S_DEBUG_H
