#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define TERM_USART USART3
#define TERM_USART_TXPIN GPIO_USART3_TX
#define TERM_USART_TXPORT GPIOB
#define TERM_USART_RXPIN GPIO_USART3_RX
#define TERM_USART_RXPORT GPIOB
#define OVER_CUR_TIMER TIM4
#define USART_BAUDRATE  115200

#endif // HWDEFS_H_INCLUDED
