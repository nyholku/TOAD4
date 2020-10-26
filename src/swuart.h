#ifndef __SWUART_H__
#define __SWUART_H__

#include <stdint.h>

extern volatile uint8_t g_uart_rx_data;
extern volatile uint8_t g_uart_rx_bit9;
extern volatile uint8_t g_uart_rx_ready;
extern volatile uint8_t g_uart_connected;
extern volatile uint8_t g_uart_tick_cntr;

void swuart_tick(uint8_t input);




#endif
