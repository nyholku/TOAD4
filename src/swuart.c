#include "swuart.h"
#include "toad4.h"

volatile uint8_t g_uart_cnt = 0;
volatile uint8_t g_uart_in;
volatile uint8_t g_uart_rx_data;
volatile uint8_t g_uart_rx_bit9;
volatile uint8_t g_uart_rx_ready = 0;
volatile uint8_t g_uart_connected = 0;
volatile uint8_t g_uart_tick_cntr;

void swuart_tick(uint8_t input) {
	g_uart_tick_cntr++;

	if (input==1) {
	   if (g_uart_connected > 0)
		  g_uart_connected--;
        }
    else
		g_uart_connected = 255;

	if (g_uart_connected > 0) {

		//LED_PIN = 0;
		if (g_uart_cnt == 0) { // wait for falling edge
			if (input == 0) {
				g_uart_cnt = 1;
			}
		} else
			g_uart_cnt++;
		if (g_uart_cnt >= 4) {
			if (g_uart_cnt <= 10 * 4) { // if data bit 0..8
				if ((g_uart_cnt & 0x3) == 3) { // sample position 2 or 3
					g_uart_rx_data = g_uart_in;
					g_uart_rx_bit9 = input;
					g_uart_in >>= 1;
					if (input == 1)
						g_uart_in |= 0x80;
				}
			} else if (g_uart_cnt >= 11 * 4 - 3)  { // stop bit
				g_uart_rx_ready = 1;
				g_uart_cnt = 0;
			}
		}

	}
}
