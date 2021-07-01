/*
 * File: main.h
 *
 * Copyright (c) 2011, Kustaa Nyholm / SpareTimeLabs
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

// Turn off PIC extended instruction set, nobody supports it properly
#pragma config XINST=OFF

#include "types.h"
#include <pic18fregs.h>
#include "usb_defs.h"
#include "toad4.h"
#include "pic-config.c"
#include "usb_core.h"
#include "command_queue.h"
#include "usb_hid.h"
#include "usb_pic_defs.h"
#include "swuart.h"
#include <string.h> // memcpyram2ram, memcpypgm2ram

#define HOME_0 HOME_X
#define HOME_1 HOME_Y
#define HOME_2 HOME_Z
#define HOME_3 HOME_A
#define HOME_4 HOME_B

#define ENABLE_0 ENABLE_X
#define ENABLE_1 ENABLE_Y
#define ENABLE_2 ENABLE_Z
#define ENABLE_3 ENABLE_A
#define ENABLE_4 ENABLE_B

/*
#define TORQUE_0 TORQUE_
#define TORQUE_1 TORQUE_Y
#define TORQUE_2 TORQUE_Z
#define TORQUE_3 TORQUE_A
*/

#define ENABLE_MOTOR(i,e) do { \
	ENABLE_##i = e; \
	} while (0)

#define CMD_MOVE 1
#define CMD_RESET_POSITION 2
#define CMD_REINIT_MOTOR 3
#define CMD_RESET_MOTOR 4

typedef union {
	uint8_t uint8;
	struct {
		unsigned bit_0 :1;
		unsigned bit_1 :1;
		unsigned bit_2 :1;
		unsigned bit_3 :1;
		unsigned bit_4 :1;
		unsigned bit_5 :1;
		unsigned bit_6 :1;
		unsigned bit_7 :1;
	};
} uint8bits_t;

typedef struct {
	int32_t position;
	uint8_t queue_state;
	unsigned home :1; // bit 0
	unsigned obs_reserved_1 :1;
	unsigned obs_reserved_2 :1;
	unsigned obs_reserved_3 :1;
	unsigned obs_reserved_4 :1;
	unsigned obs_reserved_5 :1;
	unsigned obs_reserved_6 :1;
	unsigned obs_reserved_7 :1; // bit 7
	uint8_t obs_sync_counter;

	unsigned obs_ready :1; // bit 0
	unsigned obs_ready2 :1;
	unsigned obs_not_busy :1;
	unsigned obs_not_busy2 :1;
	unsigned obs_not_empty :1;
	unsigned obs_reserved_8 :1;
	unsigned obs_in_sync :1;
	unsigned obs_reserved_9 :1; // bit 7
} stepper_status_t; // dev => host status

typedef struct {
	uint32_t debug_position;
	uint8_t debug[4];
	uint8_t printf[8];
	stepper_status_t steppers[6];
} toad4_status_t;

static uint8_t g_group_masks[NUMBER_OF_MOTORS + 1];

static uint8_t g_blink_speed = 0;

static uint8_t g_led_toggle_counter = 0;

static uint16_t g_blink_counter = 0;

static uint8_t g_special_request = 0;

static uint8_t g_RCON;

static union {
	uint8_t uint8;
	struct {
		unsigned probe_input :1; // bit 0
		unsigned probe_triggered :1;
		unsigned probe_armed_trig_on_1 :1;
		unsigned probe_armed_trig_on_0 :1;
		unsigned bit_4 :1;
		unsigned bit_5 :1;
		unsigned bit_6 :1;
		unsigned bit_7 :1;
	};
} g_probe;

static union {
	uint8_t uint8;
	struct {
		unsigned arc_transfer :1; // bit 0
		unsigned bit_1 :1;
		unsigned bit_2 :1;
		unsigned bit_3 :1;
		unsigned bit_4 :1;
		unsigned bit_5 :1;
		unsigned bit_6 :1;
		unsigned bit_7 :1;
	};
} g_digital_inputs;

static volatile toad4_status_t g_toad4_status; // this gets sent to host in toto

static volatile uint8_t g_low_pri_isr_guard;

static uint8_t g_sync_counter[NUMBER_OF_MOTORS] = { 0 };

static uint8_t g_message_id = 0;

//extern volatile uint8_t g_pwm_out; // declared in hi_speed_irq.asm

static volatile uint8_t g_ADCresult;

extern uint8_t g_hipri_int_flags;

extern uint8_t g_lopri_int_flags;

volatile uint8_t g_uart_msg[2];

volatile uint8_t g_uart_msg_byte = 0;

volatile uint8_t g_manmode_switches;

volatile uint8_t g_manmode_potentiometer;

volatile uint8_t g_manmode_on;

#define BITCPY(d,s) do {if (s) d=1; else d=0;} while(0)

#define STEPPER(i) g_stepper_states[i]

#define INIT_MOTOR(i) do { \
	g_toad4_status.steppers[i].position = 0; \
	g_stepper_states[i].next_dir = 0; \
	g_stepper_states[i].next_forward = 0; \
	g_stepper_states[i].next_reverse = 1; \
	g_stepper_states[i].next_steps = 0; \
	g_stepper_states[i].last_steps = 0; \
	g_stepper_states[i].steps = 0; \
	g_stepper_states[i].speed = 0; \
	g_stepper_states[i].position = 0; \
	g_stepper_states[i].sync_group = i; \
	g_stepper_states[i].busy_mask = 0xFF; \
	g_stepper_states[i].group_mask = 0xFF; \
	QUEUE_CLEAR(i); \
} while(0)

#define RESET_MOTOR(i) do { \
	g_toad4_status.steppers[i].position = 0; \
	g_stepper_states[i].next_dir = 0; \
	g_stepper_states[i].next_forward = 0; \
	g_stepper_states[i].next_reverse = 1; \
	g_stepper_states[i].next_steps = 0; \
	g_stepper_states[i].last_steps = 0; \
	g_stepper_states[i].steps = 0; \
	g_stepper_states[i].speed = 0; \
	g_stepper_states[i].sync_group = i; \
	g_stepper_states[i].busy_mask = 0xFF; \
	g_stepper_states[i].group_mask = 0xFF; \
	QUEUE_CLEAR(i); \
} while(0)

#define UPDATE_POS(i) do { \
	if (STEPPER(i).update_pos){ \
		STEPPER(i).update_pos = 0; \
		if (STEPPER(i).next_dir) \
			STEPPER(i).position += STEPPER(i).last_steps; \
		else \
			STEPPER(i).position -= STEPPER(i).last_steps; \
		STEPPER(i).last_steps = 0; \
		}\
	}while(0)

#define FEED_MOREX(i) do { \
		}while(0)

#define UPDATE_NOT_EMPTY(i) do { \
	if (QUEUE_EMPTY(i)) \
		g_not_empty_flags.not_empty_##i = 0; \
	else \
		g_not_empty_flags.not_empty_##i = 1; \
	}while(0)

extern uint8_t g_debug_count;
#define FEED_MORE(i) do { \
		if ((g_lopri_int_flags & STEPPER(i).group_mask) == 0) { \
			int16_t distance = QUEUE_FRONT(i)->move_distance;  \
			if (distance < 0) { \
				if (STEPPER(i).next_dir) { \
					STEPPER(i).next_dir = 0; \
					STEPPER(i).next_forward = 0; \
					STEPPER(i).next_reverse = 1; \
					} \
				distance = -distance; \
				} \
			else if (distance > 0) { \
				if (!STEPPER(i).next_dir) { \
					STEPPER(i).next_dir = 1; \
					STEPPER(i).next_forward = 1; \
					STEPPER(i).next_reverse = 0; \
				} \
			} \
			if (distance > 255) \
				distance = 255; \
			STEPPER(i).next_steps = distance; \
			STEPPER(i).last_steps = distance; \
			STEPPER(i).update_pos = 1; \
			STEPPER(i).next_speed = QUEUE_FRONT(i)->move_speed; \
			if (STEPPER(i).next_dir) \
				QUEUE_FRONT(i)->move_distance -= distance; \
			else \
				QUEUE_FRONT(i)->move_distance += distance; \
			if (QUEUE_FRONT(i)->move_distance == 0) \
				g_pop_flags.pop_##i = 0; \
			/* g_not_empty_flags.not_empty_##i = 1; */ \
			g_ready_flags.ready_##i = 0; \
		} \
	}while(0)

#define POP_QUEUE(i) do { \
		if ((g_pop_flags.pop_bits & STEPPER(i).group_mask) == 0)  \
			QUEUE_POP(i); \
		} while(0)

#define UPDATE_HOME(i) do { \
	if (STEPPER(i).seek_home || STEPPER(i).seek_not_home) { \
		if (STEPPER(i).seek_home && HOME_##i) {\
			STEPPER(i).seek_home = 0; \
			break; \
			} \
		if (STEPPER(i).seek_not_home && !HOME_##i) { \
			STEPPER(i).seek_not_home = 0; \
			break; \
			} \
		if (STEPPER(i).seek_reverse) \
			STEPPER(i).jog_reverse = 1; \
		else \
			STEPPER(i).jog_reverse = 0; \
		STEPPER(i).jog_on = 1; \
		} \
	}while(0)

#define PROCESS_MOTOR_COMMANDS(i)  do { \
	uint8_t cmd = hid_rx_buffer.uint8[i*8+0]; \
	uint8_t flags = hid_rx_buffer.uint8[i*8+1]; \
	if (flags & 0x10) \
		ENABLE_MOTOR(i,1); \
	else \
		ENABLE_MOTOR(i,0); \
	if (cmd == CMD_MOVE) { \
		/* uint8_t dist = hid_rx_buffer.uint8[i*8+2]; */ \
		uint16_t dist = hid_rx_buffer.uint16[i*4+1]; \
		/* uint8_t dir = hid_rx_buffer.uint8[i*8+3]; */ \
		uint16_t speed = hid_rx_buffer.uint16[i*4+2]; \
		/* QUEUE_REAR(i)->move_dir = dir; */ \
		QUEUE_REAR(i)->move_distance = dist; \
		QUEUE_REAR(i)->move_speed = speed; \
		QUEUE_PUSH(i); \
	} else if (cmd == CMD_RESET_POSITION) { \
		STEPPER(i).position = hid_rx_buffer.int32[i*2+1]; \
	} else if (cmd == CMD_REINIT_MOTOR) { \
		INIT_MOTOR(i); \
	} else if (cmd == CMD_RESET_MOTOR) { \
		RESET_MOTOR(i); \
		} \
	}while(0)

#define UPDATE_STATUS(i)  do { \
	if (!g_probe.probe_triggered) \
		g_toad4_status.steppers[i].position = get_position(i); \
	g_toad4_status.steppers[i].queue_state =QUEUE_SIZE(i)+(QUEUE_CAPACITY<<4); \
	if (g_busy_flags.busy_##i) \
		g_toad4_status.steppers[i].queue_state++; \
	BITCPY(g_toad4_status.steppers[i].home , HOME_##i); \
	}while(0)

#define UPDATE_SYNC_GROUP(i) \
	STEPPER(i).sync_group = hid_rx_buffer.uint8[i*8+1] & 0x07

#define UPDATE_GROUP_MASK_1(i) \
		g_group_masks[i] = 0

#define UPDATE_GROUP_MASK_2(i) \
		g_group_masks[STEPPER(i).sync_group] |= 1 << i

#define UPDATE_GROUP_MASK_3(i) \
		STEPPER(i).group_mask = g_group_masks[STEPPER(i).sync_group]

void UPDATE_OUTPUTS(unsigned char x)   {
	if ((x) & 0x01)
		SPINDLE_FWD = 1;
	else
		SPINDLE_FWD = 0;
	if ((x) & 0x02)
		SPINDLE_REV = 1;
	else
		SPINDLE_REV = 0;
	if ((x) & 0x04)
		COOLANT = 1;
	else
		COOLANT = 0;
#if TOAD_HW_VERSION== HW5
	if ((x) & 0x08)
		ARC_START = 1;
	else
		ARC_START = 0;
#endif
	}

// 0-17 => 0
// 18 => 0
// 255 => 948
#define UPDATE_PWM(x) do {\
	uint16_t x16 = 0; \
	if ((x)>0) \
		x16=35+x*62/16; \
	CCPR2L = x16>>2; \
	CCP2CON = (CCP2CON & 0xCF) | ((x16 << 4) & 0x30); \
	} while(0)

// To change number of motors supported you need to
// 1) change this macro
// 2) possibly adjust the hi-speed interrupt frequency
// 3) update the number of motor in the hi speed interrupt asm code where it is hard coded

#if NUMBER_OF_MOTORS==3
#define FOR_EACH_MOTOR_DO(DO_THIS_FOR_ONE_MOTOR) do { \
		DO_THIS_FOR_ONE_MOTOR(0); \
		DO_THIS_FOR_ONE_MOTOR(1); \
		DO_THIS_FOR_ONE_MOTOR(2); \
		}while(0)
#endif

#if NUMBER_OF_MOTORS==4
#define FOR_EACH_MOTOR_DO(DO_THIS_FOR_ONE_MOTOR) do { \
		DO_THIS_FOR_ONE_MOTOR(0); \
		DO_THIS_FOR_ONE_MOTOR(1); \
		DO_THIS_FOR_ONE_MOTOR(2); \
		DO_THIS_FOR_ONE_MOTOR(3); \
		}while(0)
#endif

#if NUMBER_OF_MOTORS==5
#define FOR_EACH_MOTOR_DO(DO_THIS_FOR_ONE_MOTOR) do { \
		DO_THIS_FOR_ONE_MOTOR(0); \
		DO_THIS_FOR_ONE_MOTOR(1); \
		DO_THIS_FOR_ONE_MOTOR(2); \
		DO_THIS_FOR_ONE_MOTOR(3); \
		DO_THIS_FOR_ONE_MOTOR(4); \
		}while(0)
#endif

#if NUMBER_OF_MOTORS==6
#define FOR_EACH_MOTOR_DO(DO_THIS_FOR_ONE_MOTOR) do { \
		DO_THIS_FOR_ONE_MOTOR(0); \
		DO_THIS_FOR_ONE_MOTOR(1); \
		DO_THIS_FOR_ONE_MOTOR(2); \
		DO_THIS_FOR_ONE_MOTOR(3); \
		DO_THIS_FOR_ONE_MOTOR(4); \
		DO_THIS_FOR_ONE_MOTOR(5); \
		}while(0)
#endif

#pragma save
#pragma nooverlay

int32_t get_position(uint8_t i) {
	uint8_t stp;
	int32_t pos;
	do { // loop until we have a reading not disturbed by the lo priority interrupt
		stp = g_low_pri_isr_guard;
		pos = g_stepper_states[i].position;
	} while (stp != g_low_pri_isr_guard);
	return pos;
}
#pragma restore

#pragma save
#pragma nojtbound
#pragma nooverlay

typedef union {
	uint16_t as_uint16;
	struct {
		uint8_t as_uint8_lo;
		uint8_t as_uint8_hi;
	};
} uint8uint16_t;

volatile uint8_t g_pwm_toggle = 0;
volatile uint16_t g_pwm_lo = 2400;
volatile uint16_t g_pwm_hi = 2400;
volatile uint8uint16_t g_ccp2_next;
volatile uint8uint16_t g_tmr3;
char rxbyte[4]={0};
static unsigned char rxcnt=0;

void low_priority_interrupt_service() __interrupt(2) {
	if (PIR2bits.CCP2IF) {
		g_low_pri_isr_guard++;
		PIR2bits.CCP2IF = 0;

		FOR_EACH_MOTOR_DO(UPDATE_NOT_EMPTY);
		g_lopri_int_flags = ~(g_ready_flags.ready_bits & g_not_empty_flags.not_empty_bits);
		g_pop_flags.pop_bits = 0xFF;
		FOR_EACH_MOTOR_DO(FEED_MORE);
		FOR_EACH_MOTOR_DO(POP_QUEUE);
		FOR_EACH_MOTOR_DO(UPDATE_POS);
	} // End of 'software' interrupt processing

	if (PIR2bits.TMR3IF) {
		PIR2bits.TMR3IF = 0;
	}

	if (INTCONbits.TMR0IF) {
		INTCONbits.TMR0IF = 0;
		swuart_tick(SW_UART);
	}
//	if (PIR2bits.CCP2IF) {
//		PIR2bits.CCP2IF = 0;
//	}

	if (PIR3bits.USBIF) {
		PIR3bits.USBIF = 0;
		usb_core_handler();
	}

 	if (PIR1bits.RC1IF) {
 		uint8_t temp = RCREG1;
		PIR1bits.RC1IF = 0;
 		if (temp==0xff && RCSTA1bits.RX9D==1) {
 			rxcnt=0;
 		} else {
 			if (rxcnt<4) {
 				rxbyte[rxcnt++]=temp;
 				}
 		}
 	}
}

#pragma restore

void enterBootloader() {
	__asm__ (" goto 0x0016\n");
}

#define TOGGLE_LED_PIN()  do {if (LED_PIN==LED_OFF) LED_PIN = LED_ON; else LED_PIN = LED_OFF; } while(0)
//#define TOGGLE_LED_PIN()  do {} while(0)

void blink_led() {
	static uint8_t old = 0;
	uint8_t new;
	//new = TMR0L; // need to read TMR0L to get TMR0H updated
	//new = TMR0H;
	new = g_uart_tick_cntr; // since uart tick increments when 8 bit TMR0 overflows this is equivalent to previous
	if (new != old) {
		old = new;
		g_blink_counter++;
		if (g_blink_speed == 0 && g_blink_counter > 732) {
			g_blink_counter = 0;
			TOGGLE_LED_PIN();
		}
		if (g_blink_speed == 1 && g_blink_counter > 732 / 2) {
			g_blink_counter = 0;
			g_blink_speed = 0;
			TOGGLE_LED_PIN();
		}
	}
}

void toggle_led() {
	g_blink_counter = 0;
	g_led_toggle_counter++;
	if ((g_led_toggle_counter & 3) == 0)
		TOGGLE_LED_PIN();
}

void check_for_firmware_update() {
	if ((hid_rx_buffer.uint8[0] == 0xFE) && (hid_rx_buffer.uint8[1] == 0xED) && (hid_rx_buffer.uint8[2] == 0xC0) && (hid_rx_buffer.uint8[3] == 0xDE)) {
		hid_rx_buffer.uint8[63] = 0; // WHY?
		enterBootloader();
	}
}

void check_probe() {
	BITCPY(g_probe.probe_input, PROBE);
	if (g_probe.probe_input & g_probe.probe_armed_trig_on_1)
		g_probe.probe_triggered = 1;
	if (!g_probe.probe_input & g_probe.probe_armed_trig_on_0)
		g_probe.probe_triggered = 1;
}

void handle_manual_mode() {
	if (g_uart_rx_ready) {
		g_uart_rx_ready = 0;
		// note that a new char arrives at 30 msec interval so we trust
		// that g_uart_rx_data does not change on us
		if (g_uart_rx_bit9 == 1) { // first byte
			if (g_uart_rx_data == 0xFF) {
				//LED_PIN=1;
				g_uart_msg_byte = 1;
			}
		} else { // rest of the bytes
			if (g_uart_rx_bit9 == 1)
				g_uart_msg_byte = 0;
			switch (g_uart_msg_byte++) {
			default:
				g_uart_msg_byte = 0;
				break;
			case 1:
				g_uart_msg[0] = g_uart_rx_data;
				break;
			case 2:
				if (g_uart_msg[0] != (uint8_t)(~g_uart_rx_data))
					g_uart_msg_byte = 0;
				break;
			case 3:
				g_uart_msg[1] = g_uart_rx_data;
				break;
			case 4:
				if (g_uart_msg[1] != (uint8_t)(~g_uart_rx_data)) {
					g_uart_msg_byte = 0;
					break;
				}
				if (((g_uart_msg[0] ^ (g_uart_msg[0] >> 4)) & 0xF) == 0xF) {
					//LED_PIN=0;
					// Got the full message, now process it
					g_manmode_potentiometer = g_uart_msg[1];
					UPDATE_PWM(g_manmode_potentiometer);
					g_manmode_switches = g_uart_msg[0] >> 4;
					SPINDLE_FWD = g_manmode_switches & 0x2;
					SPINDLE_REV = g_manmode_switches & 0x4;
					COOLANT = g_manmode_switches & 0x8;
				}
				break;
			}
		}
	}

}

void main(void) {
	// Capture the reset cause flags and clear them
	// FIXME, should we actually clear them not at startup (here)
	// but AFTER connecting to the host so that cycling MACH
	// in EazyCNC would clear the watcdog error/message?
	unsigned int i;
	ANSELA = 0x00;
 	TRISA = 0xC0;
	LATA = 0x15;


	g_RCON = RCON;
	RCON |= 0x1F;


	initIO();


	g_ready_flags.ready_bits = 0xFF;
	g_busy_flags.busy_bits = 0x00;
	g_not_empty_flags.not_empty_bits = 0x00;

	FOR_EACH_MOTOR_DO(INIT_MOTOR);
	FOR_EACH_MOTOR_DO(UPDATE_GROUP_MASK_1);
	FOR_EACH_MOTOR_DO(UPDATE_GROUP_MASK_2);
	FOR_EACH_MOTOR_DO(UPDATE_GROUP_MASK_3);



	// FIXME why do we configure RC6 which is USART TX as output, it is not used.
	// Was it for debugging?
	TRISCbits.TRISC6 = 0;

	T0CONbits.T0CS = 0; // Internal instruction clock as source for timer 0
	T0CONbits.PSA = 0;  // Enable prescaler for Timer 0
	T0CONbits.T08BIT = 1; //  8 bit

	T0CONbits.T0PS2 = 1; // prescaler 1:32 => 1464 Hz
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS0 = 0;

	T0CONbits.TMR0ON = 1; // XXX
	T0CONbits.T0SE = 1; // high to low (no effect when timer clock coming from instruction clock...)

	INTCON = 0; // Clear interrupt flag bits.

	TMR0L = 0;
	TMR0H = 0;

	T1CONbits.T1CKPS0 = 0;
	T1CONbits.T1CKPS1 = 0;
	T1CONbits.TMR1CS = 0;
	CCPTMRSbits.C1TSEL = 0; // CCP1 uses TMR1 (compares agains it)
	CCPR1H = 0;
	CCPR1L = 120; // 12 MHz/120 = 100 kHz
	CCP1CON = 0x0B; // special event trigger, set CCP1IF on match, reset timer
	T1CONbits.TMR1ON = 1;

	// NOTE! CCP2IF is used as software interrupt flag,
	// in the TMR2 PWM  mode/configuration it is not set or cleared by the hardware
	PIR2bits.CCP2IF = 1;

	// Ensure no interrupts are enabled at this point
	PIE1 = 0;
	PIE2 = 0;
	PIE3 = 0;

	// Ensure that all interrupts are low priority
	IPR1 = 0;
	IPR2 = 0;
	IPR3 = 0;

	// FIXME there are other than interrupt priority bits in these regs so
	// would be cleaner to set (clear) each one separately or relay on reset values
	INTCON = 0;
	INTCON2 = 0;
	INTCON3 = 0;

	// ... except make  CCP1 interrupt  high priority
	IPR1bits.CCP1IP = 1;	// was TMR2IP !!
	RCONbits.IPEN = 1; // enable priorities

	// TIMER 2 configuration
	T2CONbits.T2CKPS0 = 1;
	T2CONbits.T2CKPS1 = 1;
	PR2 = 255;
	CCPR2L = 25;
	CCP2CONbits.CCP2M = 0x0C; // PWM mode
	T2CONbits.TMR2ON = 1;

	T3GCONbits.TMR3GE = 0;

	T3CONbits.T3CKPS0 = 1;
	T3CONbits.T3CKPS1 = 1;
	T3CONbits.TMR3CS0 = 0;
	T3CONbits.TMR3CS1 = 0;
	T3CONbits.SOSCEN = 0;
	T3CONbits.NOT_T3SYNC = 0;
	T3CONbits.RD16 = 0;

	T3CONbits.TMR3ON = 1;

	CCPTMRSbits.C2TSEL = 1;

	// Make the PWM pin PB3 output
	TRISBbits.TRISB3 = 0;

	// Initialize the USB stack
	usb_core_init();


	// Turn the run LED on
	LED_PIN = LED_ON;


	// Init EUSART
    // ABDOVF no_overflow; CKTXP async_noninverted_sync_fallingedge; BRG16 16bit_generator; WUE disabled; ABDEN disabled; DTRXP not_inverted;
    BAUDCON1 = 0x08;

    // SPEN enabled; RX9 9-bit; CREN enabled; ADDEN disabled; SREN disabled;
    RCSTA1 = 0xD0;

    // TX9 9-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave_mode;
    TXSTA1 = 0x64;

    // SPBRG1 1249;
    SPBRG1 = 1249%256;

    // SPBRGH1 0;
    SPBRGH1 = 1249/256;




	// This seems to be as good a place as any to document interrupt usage
	// TMR2 interrupt = high priority, written in asm, running the NCOs (Numerically Controlled Oscillators)
	// CCP1 interrupt =  used to feed the high priority NCO interrupt
	// USB interrupt = low priority, runs the USB stack, what else
	// TMR3 interrupt = no funtion
	// CCP2 interrupt =  no function

	// Enable interrupts

	PIE1bits.CCP1IE = 1;
	PIE3bits.USBIE = 1;
	PIE2bits.CCP2IE = 1;
	INTCONbits.TMR0IE = 1;
	PIE1bits.RCIE = 1;


	INTCONbits.PEIE = 1; // enable peripheral interrupts
	INTCONbits.GIE = 1; // global interrupt enable

/*
	g_manmode_on = 1;
	TRISCbits.TRISC2 = 0; // dir=output
    LATCbits.LATC2 = 1;
	while (1) {
		unsigned int i=0;
		for (i=0; i<65535; i++)
			g_manmode_on=1;
		LATCbits.LATC2 = !LATCbits.LATC2;
		}
*/
	// Enable the watchdog
	WDTCONbits.SWDTEN = 1;

	while (1) {
		__asm__ (" CLRWDT  ");

        // trig sw interrupt so that the queues get updated
        PIR2bits.CCP2IF = 1;

		// If manual/cnc mode changes clear spindle and coolant
		if (g_manmode_on != (g_uart_connected != 0)) {
			g_manmode_on = !g_manmode_on;
			UPDATE_OUTPUTS(0);
			UPDATE_PWM(0);
		}
		if (g_manmode_on)
			handle_manual_mode();
		if (!(ep2_o.STAT & UOWN)) { // new data from host, so process it
			g_message_id = hid_rx_buffer.uint8[63];
			toggle_led();

			while (hid_rx_buffer.uint8[0] == 0x73) { // watchdog test
				// loops until watchdog reset
			}
			if (hid_rx_buffer.uint8[0] == 0xFE) { // special message
				check_for_firmware_update(); // may not ever return
				g_special_request = hid_rx_buffer.uint8[1];
			} else { // normal message
				g_special_request = 0;
				FOR_EACH_MOTOR_DO(UPDATE_SYNC_GROUP);
				FOR_EACH_MOTOR_DO(UPDATE_GROUP_MASK_1);
				FOR_EACH_MOTOR_DO(UPDATE_GROUP_MASK_2);
				FOR_EACH_MOTOR_DO(UPDATE_GROUP_MASK_3);

				FOR_EACH_MOTOR_DO(PROCESS_MOTOR_COMMANDS);

				UPDATE_OUTPUTS(hid_rx_buffer.uint8[50]);
				UPDATE_PWM(hid_rx_buffer.uint8[52]);

				BITCPY(g_probe.probe_armed_trig_on_1, 0x01 & hid_rx_buffer.uint8[49]);
				BITCPY(g_probe.probe_armed_trig_on_0, 0x02 & hid_rx_buffer.uint8[49]);
				if (!g_probe.probe_armed_trig_on_1 && !g_probe.probe_armed_trig_on_0)
					g_probe.probe_triggered = 0;

			}
			// turn the buffer over to SIE so we can get more data
			ep2_o.CNT = 64;
			if (ep2_o.STAT & DTS)
				ep2_o.STAT = DTSEN;
			else
				ep2_o.STAT = DTS | DTSEN;
			ep2_o.STAT |= UOWN;
		}

		// update toad4 status, we do this all the time so as to be ready when we can send it over

		check_probe();

		FOR_EACH_MOTOR_DO(UPDATE_STATUS);

		if (ADCON0bits.GO_NOT_DONE == 0) {
			g_ADCresult = ADRESH;
			ADCON0bits.GO_NOT_DONE = 1;
		}

		if (!(ep2_i.STAT & UOWN)) {	// we own the USB buffer, so update data going to the host
			g_blink_speed = 1;
			hid_tx_buffer.uint8[63] = g_message_id;

			if (g_special_request == 0x01) {
				memcpypgm2ram(&hid_tx_buffer.uint8[0], get_toad4_config(), 63);
				hid_tx_buffer.uint8[62] = g_RCON;
				g_RCON = RCON;
			} else {
				uint8_t updf = 0;

				// FIXME make structure g_toad4_status so that we can use a single copy
				// or see if it would be better to create a memcpyram2ram that move 8 bytes
				// with direct move commands and no looping
				memcpyram2ram(&hid_tx_buffer.uint8[0], &g_toad4_status.steppers[0], 8);
				memcpyram2ram(&hid_tx_buffer.uint8[8], &g_toad4_status.steppers[1], 8);
				memcpyram2ram(&hid_tx_buffer.uint8[16], &g_toad4_status.steppers[2], 8);
				memcpyram2ram(&hid_tx_buffer.uint8[24],	&g_toad4_status.steppers[3], 8);
				memcpyram2ram(&hid_tx_buffer.uint8[32],	&g_toad4_status.steppers[4], 8);

				// FIXME, see if above and below could be combined to a single memcpyram2ram by
				// rearranging the structures
                /* DEBUG STUFF
				hid_tx_buffer.uint8[32] = g_stepper_states[0].steps;
				hid_tx_buffer.uint8[33] = g_stepper_states[0].last_steps;
				hid_tx_buffer.uint8[34] = g_stepper_states[0].next_steps;
				hid_tx_buffer.uint8[35] = g_stepper_states[0].update_pos;

				hid_tx_buffer.uint8[36] = g_stepper_states[1].steps;
				hid_tx_buffer.uint8[37] = g_stepper_states[1].last_steps;
				hid_tx_buffer.uint8[38] = g_stepper_states[1].next_steps;
				hid_tx_buffer.uint8[39] = g_stepper_states[1].update_pos;

				hid_tx_buffer.uint8[40] = g_stepper_states[2].steps;
				hid_tx_buffer.uint8[41] = g_stepper_states[2].last_steps;
				hid_tx_buffer.uint8[42] = g_stepper_states[2].next_steps;
				hid_tx_buffer.uint8[43] = g_stepper_states[2].update_pos;

				hid_tx_buffer.uint8[44] = g_stepper_states[3].steps;
				hid_tx_buffer.uint8[45] = g_stepper_states[3].last_steps;
				hid_tx_buffer.uint8[46] = g_stepper_states[3].next_steps;
				hid_tx_buffer.uint8[47] = g_stepper_states[3].update_pos;
                */
				hid_tx_buffer.uint8[49] = g_probe.uint8; // & 0x03; //
#if TOAD_HW_VERSION== HW5
				g_digital_inputs.arc_transfer = ARC_TRANSFER;
#endif
				hid_tx_buffer.uint8[50] = g_digital_inputs.uint8;

				hid_tx_buffer.uint8[51] = 0; // digital inputs 8-15
				hid_tx_buffer.uint8[52] = rxbyte[0]; // g_ADCresult; // analog input 0
				if (g_manmode_on) {
					hid_tx_buffer.uint8[53] = g_manmode_switches;
					hid_tx_buffer.uint8[54] = g_manmode_potentiometer;
				} else {
					hid_tx_buffer.uint8[53] = 0;
					hid_tx_buffer.uint8[54] = 0;
				}
				hid_tx_buffer.uint8[55] = rxbyte[0]; //rxbyte[0];// STEPPER(0).group_mask;
				hid_tx_buffer.uint8[56] = rxbyte[1];//STEPPER(1).group_mask;
				hid_tx_buffer.uint8[57] = rxbyte[2];//STEPPER(2).group_mask;
				hid_tx_buffer.uint8[58] = rxbyte[3];//STEPPER(3).group_mask;
				hid_tx_buffer.uint8[59] = g_debug_count;
				hid_tx_buffer.uint8[60] = g_not_empty_flags.not_empty_bits;
				hid_tx_buffer.uint8[61] = g_busy_flags.busy_bits;
				hid_tx_buffer.uint8[62] = g_ready_flags.ready_bits;
			}
			// turn the buffer over to the SIE so the host will pick it up
			ep2_i.CNT = 64;
			if (ep2_i.STAT & DTS)
				ep2_i.STAT = DTSEN;
			else
				ep2_i.STAT = DTS | DTSEN;
			ep2_i.STAT |= UOWN;
		}

    if(1 == RCSTA1bits.OERR)
    {
        // EUSART1 error - restart

        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1;
    }
	if (PIR1bits.TX1IF)
		TXREG1 =0x00;
	blink_led();
	}
}
