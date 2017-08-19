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
#include <string.h> // memcpyram2ram, memcpypgm2ram

#define HOME_0 HOME_X
#define HOME_1 HOME_Y
#define HOME_2 HOME_Z
#define HOME_3 HOME_A

#define ENABLE_0 ENABLE_X
#define ENABLE_1 ENABLE_Y
#define ENABLE_2 ENABLE_Z
#define ENABLE_3 ENABLE_A

#define TORQUE_0 TORQUE_X
#define TORQUE_1 TORQUE_Y
#define TORQUE_2 TORQUE_Z
#define TORQUE_3 TORQUE_A

#define ENABLE_MOTOR(i,e) do { \
	ENABLE_##i = e; \
	TORQUE_##i = e; \
	} while (0)

#define CMD_MOVE 1
#define CMD_RESET_POSITION 2

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
	unsigned reserved_1 :1;
	unsigned reserved_2 :1;
	unsigned reserved_3 :1;
	unsigned reserved_4 :1;
	unsigned reserved_5 :1;
	unsigned reserved_6 :1;
	unsigned reserved_7 :1; // bit 7
	uint8_t sync_counter;

	unsigned ready :1; // bit 0
	unsigned ready2 :1;
	unsigned not_busy :1;
	unsigned not_busy2 :1;
	unsigned not_empty :1;
	unsigned reserved_8 :1;
	unsigned in_sync :1;
	unsigned reserved_9 :1; // bit 7
} stepper_status_t; // dev => host status

typedef struct {
	uint32_t debug_position;
	uint8_t debug[4];
	uint8_t printf[8];
	stepper_status_t steppers[6];
} toad4_status_t;


static uint8_t g_blink_speed = 0;

static uint8_t g_led_toggle_counter = 0;

static uint16_t g_blink_counter = 0;

static uint8_t g_special_request = 0;

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

static volatile toad4_status_t g_toad4_status; // this gets sent to host in toto

static volatile uint8_t g_low_pri_isr_guard;

static uint8_t g_sync_counter[NUMBER_OF_MOTORS] = { 0 };

static uint8_t g_message_id = 0;

extern volatile uint8_t g_pwm_out; // declared in hi_speed_irq.asm

static uint8_t g_ADCresult;

#define BITCPY(d,s) do {if (s) d=1; else d=0;} while(0)

#define STEPPER(i) g_stepper_states[i]

#define INIT_MOTOR(i) do { \
	g_toad4_status.steppers[i].position = 0; \
	g_stepper_states[i].has_last = 0; \
	g_stepper_states[i].next_dir = 0; \
	g_stepper_states[i].last_dir = 0; \
	g_stepper_states[i].next_steps = 0; \
	g_stepper_states[i].last_steps = 0; \
	g_stepper_states[i].ready = 1; \
	g_stepper_states[i].ready2 = 1; \
	g_stepper_states[i].not_busy = 1; \
	g_stepper_states[i].not_busy2 = 1; \
	g_stepper_states[i].in_sync = 1; \
	g_stepper_states[i].steps = 0; \
	g_stepper_states[i].speed = 0; \
	g_stepper_states[i].position = 0; \
	g_stepper_states[i].sync_group = 0; \
	g_stepper_states[i].state= 0; \
	QUEUE_CLEAR(i); \
} while(0)

#define UPDATE_POS(i) do { \
	if (STEPPER(i).ready && !STEPPER(i).ready2){ \
		STEPPER(i).ready2 = 1; \
		if (STEPPER(i).last_dir) \
			STEPPER(i).position += STEPPER(i).last_steps; \
		else \
			STEPPER(i).position -= STEPPER(i).last_steps; \
		BITCPY(STEPPER(i).last_dir,STEPPER(i).next_dir); \
		STEPPER(i).last_steps = STEPPER(i).next_steps; \
		}\
	if (STEPPER(i).not_busy && !STEPPER(i).not_busy2) { \
		STEPPER(i).not_busy2 = 1; \
		g_sync_counter[ STEPPER(i).sync_group ]--; \
		}\
	}while(0)

#define UPDATE_SYNC(i) do { \
		STEPPER(i).in_sync = g_sync_counter[ STEPPER(i).sync_group ] == 0; \
	}while(0)

#define FEED_MORE(i) do { \
	if (STEPPER(i).ready && !QUEUE_EMPTY(i) && STEPPER(i).in_sync) { \
		/* int16_t distance = QUEUE_FRONT(i)->move_distance; */ \
		int8_t distance = QUEUE_FRONT(i)->move_distance; \
		BITCPY(STEPPER(i).next_dir,QUEUE_FRONT(i)->move_dir); \
		STEPPER(i).next_steps = distance; \
		STEPPER(i).next_speed = QUEUE_FRONT(i)->move_speed; \
		if ((QUEUE_FRONT(i)->move_distance -= distance) == 0) { \
			QUEUE_POP(i); \
			} \
		g_sync_counter[ STEPPER(i).sync_group ]++; \
		STEPPER(i).not_busy2 = 0; \
		STEPPER(i).ready2 = 0; \
		STEPPER(i).ready = 0; \
		STEPPER(i).not_busy = 0; \
		} \
	}while(0)


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


#define READ_HOME(i) do {\
	if (HOME_##i) \
		g_toad4_status.steppers[i].home = 1; \
	else \
		g_toad4_status.steppers[i].home = 0; \
	}while(0)

#define UPDATE_JOG(i) do { \
	if (QUEUE_EMPTY(i) && (STEPPER(i).jog_on || STEPPER(i).jog_speed)) { \
		if (STEPPER(i).jog_on) { \
			STEPPER(i).jog_on = 0; \
			if (STEPPER(i).jog_speed < STEPPER(i).max_speed - STEPPER(i).max_accel) \
				STEPPER(i).jog_speed += STEPPER(i).max_accel; \
			else \
				STEPPER(i).jog_speed = STEPPER(i).max_speed; \
			} \
		else { \
			if (STEPPER(i).jog_speed > STEPPER(i).max_accel) \
				STEPPER(i).jog_speed -= STEPPER(i).max_accel; \
			else {\
				STEPPER(i).jog_speed = 0; \
				break; \
				} \
			} \
		{ \
			int16_t n = STEPPER(i).jog_speed_hi; /* jog_speed/256 */ \
			if (n==0) \
				n=1; \
			n <<= 3; \
			if (STEPPER(i).jog_reverse) \
 	 	 	 	n=-n; /* jog_speed/256*8 */\
			QUEUE_REAR(i)->move_distance = n; \
			QUEUE_REAR(i)->move_speed = STEPPER(i).jog_speed; \
			QUEUE_PUSH(i);	\
		}\
		} \
	}while(0)

#define UPDATE_PROBE(i) do { \
	if (g_probe) { \
		if (STEPPER(i).probeTrigOnRising) { \
			STEPPER(i).probeTriggered = 1; \
			STEPPER(i).probeTrigOnRising = 0; \
			} \
	} else { \
		if (STEPPER(i).probeTrigOnFalling) { \
			STEPPER(i).probeTriggered = 1; \
			STEPPER(i).probeTrigOnFalling = 0; \
			} \
		} \
	}while(0)


#define PROCESS_MOTOR_COMMANDS(i)  do { \
	uint8_t cmd = hid_rx_buffer.uint8[i*8+0]; \
	uint8_t flags = hid_rx_buffer.uint8[i*8+1]; \
	STEPPER(i).sync_group = flags & 0x0F; \
	if (flags & 0x10) \
		ENABLE_MOTOR(i,1); \
	else \
		ENABLE_MOTOR(i,0); \
	if (cmd == CMD_MOVE) { \
		uint8_t dist = hid_rx_buffer.uint8[i*8+2]; \
		uint8_t dir = hid_rx_buffer.uint8[i*8+3]; \
		uint16_t speed = hid_rx_buffer.uint16[i*4+2]; \
		QUEUE_REAR(i)->move_dir = dir; \
		QUEUE_REAR(i)->move_distance = dist; \
		QUEUE_REAR(i)->move_speed = speed; \
		QUEUE_PUSH(i); \
	}  else if (cmd == CMD_RESET_POSITION) { \
		STEPPER(i).position = hid_rx_buffer.int32[i*2+1]; \
	} \
	}while(0)


#define UPDATE_STATUS(i)  do { \
	if (!g_probe.probe_triggered) \
		g_toad4_status.steppers[i].position = get_position(i); \
	g_toad4_status.steppers[i].queue_state =QUEUE_SIZE(i)+(QUEUE_CAPACITY<<4); \
	if (!STEPPER(i).not_busy2) \
		g_toad4_status.steppers[i].queue_state++; \
	BITCPY(g_toad4_status.steppers[i].ready , STEPPER(i).ready); \
	BITCPY(g_toad4_status.steppers[i].ready2 , STEPPER(i).ready2); \
	BITCPY(g_toad4_status.steppers[i].not_busy , STEPPER(i).not_busy); \
	BITCPY(g_toad4_status.steppers[i].not_busy2 , STEPPER(i).not_busy2); \
	BITCPY(g_toad4_status.steppers[i].in_sync,STEPPER(i).in_sync); \
	BITCPY(g_toad4_status.steppers[i].not_empty , !QUEUE_EMPTY(i)); \
	BITCPY(g_toad4_status.steppers[i].home , HOME_##i); \
	g_toad4_status.steppers[i].sync_counter = g_sync_counter[ STEPPER(i).sync_group ]; \
	}while(0)


#define UPDATE_OUTPUTS(x)  do { \
	if ((x) & 0x01) \
		SPINDLE_FWD = 1; \
	else \
		SPINDLE_FWD = 0; \
	if ((x) & 0x02) \
		SPINDLE_REV = 1; \
	else \
		SPINDLE_REV = 0; \
	if ((x) & 0x04) \
		COOLANT = 1; \
	else \
		COOLANT = 0; \
	} while (0)

#define UPDATE_PWM_OUTPUT_0_480(x) do {\
		uint16_t v = (x); \
		uint16_t r = ((v << 4) - (v)) >> 5; \
		CCPR2L = r>>2; \
		CCP2CON = (CCP2CON & 0xCF) | ((r << 4) & 0x30); \
	} while(0)

#define UPDATE_PWM_OUTPUT(x) do {\
		uint16_t v = (x); \
		uint16_t r; \
		v = v << 3; \
		r = v; \
		v = v >> 1; \
		r += v; \
		v = v >> 1;\
		r += v;\
		v = v >> 1;\
		r += v; \
		r = r >> 5; \
		CCPR2L = r>>2; \
		CCP2CON = (CCP2CON & 0xCF) | ((r << 4) & 0x30); \
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
	uint8_t dlt;
	int32_t pos;
	uint8_t dir;
	do { // loop until we have a reading not disturbed by the lo priority interrupt
		stp = g_low_pri_isr_guard;
		dlt = g_stepper_states[i].last_steps - g_stepper_states[i].steps;
		pos = g_stepper_states[i].position;
		dir = g_stepper_states[i].last_dir;
	} while (stp != g_low_pri_isr_guard);
	if (dir)
		pos += dlt;
	else
		pos -= dlt;
	return pos;
}
#pragma restore

#pragma save
#pragma nojtbound
#pragma nooverlay

typedef
	union{
		uint16_t as_uint16;
		struct {
			uint8_t as_uint8_lo;
			uint8_t as_uint8_hi;
		};
	} uint8uint16_t ;


volatile uint8_t g_pwm_toggle = 0;
volatile uint16_t g_pwm_lo = 2400;
volatile uint16_t g_pwm_hi = 2400;
volatile uint8uint16_t g_ccp2_next;
volatile uint8uint16_t g_tmr3;

void low_priority_interrupt_service() __interrupt(2) {
	if (PIR1bits.CCP1IF) {
		g_low_pri_isr_guard++;
		PIR1bits.CCP1IF = 0;


		FOR_EACH_MOTOR_DO(UPDATE_POS);
		FOR_EACH_MOTOR_DO(UPDATE_SYNC);
		FOR_EACH_MOTOR_DO(FEED_MORE);

	} // End of 'software' interrupt processing

	if (PIR2bits.TMR3IF) {
		PIR2bits.TMR3IF = 0;
	}

	if (INTCONbits.TMR0IF) {
		INTCONbits.TMR0IF = 0;
	}
	if (PIR2bits.CCP2IF) {
		PIR2bits.CCP2IF = 0;
	}

	if (PIR3bits.USBIF) {
		PIR3bits.USBIF = 0;
		usbcdc_handler();
	} // End of USB interrupt processing
}

#pragma restore

void enterBootloader() {
	__asm__ (" goto 0x0016\n");
}


#define TOGGLE_LED_PIN()  do {if (LED_PIN) LED_PIN = 0; else LED_PIN = 1; } while(0)

void blink_led() {
	static uint8_t old = 0;
	uint8_t new = TMR0H;
	if (new != old) {
		old = new;
		g_blink_counter++;
		if (g_blink_speed == 0 && g_blink_counter > 732) {
			g_blink_counter = 0;
			TOGGLE_LED_PIN();
		}
		if (g_blink_speed == 1 && g_blink_counter > 732/2) {
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
	if ((hid_rx_buffer.uint8[0] == 0xFE)
			&& (hid_rx_buffer.uint8[1] == 0xED)
			&& (hid_rx_buffer.uint8[2] == 0xC0)
			&& (hid_rx_buffer.uint8[3] == 0xDE)) {
		hid_rx_buffer.uint8[63] = 0; // WHY?
		enterBootloader();
	}
}


void check_probe() {
	BITCPY(g_probe.probe_input,PROBE);
	if (g_probe.probe_input & g_probe.probe_armed_trig_on_1)
		g_probe.probe_triggered = 1;
	if (!g_probe.probe_input & g_probe.probe_armed_trig_on_0)
		g_probe.probe_triggered = 1;
}

void main(void) {
	initIO();

	FOR_EACH_MOTOR_DO(INIT_MOTOR);

	TRISCbits.TRISC6 = 0;

// Timer 0 interrupt rate = ((48 000 000 MHz / 4) / 4) / 256 = 11 718.7 Hz
// rxtimeout = 255/11718.75 = 21.7 msec
// Note Timer 2 would allow faster rates if the interrupt processing can be optimized

	T0CONbits.T0CS = 0; // Internal instruction clock as source for timer 0
	T0CONbits.PSA = 0;  // Enable prescaler for Timer 0
	T0CONbits.T08BIT = 0; //  16 bit

	T0CONbits.T0PS2 = 1; // prescaler 1:32 => 1464 Hz
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS0 = 0;

	T0CONbits.TMR0ON = 1; // XXX
	T0CONbits.T0SE = 1; // high to low (no effect when timer clock coming from instruction clock...)

	usbcdc_init();

	INTCON = 0; // Clear interrupt flag bits.

	TMR0L = 0;
	TMR0H = 0;

	T2CONbits.T2CKPS0 = 0;
	T2CONbits.T2CKPS1 = 0; // Timer 2 prescaler 1 => 48 MHz / 4  /  4 = 3 Mhz
//  PR2 = 69 = 12 Mhz / 69 = 174 kHz abs max interrupt frequency
	PR2 = 120; // 3 Mhz / 30 = 100 kHz


	T2CONbits.TMR2ON = 1;

	PIR1bits.CCP1IF = 1;


	IPR1 = 0; // All interrupts low priority
	IPR2 = 0;
	IPR3 = 0;
	INTCON = 0;
	INTCON2 = 0;
	INTCON3 = 0;
	IPR1bits.TMR2IP = 1; // except TIMER2  high priority
	RCONbits.IPEN = 1; // enable priorities

	PIE1 = 0;
	PIE2 = 0;

	// TIMER 1 configuration
	CCP1CON = 0x00; // XXX disable CCP1M
	CCP2CON = 0x0A; // XXX CCP2M = 1010 = interupt on match

	// T3CONbits.TMR3ON = 1; // XXX always on
	T3GCONbits.TMR3GE = 0; // XXX always on

	T3CONbits.T3CKPS0 = 1;
	T3CONbits.T3CKPS1 = 1;
	T3CONbits.TMR3CS0 = 0;
	T3CONbits.TMR3CS1 = 0;
	T3CONbits.SOSCEN = 0;
	T3CONbits.NOT_T3SYNC = 0;
	T3CONbits.RD16 = 0;

	T3CONbits.TMR3ON = 1;

	CCPTMRSbits.C2TSEL=1;

	PIE2bits.TMR3IE = 1;
	PIE2bits.CCP2IE = 1;
	TRISBbits.TRISB3 = 0;

	PIE1bits.CCP1IE = 1; // Enable CCP1 interrupt
	PIE3bits.USBIE = 1; // Enable USB interrupts
	PIE1bits.TMR2IE = 1; // Enable Timer 2 interrupts
	//INTCONbits.TMR0IE = 0; // Enable Timer 0 interrupts

	LED_PIN = 0;


	INTCONbits.PEIE = 1; // enable peripheral interrupts
	INTCONbits.GIE = 1; // global interrupt enable

	WDTCONbits.SWDTEN = 1; // watchdog

	while (1) {
		__asm__ (" CLRWDT  ");

		if (!(ep2_o.STAT & UOWN)) {// new data from host, so process it
			g_message_id = hid_rx_buffer.uint8[63];
			toggle_led();

			if (hid_rx_buffer.uint8[0] == 0xFE) { // special message
				check_for_firmware_update(); // may not ever return
				g_special_request = hid_rx_buffer.uint8[1];
			}
			else { // normal message
				g_special_request = 0;
				FOR_EACH_MOTOR_DO(PROCESS_MOTOR_COMMANDS);

				UPDATE_OUTPUTS(hid_rx_buffer.uint8[50]);

				g_pwm_out = hid_rx_buffer.uint8[52];

				BITCPY(g_probe.probe_armed_trig_on_1, 0x01 & hid_rx_buffer.uint8[49]);
				BITCPY(g_probe.probe_armed_trig_on_0 ,0x02 & hid_rx_buffer.uint8[49]);
				if (!g_probe.probe_armed_trig_on_1 && !g_probe.probe_armed_trig_on_0)
					g_probe.probe_triggered=0;

				// trig sw interrupt so that the queues get updated
				PIR1bits.CCP1IF = 1;

			}
			// turn the buffer over to SIE so we can get more data
			ep2_o.CNT = 64;
			if (ep2_o.STAT & DTS)
				ep2_o.STAT = UOWN | DTSEN;
			else
				ep2_o.STAT = UOWN | DTS | DTSEN;

		}

		// update toad4 status, we do this all the time so as to be ready when we can send it over

		check_probe();

		FOR_EACH_MOTOR_DO(UPDATE_STATUS);

		if (ADCON0bits.GO_NOT_DONE == 0)  {
			g_ADCresult = ADRESH;
			ADCON0bits.GO_NOT_DONE = 1;
			}

		if (!(ep2_i.STAT & UOWN)) {	// we own the USB buffer, so update data going to the host
			g_blink_speed = 1;
			hid_tx_buffer.uint8[63] = g_message_id;

			if (g_special_request==0x01) {
				memcpypgm2ram(&hid_tx_buffer.uint8[0],get_toad4_config(),63);
			} else {

				// make structure g_toad4_status so that we can use a single copy
				memcpyram2ram(&hid_tx_buffer.uint8[0], &g_toad4_status.steppers[0],8);
				memcpyram2ram(&hid_tx_buffer.uint8[8], &g_toad4_status.steppers[1],8);
				memcpyram2ram(&hid_tx_buffer.uint8[16], &g_toad4_status.steppers[2],8);
				memcpyram2ram(&hid_tx_buffer.uint8[32], &g_toad4_status.steppers[3],8);

				// 6*8 = 48
				//FOR_EACH_MOTOR_DO(READ_HOME);

				hid_tx_buffer.uint8[49] = g_probe.uint8; // & 0x03; //
				hid_tx_buffer.uint8[50] = PORTA;//0; // digital inputs 0-7
				hid_tx_buffer.uint8[51] = 0; // digital inputs 8-15
				hid_tx_buffer.uint8[52] = g_ADCresult; // analog input 0
				hid_tx_buffer.uint8[53] = 0;
				hid_tx_buffer.uint8[54] = 0;
				hid_tx_buffer.uint8[55] = 0;
				hid_tx_buffer.uint8[56] = 0;
				hid_tx_buffer.uint8[57] = 0;
				hid_tx_buffer.uint8[58] = 0;
				hid_tx_buffer.uint8[59] = 0;
				hid_tx_buffer.uint8[60] = 0;
				hid_tx_buffer.uint8[61] = 0;
			}
			// turn the buffer over to the SIE so the host will pick it up
			ep2_i.CNT = 64;
			if (ep2_i.STAT & DTS)
				ep2_i.STAT = UOWN | DTSEN;
			else
				ep2_i.STAT = UOWN | DTS | DTSEN;
		}

		blink_led();
	}

}


// TEST

// hid_rx_buffer.uint8[49]; current control (common to all motors? , would break symmetry
// hid_rx_buffer.uint8[50]; digital outputs 0-7
// hid_rx_buffer.uint8[51]; digital outputs 8-15
// hid_rx_buffer.uint8[52]; analog output 0
// hid_rx_buffer.uint8[53]; analog output 1

