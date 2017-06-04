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

#include "types.h"
#include <pic18fregs.h>
//#include "usb_cdc.h"
#include "usb_defs.h"

#include "stepper.h"
#include "toad4.h"
//#include "cmdInterp.h"
#include "critical.h"
#include "PIC-config.c"
#include "stepperirq.h"
#include "toad4.h"
#include "printft.h"
#include "usb_core.h"
#include "state_machine.h"
#include "command_queue.h"
#include <string.h> // memcpyram2ram#include "usb_hid.h"
#include "usb_pic_defs.h"
//

// TODO
// Analog input
// Inputs
// Outputs

//uint8_t g_hid_i_cnt_0 = 0;
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
#define CMD_ARM_PROBE 3

#define CMD_JOG_FORWARD 8
#define CMD_JOG_REVERSE 9
#define CMD_SEEK_HOME_FORWARD 10
#define CMD_SEEK_HOME_REVERSE 11
#define CMD_SEEK_NOTHOME_FORWARD 12
#define CMD_SEEK_NOTHOME_REVERSE 13

#define CMD_JOG_FIRST CMD_JOG_FORWARD
#define CMD_JOG_LAST CMD_SEEK_NOTHOME_REVERSE


typedef struct {
	int32_t position;
	uint8_t queue_state;
	uint8_t debug_0;
	uint8_t sync_counter;

	unsigned ready :1;
	unsigned ready2 :1;
	unsigned not_busy :1;
	unsigned not_busy2 :1;
	unsigned not_empty :1;
	unsigned bit6 :1;
	unsigned in_sync :1; // bit 0
	unsigned home :1; // bit 7
} stepper_status_t; // dev => host status

typedef struct {
	uint32_t debug_position;
	uint8_t debug[4];
	uint8_t printf[8];
	stepper_status_t steppers[6];
} toad4_status_t;

static union {
	struct {
		unsigned bit0 :1; // bit 0
		unsigned bit1 :1;
		unsigned bit2 :1;
		unsigned bit3 :1;
		unsigned bit4 :1;
		unsigned bit5 :1;
		unsigned bit6 :1;
		unsigned bit7 :1; // bit 7
	};
	uint8_t as_uint8;
} g_home_status_obsolete;

static uint8_t g_probe;

static volatile toad4_status_t g_toad4_status; // this gets sent to host in toto

static volatile uint8_t g_low_pri_isr_guard;

static uint8_t g_sync_counter[NUMBER_OF_MOTORS] = { 0 };

static uint8_t g_message_id = 0;

void putchar(char c) __wparam
{
	uint8_t i = g_toad4_status.printf[0] + 1;
	if (i < sizeof(g_toad4_status.printf)) {
		g_toad4_status.printf[i] = c;
		g_toad4_status.printf[0] = i;
	}
}

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
		STEPPER(i).last_dir = STEPPER(i).next_dir; \
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

//#define STATE_PROCESS_QUEUE 0
//#define STATE_JOG 1

#define FEED_MORE(i) do { \
	if (STEPPER(i).ready && !QUEUE_EMPTY(i) && STEPPER(i).in_sync) { \
		int16_t distance = QUEUE_FRONT(i)->move_distance; \
		uint8_t next_steps; \
		if (distance < 0) { \
			STEPPER(i).next_dir = 0; \
			if (distance < -255) \
				distance = -255; \
			next_steps = -distance; \
			} \
		else { \
			STEPPER(i).next_dir = 1; \
			if (distance > +255) \
				distance = +255; \
			next_steps = distance; \
			} \
		STEPPER(i).next_steps = next_steps; \
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

/*
 */

// NOTE! Jog dir not handled above
// NOTE! if jog speed<256 jog dist = 0, which is bad

// how do we run to target coordinates ?
// we need acceleration/deceleration

struct {
	DEFINE_SAFE_UINT16(val)
	;
} g_safe_assign_uint16;

#define SAFE_ASSIGN(old,new) do { \
	g_safe_assign_uint16.val = new; \
	if (new > old) { \
		old##_lo = g_safe_assign_uint16.val##_lo; \
		old##_hi = g_safe_assign_uint16.val##_hi; \
		} \
	else { \
		old##_hi = g_safe_assign_uint16.val##_hi; \
		old##_lo = g_safe_assign_uint16.val##_lo; \
		} \
	} while (0)

//*** check how to clear flags ***

#define PROCESS_MOTOR_COMMANDS(i)  do { \
	uint8_t cmd = hid_rx_buffer.uint8[i*8+0]; \
	uint8_t flags = hid_rx_buffer.uint8[i*8+1]; \
	STEPPER(i).sync_group = flags & 0x0F; \
	if (flags & 0x10) \
		ENABLE_MOTOR(i,1); \
	else \
		ENABLE_MOTOR(i,0); \
	if (cmd == CMD_MOVE) { \
		int16_t dist = hid_rx_buffer.int16[i*4+1]; \
		uint16_t speed = hid_rx_buffer.uint16[i*4+2]; \
		if (dist==0 || speed!=0) { \
			QUEUE_REAR(i)->move_distance = dist; \
			QUEUE_REAR(i)->move_speed = speed; \
			QUEUE_PUSH(i); \
			hid_rx_buffer.int16[i*2+0] = 1; \
			hid_rx_buffer.uint16[i*2+1] = 0; \
			} \
	} else if ((cmd >= CMD_JOG_FIRST) && (cmd <= CMD_JOG_LAST)) { do { \
		SAFE_ASSIGN(STEPPER(i).max_accel, hid_rx_buffer.int16[i*4+1]);\
		SAFE_ASSIGN(STEPPER(i).max_speed, hid_rx_buffer.int16[i*4+2]);\
		STEPPER(i).seek_home = 0; \
		STEPPER(i).seek_not_home = 0; \
		if (cmd == CMD_JOG_FORWARD) { \
			STEPPER(i).jog_reverse = 0; \
			STEPPER(i).jog_on = 1; \
			break; \
			} \
		if (cmd == CMD_JOG_REVERSE) { \
			STEPPER(i).jog_reverse = 1; \
			STEPPER(i).jog_on = 1; \
			break; \
			} \
		if (cmd == CMD_SEEK_HOME_FORWARD) { \
			STEPPER(i).seek_reverse = 0; \
			STEPPER(i).seek_home = 1; \
			break; \
			} \
		if (cmd == CMD_SEEK_HOME_REVERSE) { \
			STEPPER(i).seek_reverse = 1; \
			STEPPER(i).seek_home = 1; \
			break; \
			} \
		if (cmd == CMD_SEEK_NOTHOME_FORWARD) { \
			STEPPER(i).seek_reverse = 0; \
			STEPPER(i).seek_not_home = 1; \
			break; \
			} \
		if (cmd == CMD_SEEK_NOTHOME_REVERSE) { \
			STEPPER(i).seek_reverse = 1; \
			STEPPER(i).seek_not_home = 1; \
			break; \
			} \
		} while(0); \
	} else if (cmd == CMD_RESET_POSITION) { \
		STEPPER(i).position = hid_rx_buffer.int32[i*2+1]; \
	} else if (cmd == CMD_ARM_PROBE) { \
		uint8_t arm = hid_rx_buffer.uint8[i*8+2]; \
		if  (arm==0) { \
			STEPPER(i).probeTriggered = 0; \
			STEPPER(i).probeTrigOnFalling= 0; \
			STEPPER(i).probeTrigOnRising= 0; \
			} \
		if  (arm==1) { \
			STEPPER(i).probeTriggered = 0; \
			STEPPER(i).probeTrigOnFalling= 0; \
			STEPPER(i).probeTrigOnRising= 1; \
			} \
		if  (arm==2) { \
			STEPPER(i).probeTriggered = 0; \
			STEPPER(i).probeTrigOnFalling= 1; \
			STEPPER(i).probeTrigOnRising= 0; \
			} \
	} \
	hid_rx_buffer.uint8[i*8] = 0; /*just in case*/ \
	}while(0)

#define BITCOPY(d,s) do {if (s) d=1; else d=0;} while(0)

#define UPDATE_STATUS(i)  do { \
	if (!STEPPER(i).probeTriggered) \
		g_toad4_status.steppers[i].position = get_position(i); \
	g_toad4_status.steppers[i].queue_state =QUEUE_SIZE(i)+(QUEUE_CAPACITY<<4); \
	BITCOPY(g_toad4_status.steppers[i].ready , STEPPER(i).ready); \
	BITCOPY(g_toad4_status.steppers[i].ready2 , STEPPER(i).ready2); \
	BITCOPY(g_toad4_status.steppers[i].not_busy , STEPPER(i).not_busy); \
	BITCOPY(g_toad4_status.steppers[i].not_busy2 , STEPPER(i).not_busy2); \
	BITCOPY(g_toad4_status.steppers[i].in_sync,STEPPER(i).in_sync); \
	BITCOPY(g_toad4_status.steppers[i].not_empty , !QUEUE_EMPTY(i)); \
	g_toad4_status.steppers[i].sync_counter = g_sync_counter[ STEPPER(i).sync_group ]; \
	}while(0)

/*
		BITCOPY(g_toad4_status.steppers[i].in_sync,STEPPER(i).in_sync); \
		BITCOPY(g_toad4_status.steppers[i].ready , STEPPER(i).ready); \
		BITCOPY(g_toad4_status.steppers[i].not_empty , QUEUE_EMPTY(i)); \
		g_toad4_status.steppers[i].sync_counter = g_sync_counter[ STEPPER(i).sync_group ]; \
*/

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

void low_priority_interrupt_service() __interrupt(2) {
	if (PIR1bits.CCP1IF) {
		g_low_pri_isr_guard++;
		PIR1bits.CCP1IF = 0;

		if (PROBE)
			g_probe = 1;
		else
			g_probe = 0;

		FOR_EACH_MOTOR_DO(UPDATE_POS);
		FOR_EACH_MOTOR_DO(UPDATE_SYNC);
		FOR_EACH_MOTOR_DO(UPDATE_HOME);
		FOR_EACH_MOTOR_DO(UPDATE_PROBE);
		FOR_EACH_MOTOR_DO(UPDATE_JOG);
		FOR_EACH_MOTOR_DO(FEED_MORE);

	} // End of 'software' interrupt processing

	if (INTCONbits.TMR0IF) {  // TIMER0 interrupt processing
		INTCONbits.TMR0IF = 0;  // Clear flag
		update_state_machine();
	} // End of TIMER0 interrupt processing

	if (PIR3bits.USBIF) { // USB interrupt processing
		PIR3bits.USBIF = 0;
		//LED_PIN = !LED_PIN;
		usbcdc_handler();
	} // End of USB interrupt processing
}

#pragma restore

int16_t next = 16;
int count = 0;



/*
 void _entry (void) __naked __interrupt 0;

 void _entry (void) __naked __interrupt 0
 {
 __asm
 goto    _main
 __endasm;
 }
 */

void enterBootloader() {
	__asm__ (" goto 0x0016\n");
}

//void trigLoPriorityInterrupt() {
//	__asm__ (" BSF _PIR1, 2  ");
//}

// Turn off PIC extended instruction set, nobody supports it properly
#pragma config XINST=OFF

int counter = 0;

void main(void) {
	initIO();

	FOR_EACH_MOTOR_DO(INIT_MOTOR);

	//g_stepper_states[0].sync_group = 1;
	//g_stepper_states[1].sync_group = 1;

	TRISCbits.TRISC6 = 0;

// Timer 0 interrupt rate = ((48 000 000 MHz / 4) / 4) / 256 = 11 718.7 Hz
// rxtimeout = 255/11718.75 = 21.7 msec
// Note Timer 2 would allow faster rates if the interrupt processing can be optimized

	T0CONbits.T0CS = 0; // Internal instruction clock as source for timer 0
	T0CONbits.PSA = 0;  // Enable prescaler for Timer 0
	T0CONbits.T08BIT = 1; //  8 bit

	T0CONbits.T0PS2 = 1; // prescaler 1:64 => 1.36533333 ms interrupt period
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS0 = 1;

	T0CONbits.TMR0ON = 1;
	T0CONbits.T0SE = 1; // high to low (no effect when timer clock coming from instruction clock...)

	usbcdc_init();

	INTCON = 0; // Clear interrupt flag bits.

	TMR0L = 0;
	TMR0H = 0;

	T2CONbits.T2CKPS0 = 0;
	T2CONbits.T2CKPS1 = 0; // Timer 2 prescaler 1 => 48 MHz / 4  /  1 = 12 Mhz
//  PR2 = 69 = 12 Mhz / 69 = 174 kHz abs max interrupt frequency
	PR2 = 100; // 12 Mhz / 100 = 120 kHz

	T2CONbits.TMR2ON = 1;

	PIR1bits.CCP1IF = 1;

	IPR1bits.CCP1IP = 0; // CCP1 low priority

	RCONbits.IPEN = 1; // enable priorities
	IPR3bits.USBIP = 0; // USB low priority
	INTCON2bits.TMR0IP = 0; // Timer 0 low priority
	IPR1bits.TMR2IP = 1; // Timer 2  high priority

	INTCON = 0;
	INTCON2 = 0;
	INTCON3 = 0;
	PIE1 = 0;
	PIE2 = 0;

	PIE1bits.CCP1IE = 1; // CCP1 enabled
	PIE3bits.USBIE = 1; // Enable USB interrupts
	PIE1bits.TMR2IE = 1; // Enable Timer 2 interrupts
	INTCONbits.TMR0IE = 1; // Enable Timer 0 interrupts

	LED_PIN = 0;

	INTCONbits.PEIE = 1; // enable peripheral interrupts
	INTCONbits.GIE = 1; // global interrupt enable

	g_sync_mask = 0x03;

	/*
	 QUEUE_REAR(0)->move_distance = 0x10;
	 QUEUE_REAR(0)->move_speed = 1;
	 QUEUE_PUSH(0);

	 QUEUE_REAR(1)->move_distance = 0x20;
	 QUEUE_REAR(1)->move_speed = 1;
	 QUEUE_PUSH(1);

	 QUEUE_REAR(1)->move_distance = 0x20;
	 QUEUE_REAR(1)->move_speed = 1;
	 QUEUE_PUSH(1);

	 QUEUE_REAR(0)->move_distance = 0x20;
	 QUEUE_REAR(0)->move_speed = 2;
	 QUEUE_PUSH(0);

	 QUEUE_REAR(0)->move_distance = 0 - 0x30;
	 QUEUE_REAR(0)->move_speed = 1;
	 QUEUE_PUSH(0);
	 */

	while (1) {
		// push move request from the USB message to the queues

		if (!(ep2_o.STAT & UOWN)) {
			// new data from host, so process it

			// check for firmware update command
			if ((hid_rx_buffer.uint8[0] == 0xFE)
					&& (hid_rx_buffer.uint8[1] == 0xED)
					&& (hid_rx_buffer.uint8[2] == 0xC0)
					&& (hid_rx_buffer.uint8[3] == 0xDE)) {
				hid_rx_buffer.uint8[63] = 0; // WHY?
				enterBootloader();
			}

			// not firmware update, next process motor commands
			FOR_EACH_MOTOR_DO(PROCESS_MOTOR_COMMANDS);

			g_message_id = hid_rx_buffer.uint8[48];
			// hid_rx_buffer.uint8[49]; current control (common to all motors? , would break symmetry
			// hid_rx_buffer.uint8[50]; digital outputs 0-7
			// hid_rx_buffer.uint8[51]; digital outputs 8-15
			// hid_rx_buffer.uint8[52]; analog output 0
			// hid_rx_buffer.uint8[53]; analog output 1

			// trig interrupt so that the queues get updated
			PIR1bits.CCP1IF = 1;

			// turn the buffer over to SIE so we can get more data
			ep2_o.CNT = 64;
			if (ep2_o.STAT & DTS)
				ep2_o.STAT = UOWN | DTSEN;
			else
				ep2_o.STAT = UOWN | DTS | DTSEN;
		}

		// update toad4 status, we do this all the time so as to be ready when we can send it over

		FOR_EACH_MOTOR_DO(UPDATE_STATUS);

		if (!(ep2_i.STAT & UOWN)) {
			// we own the USB buffer, so update data going to the host

			// make structure g_toad4_status so that we can use a single copy
			memcpyram2ram(&hid_tx_buffer.uint8[0], &g_toad4_status.steppers[0],8);
			memcpyram2ram(&hid_tx_buffer.uint8[8], &g_toad4_status.steppers[1],8);
			memcpyram2ram(&hid_tx_buffer.uint8[16], &g_toad4_status.steppers[2],8);
			memcpyram2ram(&hid_tx_buffer.uint8[32], &g_toad4_status.steppers[3],8);

			// 6*8 = 48
			FOR_EACH_MOTOR_DO(READ_HOME);

			hid_tx_buffer.uint8[48] = g_message_id;
			//hid_tx_buffer.uint8[49] = g_home_status.as_uint8;
			hid_tx_buffer.uint8[50] = PORTA;//0; // digital inputs 0-7
			hid_tx_buffer.uint8[51] = 0; // digital inputs 8-15
			hid_tx_buffer.uint8[52] = ADRESH; // analog input 0
			hid_tx_buffer.uint8[53] = 0; // analog input 1


			hid_tx_buffer.uint16[31] = 0x1234; // BCD version number
			// turn the buffer over to the SIE so the host will pick it up
			ep2_i.CNT = 64;
			if (ep2_i.STAT & DTS)
				ep2_i.STAT = UOWN | DTSEN;
			else
				ep2_i.STAT = UOWN | DTS | DTSEN;
		}

#if 0
		while (1) {
			volatile unsigned long i;
			for (i = 0; i<200000;)
			i++;
			LED_PIN = !LED_PIN;
			memcpyram2ram(&g_toad4_status, &g_toad4_status, 64);
		}
#endif

//		test_hid();
#if 1
		counter++;

		if (counter & 128)
			LED_PIN = 1;
		else
			LED_PIN = 0;
#endif

//		if (!usbcdc_wr_busy()) {
//			printft("Hello %d\n",t );
//		}
		//	printft("*\n");

		// trig interrupt so that the queues get updated
		//PIR1bits.CCP1IF=1;

	}

}
