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
#include "usb_cdc.h"
#include "usb_defs.h"

#include "stepper.h"
#include "toad4.h"
#include "cmdInterp.h"
#include "critical.h"
#include "PIC-config.c"
#include "stepperirq.h"
#include "toad4.h"
#include "printft.h"
#include "usb_core.h"
#include "state_machine.h"
#include "command_queue.h"
#include <string.h> // memcpyram2ram
void test_hid();

typedef struct {
	uint32_t position;
//uint8_t padding[10];
} stepper_status_t;

// size 4+10 = 14 * 4= 56 => leaves 8 for other stuff

typedef struct {
	stepper_status_t steppers[4];
	uint32_t debug_position;
	uint8_t debug_last_steps;
} toad4_status_t;

volatile toad4_status_t g_toad4_status;

#define UPDATE_STEPPER_STATE(i)  	\
		if (!g_stepper_states[i].has_next) {\
			uint8_t next_steps = 100;\
			uint8_t next_dir = 1;\
			if (g_stepper_states[i].next_dir)\
				g_stepper_states[i].position += g_stepper_states[i].next_steps;\
			else\
				g_stepper_states[i].position -= g_stepper_states[i].next_steps;\
			g_stepper_states[i].next_speed = 6000;\
			g_stepper_states[i].next_steps = next_steps;\
			g_stepper_states[i].next_dir = next_dir;\
			g_stepper_states[i].has_next = 1;\
		}\

void putchar(char c) __wparam
{
	if (c == '\n') {
		usbcdc_putchar('\r');
	}

	usbcdc_putchar(c);
	if (c == '\n')
		usbcdc_flush();
}

char getchar() {
	usbcdc_flush();
	return usbcdc_getchar();
}


#define STEPPER(i) g_stepper_states[i]
#define UPDATE_QUEUE(i) do {\
	if (!STEPPER(i).has_next){\
		if (g_stepper_states[i].last_dir)\
			STEPPER(i).position += STEPPER(i).last_steps;\
		else\
			STEPPER(i).position -= STEPPER(i).last_steps;\
		STEPPER(i).last_dir = STEPPER(i).next_dir;\
		STEPPER(i).last_steps = STEPPER(i).next_steps;\
		if (!QUEUE_EMPTY(i)) {\
			int16_t distance = QUEUE_FRONT(i)->move_distance;\
			uint8_t next_steps;\
			if (distance < 0) {\
				STEPPER(i).next_dir = 0;\
				if (distance < -255) \
					distance = -255;\
				next_steps = -distance;\
 				}\
 			else {\
 				STEPPER(i).next_dir = 1;\
 				if (distance > +255) \
 					distance = +255;\
				next_steps = distance;\
				}\
			STEPPER(i).next_steps = next_steps;\
			STEPPER(i).next_speed = QUEUE_FRONT(i)->move_speed;\
			if ((QUEUE_FRONT(i)->move_distance -= distance)==0) {\
				QUEUE_POP(i);\
			}\
			STEPPER(i).has_next = 1;\
		}\
		else \
			STEPPER(i).next_steps = 0;\
		}\
	}while(0)\


#define GET_POSITION(i) \
	 (g_stepper_states[i].last_dir)?\
		( g_stepper_states[i].position + (g_stepper_states[i].last_steps - g_stepper_states[i].steps))\
	:\
		( g_stepper_states[i].position - (g_stepper_states[i].last_steps - g_stepper_states[i].steps))\


static uint8_t syncCounter=0;


#pragma save
#pragma nojtbound
#pragma nooverlay

void low_priority_interrupt_service() __interrupt(2) {
	if (PIR1bits.CCP1IF) {
		PIR1bits.CCP1IF = 0; // FIXME, what about if we the bits gets re-set before we clear it here, do we miss that???

		UPDATE_QUEUE(0);
		UPDATE_QUEUE(1);
		UPDATE_QUEUE(2);
		UPDATE_QUEUE(3);
	}
	if (INTCONbits.TMR0IF) {  // TIMER0 interrupt processing
		INTCONbits.TMR0IF = 0; // Clear flag
		update_state_machine();
	} //End of TIMER0 interrupt processing

	if (PIR2bits.USBIF) { // USB interrupt processing
		PIR2bits.USBIF = 0;
		//LED_PIN = !LED_PIN;
		usbcdc_handler();
	} // End of USB interrupt processing
}

#pragma restore

int16_t next=16;
int count = 0;

void main(void) {
	OSCCON = 0x70;
	initIO();

	{
		uint8_t i;
		for (i = 0; i < 4; i++) {
			g_toad4_status.steppers[i].position = 0;
			g_stepper_states[i].has_last = 0;
			g_stepper_states[i].has_next = 0;
			g_stepper_states[i].next_dir = 0;
			g_stepper_states[i].last_dir = 0;
			g_stepper_states[i].next_steps = 0;
			g_stepper_states[i].last_steps = 0;
			g_stepper_states[i].steps = 0;
			g_stepper_states[i].speed = 0;
			g_stepper_states[i].position = 0;
		}
	}
	QUEUE_CLEAR(0);
	QUEUE_CLEAR(1);
	QUEUE_CLEAR(2);
	QUEUE_CLEAR(3);


//	stepperInit(MOTOR_X);

//	stepperInit(MOTOR_Y);

//	stepperInit(MOTOR_Z);

//	stepperInit(MOTOR_4);

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
	INTCONbits.TMR0IE = 1; // Enable Timer 0 interrupts

	T2CONbits.T2CKPS0 = 0;
	T2CONbits.T2CKPS1 = 0; // Timer 2 prescaler 1 => 48 MHz / 4  /  1 = 12 Mhz
	T2CONbits.TOUTPS0 = 0;
	T2CONbits.TOUTPS1 = 0;
	T2CONbits.TOUTPS2 = 0;
	T2CONbits.TOUTPS3 = 0;
	// PR2 = 69 = 12 Mhz / 69 = 174 kHz abs max interrupt frequency
	PR2 = 100; // 12 Mhz / 100 = 120 kHz

	T2CONbits.TMR2ON = 1;

///	stepperSetMode(MOTOR_X, 0xF, 0, 0, 0, 0);

//	stepperSetMode(MOTOR_Y, 0xF, 0, 0, 0, 0);

//	stepperSetMode(MOTOR_Z, 0xF, 0, 0, 0, 0);

//	stepperSetMode(MOTOR_4, 0xF, 0, 0, 0, 0);

	PIR1bits.CCP1IF=1;

	IPR1bits.CCP1IP =0 ; // CCP1 low prioritt
	PIE1bits.CCP1IE=1; // CCP1 enabled

	RCONbits.IPEN = 1; // enable priorities
	IPR2bits.USBIP = 0; // USB low priority
	INTCON2bits.TMR0IP = 0; // Timer 0 low priority
	IPR1bits.TMR2IP = 1; // Timer 2  high priority
	PIE2bits.USBIE = 1; // Enable USB interrupts
	PIE1bits.TMR2IE = 1; // Enable Timer 2 interrupts
	INTCONbits.PEIE = 1; // enable peripheral interrupts
	INTCONbits.GIE = 1; // global interrupt enable

	LED_PIN = 0;

	QUEUE_REAR(0)->move_distance = 0x10;
	QUEUE_REAR(0)->move_speed = 1;
	QUEUE_PUSH(0);

	QUEUE_REAR(0)->move_distance = 0x20;
	QUEUE_REAR(0)->move_speed = 2;
	QUEUE_PUSH(0);

	QUEUE_REAR(0)->move_distance = 0-0x30;
	QUEUE_REAR(0)->move_speed = 1;
	QUEUE_PUSH(0);

	while (1) {
		if (QUEUE_EMPTY(0)) {
			count++;
			QUEUE_REAR(0)->move_distance = next;
			QUEUE_REAR(0)->move_speed = 4;
			QUEUE_PUSH(0);
			next=-next;
			if (next>0)
				next++;
			}



		//UPDATE_STEPPER_STATE(0);
		//UPDATE_STEPPER_STATE(1);
		//UPDATE_STEPPER_STATE(2);
		//UPDATE_STEPPER_STATE(3);

		g_toad4_status.steppers[0].position = GET_POSITION(0);
		g_toad4_status.steppers[1].position = GET_POSITION(1);
		g_toad4_status.steppers[2].position = GET_POSITION(2);
		g_toad4_status.steppers[3].position = GET_POSITION(3);

		g_toad4_status.debug_position=g_stepper_states[0].position;
		g_toad4_status.debug_last_steps= count; // g_stepper_states[0].last_steps;
		memcpyram2ram(&hid_tx_buffer, &g_toad4_status, 64);

		test_hid();
		//LED_PIN=!LED_PIN;
		if (!usbcdc_wr_busy()) {
			printft("Hello %d\n", (g_toad4_status.steppers[0].position));
		}
		//	printft("*\n");
	}

}
