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

static u8 blink = 0;
void test_hid();

volatile stepperState __at( 0x0600 ) steppers[NUM_OF_MOTORS];
volatile cmdQueue __at( 0x0300 ) queues[NUM_OF_MOTORS];
volatile u8 syncCounter = 0;
volatile u8 waitInt = 0;

void init_stepperirq_test() {
	uint8_t i;
	for (i=0; i<4; i++) {
		g.motor[i].next_speed = 6000;
		g.motor[i].next_steps = 1;
	}
	steppers[0].accelNCO = 0;
	queues[0].queue[0].moveDistance = 0;
}

#pragma save
#pragma nojtbound
#pragma nooverlay

void low_priority_interrupt_service() __interrupt(2) {
	if (INTCONbits.TMR0IF) {  // TIMER0 interrupt processing
		INTCONbits.TMR0IF = 0; // Clear flag
	} //End of TIMER0 interrupt processing

	if (PIR2bits.USBIF) { // USB interrupt processing
		PIR2bits.USBIF = 0;
		//LED_PIN = !LED_PIN;
		usbcdc_handler();
	} // End of USB interrupt processing
}

#pragma restore

void main(void) {
	OSCCON = 0x70;
	initIO();

	stepperInit(MOTOR_X);

	stepperInit(MOTOR_Y);

	stepperInit(MOTOR_Z);

	stepperInit(MOTOR_4);

	TRISCbits.TRISC6 = 0;

	// Timer 0 interrupt rate = ((48 000 000 MHz / 4) / 4) / 256 = 11 718.7 Hz
	// rxtimeout = 255/11718.75 = 21.7 msec
	// Note Timer 2 would allow faster rates if the interrupt processing can be optimized

	T0CONbits.T0CS = 0; // Internal instruction clock as source for timer 0
	T0CONbits.PSA = 0;  // Assign prescaler to Timer 0. PSA=1 would be equivalent prescaler 1:1
	T0CONbits.T08BIT = 1; //  8 bit

	T0CONbits.T0PS2 = 0; // Set up prescaler to 1:4 = 001
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS0 = 1; // se above

	T0CONbits.TMR0ON = 1;
	T0CONbits.T0SE = 1; // high to low (no effect when timer clock coming from instruction clock...)

	usbcdc_init();


	INTCON = 0; // Clear interrupt flag bits.

	//INTCONbits.TMR0IE = 1; // Enable Timer 0 interrupts

	T2CONbits.T2CKPS0 = 0;
	T2CONbits.T2CKPS1 = 0; // Timer 2 prescaler 1 => 48 MHz / 4  /  1 = 12 Mhz
	T2CONbits.TOUTPS0 = 0;
	T2CONbits.TOUTPS1 = 0;
	T2CONbits.TOUTPS2 = 0;
	T2CONbits.TOUTPS3 = 0;
	// PR2 = 82 = 12 Mhz / 82 = 146 kHz
	PR2 = 100; // 12 Mhz / 120 = 100 kHz

	T2CONbits.TMR2ON = 1;


	TMR0L = 0;
	TMR0H = 0;

	stepperSetMode(MOTOR_X, 0xF, 0, 0, 0, 0);

	stepperSetMode(MOTOR_Y, 0xF, 0, 0, 0, 0);

	stepperSetMode(MOTOR_Z, 0xF, 0, 0, 0, 0);

	stepperSetMode(MOTOR_4, 0xF, 0, 0, 0, 0);

	init_stepperirq_test();

	RCONbits.IPEN = 1; // enable priorities
	IPR2bits.USBIP = 0; // USB low priority
	IPR1bits.TMR2IP = 1; // Timer 2  high priority
	PIE2bits.USBIE = 1; // Enable USB interrupts
	PIE1bits.TMR2IE = 1; // Enable Timer 2 interrupts
	INTCONbits.PEIE = 1; // enable peripheral interrupts
	INTCONbits.GIE = 1; // global interrupt enable

	LED_PIN = 0;

	while (1) {
		//init_stepperirq_test();
		test_hid();
	}


}

unsigned short x;
unsigned short y;
void foo() {
	x+=y;
}
