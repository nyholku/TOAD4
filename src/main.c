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
#include "usbcdc.h"
#include "usb_defs.h"

#include "stepper.h"
#include "toad4.h"
#include "cmdInterp.h"
#include "critical.h"
#include "PIC-config.c"

static u8 blink = 0;


void  main(void) {
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

	INTCON2bits.TMR0IP = 1; // Make Timer0  high priority

	INTCON = 0; // Clear interrupt flag bits.

	INTCONbits.TMR0IE = 1; // Enable Timer 0 interrupts

	TMR0L = 0;
	TMR0H = 0;

	stepperSetMode(MOTOR_X, 0xF, 0, 0, 0, 0);

	stepperSetMode(MOTOR_Y, 0xF, 0, 0, 0, 0);

	stepperSetMode(MOTOR_Z, 0xF, 0, 0, 0, 0);

	stepperSetMode(MOTOR_4, 0xF, 0, 0, 0, 0);

	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;

	LED_PIN = 0;

	while (1) {

	}

}
