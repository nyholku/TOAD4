/*
 * File: stepperirq.c
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

#include "stepper.h"
#include "toad4.h"
#include "critical.h"
#include "usbcdc.h"

volatile u8 waitInt;

volatile u8 syncCounter = 0;
u8 inSync = 0;
u8 syncN = 0;
u8 syncC = 0;

static u8 probeInput;

volatile stepperState __at( 0x0600 ) steppers[NUM_OF_MOTORS];
volatile cmdQueue __at( 0x0300 ) queues[NUM_OF_MOTORS];

// some variable to hold temp values
uint16 oldv;
pStepperCmd cp;
u8 forward;
int16 move;
u8 stepsleft;

// Following allows efficient access (with SDCC / PIC16 architecture)
// to the stepsLeft flags and also makes it possible to treat them as
// a group (this is of course a bit fragile)
static union {
	struct {
		unsigned motorX :1; // LSB of allMotors
		unsigned motorY :1;
		unsigned motorZ :1;
		unsigned motor4 :1;
		unsigned reserved :4;
	};
	u8 allMotors;
} stepsLeft;

//#pragma nooverlay
#pragma save
#pragma nojtbound
void stepperirq() __interrupt(1) {

	if (INTCONbits.TMR0IF) {  // TIMER0 interrupt processing
		waitInt = 0;
		INTCONbits.TMR0IF = 0; // Clear flag
		if (rx_timeout)
			rx_timeout--;

		probeInput = (PROBE ? probeTrigValue : !probeTrigValue); // PROBE

		if (probeInput && g_probeArmed && !g_probeTriggered) {
			g_probeTriggered = 1;
			steppers[0].probePosition = steppers[0].position;
			steppers[1].probePosition = steppers[1].position;
			steppers[2].probePosition = steppers[2].position;
			steppers[3].probePosition = steppers[3].position;
			}


		syncC = 0;
		syncN = 0;
		inSync = FALSE;

#define STEP_GENERATION
#include "stepperirq-inc.c"

		if (syncC > 0 && syncC == syncN) {
			inSync = TRUE;
			if (syncCounter == QUEUE_CAPACITY - 1)
				syncCounter = 0;
			else
				syncCounter++;
		}

#define QUEUE_PROCESSING
#include "stepperirq-inc.c"
	} //End of TIMER0 interrupt processing

	if (PIR2bits.USBIF) { // USB interrupt processing

		usbcdc_handler();

		PIR2bits.USBIF = 0;

	}

} // The closing brace of the interrupt handling routine
#pragma restore

