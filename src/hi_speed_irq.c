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

#ifndef SELFINCLUDE
 
#include "hi_speed_irq.h"
#include "toad4.h"

globals_t g = { 0 };


//-----------------------------------------------------------------------------------------

// Because '__interrupt 1 __naked' places code directly into address 0x8
// the actual interrupt routine cannot really tagged '__naked' as it would
// overrun the low priority __interrupt 2 vector, so instead we put in
// this inline  assembly jump to the actual service routine which is tagged
// as '__naked' (to make it fit) but not as '__interrupt' so it will be
// re-located by the linker to some free code memory.

#pragma save
#pragma nojtbound
#pragma nooverlay

//void goto_high_priority_interrupt_service() __interrupt(1) __naked {
//	__asm__("GOTO	_high_priority_interrupt_service");
//}

// This is the actual high priority interrupt service routine, to prevent
// SDCC from conservatively (and unnecessarily) saving/restoring some registers.
// This routine is tagged wiht '__naked' and uses hand crafted inline assmebly
// prologue and epilogue

void high_priority_interrupt_service() __interrupt(1) {
	//INLINE_ASM_ISR_PROLOGUE();
	LED_PIN = 1;
	if (PIR1bits.TMR2IF) {
		PIR1bits.TMR2IF = 0;

		// Clear irq_flag to indicate that step pulse interrupt has been processed
		g.irq_flag = 0;

		// Turn ON *all* STEP signals, this generates a synchronized rising edge step pulse
		// for those outputs that were OFF and also include the shadowed port D bits
		// FIXME, initilize the port correctly or we miss first step on each motot

		STEP_OUTPUT_PORT |= STEP_OUTPUT_ALL;

// Now, include the step geneation four times
#define SELFINCLUDE

#define MOTOR g.motor[0]
#define STEP_OUTPUT STEP_X
#define DIR_OUTPUT DIR_X
#include "stepperirq.c"

#define MOTOR g.motor[1]
#define STEP_OUTPUT STEP_Y
#define DIR_OUTPUT DIR_Y
#include "stepperirq.c"

#define MOTOR g.motor[2]
#define STEP_OUTPUT STEP_Z
#define DIR_OUTPUT DIR_Z
#include "stepperirq.c"

#define MOTOR g.motor[3]
#define STEP_OUTPUT STEP_4
#define DIR_OUTPUT DIR_4
#include "stepperirq.c"

#undef SELFINCLUDE

#endif /*ifndef SELFINCLUDE*/

#ifdef __CDT_PARSER__
#define SELFINCLUDE
#endif

// And here is the self included code ....
#ifdef SELFINCLUDE
		//LED_PIN = 1;

		MOTOR.nco += MOTOR.speed;
		if (MOTOR.steps) {
			if (MOTOR.nco_hi_bit) { // speed NCO overflowed and we have steps left => generate pulse
				MOTOR.steps--;
				STEP_OUTPUT = 0;// STEP signal = 0 generates a rising edge on next interrupt
			}
		} else if (MOTOR.has_next) { // check if there more steps in the queue
			MOTOR.has_next = 0;// signals to higher level that we have consumed the next 'step' so to speak
			MOTOR.steps = MOTOR.next_steps;
			MOTOR.speed = MOTOR.next_speed;
			// Following looks more complex than DIR_OUTPUT = MOTOR.next_dir, but SDCC generates better code this way
			// Note, this is NOT too early to set next direction as we never get here if above STEP_OUT was initiated
			if (MOTOR.next_dir) {
				DIR_OUTPUT=1;
			} else {
				DIR_OUTPUT=0;
			}
		}
		MOTOR.has_next = 1; // FIXME: THIS LINE JUST FOR TESTING
		MOTOR.nco_hi_bit=0; // Clear the nco overflow

		//LED_PIN = 0;

#ifndef __CDT_PARSER__
#undef MOTOR
#undef STEP_OUTPUT
#undef DIR_OUTPUT
#endif

#endif /*ifdef SELFINCLUDE*/

#ifdef __CDT_PARSER__
#undef SELFINCLUDE
#endif

#ifndef SELFINCLUDE
		// End of the self included code ....

	}
	LED_PIN = 0;

	//INLINE_ASM_ISR_EPILOGUE();
} /* end of high_priority_interrupt_service() */

#pragma restore

//-----------------------------------------------------------------------------------------

#endif /*ifndef SELFINCLUDE*/

