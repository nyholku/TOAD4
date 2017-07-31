/*
 * File: toad4.h
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

#ifndef __TOAD4_H__
#define __TOAD4_H__

#include <stdint.h>
#include <pic18fregs.h>

#define NUMBER_OF_MOTORS 4

__code void* get_toad4_config();

// note all this is referenced from assembler so do-not rearrange at willy-nilly
typedef volatile struct {
	uint16_t nco; // offset 0
	uint16_t speed; // offset 2
	uint16_t next_speed; // offset 4
	uint8_t steps; // offset 6
	uint8_t next_steps; // offset 7
	uint8_t last_steps; // offset 8
	union {
	struct { //offset 9
		unsigned in_sync :1; // bit 0
		unsigned next_dir :1;
		unsigned last_dir :1;
		unsigned has_last :1;
		unsigned ready :1;
		unsigned ready2 :1;
		unsigned not_busy :1;
		unsigned not_busy2 :1; // bit 7
	};
		uint8_t flags;
	};
	uint8_t queue_size; // offset 10
	uint8_t queue_rear; // offset 11
	uint8_t queue_front; // offset 12
	int32_t position; // offset 13
	uint8_t sync_group; // offset 17
// size this far 18
	uint8_t state;
	uint16_t jog_speed_OBSOLETE;
	uint16_t max_accel_OBSOLETE;
	uint16_t max_speed_OBSOLETE;
	union {
	struct {
		unsigned jog_on :1; // bit 0
		unsigned jog_reverse :1;
		unsigned seek_home :1;
		unsigned seek_not_home :1;
		unsigned seek_reverse :1;
		unsigned probeTriggered :1;
		unsigned probeTrigOnRising :1;
		unsigned probeTrigOnFalling :1; // bit 7
	};
		uint8_t flags2;
	};
// size this far 26
} stepper_state_t;

typedef volatile struct {
	struct { // packing 'booleans' like this into one bit fields allows faster code generation on SDCC
		unsigned irq_flag :1; // High priority interrupt clears this
		unsigned reserve_b1 :1;
		unsigned reserve_b2 :1;
		unsigned reserve_b3 :1;
		unsigned reserve_b4 :1;
		unsigned reserve_b5 :1;
		unsigned reserve_b6 :1;
		unsigned reserve_b7 :1;
	};
} irq_flags_t;

typedef volatile struct {
	union {
		struct { // packing 'booleans' like this into one bit fields allows faster code generation on SDCC
			unsigned stepper_0 :1; // High priority interrupt clears this
			unsigned stepper_1 :1;
			unsigned stepper_2 :1;
			unsigned stepper_3 :1;
			unsigned stepper_4 :1;
			unsigned stepper_5 :1;
			unsigned stepper_6 :1;
			unsigned stepper_7 :1;
		};
		uint8_t all_steppers;
	};
} stepper_flags_t;

extern volatile stepper_state_t g_stepper_states[NUMBER_OF_MOTORS];

// Supported hardware versions
#define HW4 4

// Define for which version this is built
#define TOAD_HW_VERSION HW4

// Form the version literals

#if TOAD_HW_VERSION== HW4
#define HW_VERSION_STRING "4"
#endif

#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 0
#define FW_VERSION_BUGFIX 0

#define NCO_FREQUENCY 100000

#define NUMBER_OF_MOTORS 4

void initIO();

#if TOAD_HW_VERSION==HW4

#define STEP_X 				LATDbits.LATD0
#define STEP_X_TRIS 		TRISDbits.TRISD0

#define STEP_Y 				LATDbits.LATD1
#define STEP_Y_TRIS 		TRISDbits.TRISD1

#define STEP_Z 				LATDbits.LATD2
#define STEP_Z_TRIS 		TRISDbits.TRISD2

#define STEP_A 				LATDbits.LATD3
#define STEP_A_TRIS 		TRISDbits.TRISD3

#define DIR_X 				LATCbits.LATC0
#define DIR_X_TRIS 			TRISCbits.TRISC0

#define DIR_Y 				LATCbits.LATC1
#define DIR_Y_TRIS 			TRISCbits.TRISC1

#define DIR_Z 				LATCbits.LATC2
#define DIR_Z_TRIS 			TRISCbits.TRISC2

#define DIR_A 				LATBbits.LATB6
#define DIR_A_TRIS 			TRISBbits.TRISB6

#define ENABLE_X 			LATDbits.LATD4
#define ENABLE_X_TRIS 		TRISDbits.TRISD4

#define ENABLE_Y 			LATDbits.LATD5
#define ENABLE_Y_TRIS 		TRISDbits.TRISD5

#define ENABLE_Z 			LATDbits.LATD6
#define ENABLE_Z_TRIS 		TRISDbits.TRISD6

#define ENABLE_A 			LATDbits.LATD7
#define ENABLE_A_TRIS 		TRISDbits.TRISD7

#define HOME_X 				PORTAbits.RA2
#define HOME_X_TRIS 		TRISAbits.TRISA2

#define HOME_Y 				PORTAbits.RA3
#define HOME_Y_TRIS 		TRISAbits.TRISA3

#define HOME_Z 				PORTAbits.RA4
#define HOME_Z_TRIS 		TRISAbits.TRISA4

#define HOME_A 				PORTAbits.RA5
#define HOME_A_TRIS 		TRISAbits.TRISA5

#define LED_PIN   			LATBbits.LATB4
#define LED_TRIS  			TRISBbits.TRISB4

#define PROBE 				PORTAbits.RA1
#define PROBE_TRIS	 		TRISAbits.TRISA1

#define SPEED_ANSEL			ANSELAbits.ANSA0
#define SPEED_TRIS			TRISAbits.TRISA0
#define SPEED_CHANNEL		0

#define TORQUE_X 			PORTBbits.RB0
#define TORQUE_X_TRIS		TRISBbits.TRISB0

#define TORQUE_Y 			PORTBbits.RB1
#define TORQUE_Y_TRIS		TRISBbits.TRISB1

#define TORQUE_Z 			PORTBbits.RB2
#define TORQUE_Z_TRIS		TRISBbits.TRISB2

#define TORQUE_A 			PORTBbits.RB7
#define TORQUE_A_TRIS		TRISBbits.TRISB7

#define SPINDLE_FWD 		LATEbits.LATE0
#define SPINDLE_FWD_TRIS	TRISEbits.TRISE0

#define SPINDLE_REV 		LATEbits.LATE1
#define SPINDLE_REV_TRIS	TRISEbits.TRISE1

#define OVERHEAT 			LATEbits.LATE2
#define OVERHEAT_TRIS		TRISEbits.TRISE2

#define SPINDLE_PWM 		PORTBbits.RB3
#define SPINDLE_PWM_TRIS	TRISBbits.TRISB3

#define COOLANT				PORTBbits.RB5
#define COOLANT_TRIS		TRISBbits.TRISB5

#endif

#endif
