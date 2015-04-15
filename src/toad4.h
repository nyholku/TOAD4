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

#ifndef TOAD4_H_
#define TOAD4_H_

#include <stdint.h>
#include <pic18fregs.h>

#define NUMBER_OF_MOTORS 4

typedef volatile struct {
	uint16_t nco; // offset 0
	uint16_t speed; // offset 2
	uint16_t next_speed; // offset 4
	uint8_t steps; // offset 6
	uint8_t next_steps; // offset 7
	uint8_t last_steps; // offset 8
	struct { //offset 9
		unsigned reserver_b0 :1;
		unsigned next_dir :1;
		unsigned last_dir :1;
		unsigned has_last :1;
		unsigned reserve_b4 :1;
		unsigned reserve_b5 :1;
		unsigned reserve_b6 :1;
		unsigned reserve_b7 :1;
	};
	uint8_t queue_size; // offset 10
	uint8_t queue_rear; // offset 11
	uint8_t queue_front; // offset 12
	int32_t position; // offset 13
// size now 17
//uint8_t reserve[4];
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

extern volatile stepper_state_t g_stepper_states[4];
extern volatile irq_flags_t g_irq_flags;
extern volatile stepper_flags_t g_ready_flags;
extern volatile uint8_t g_sync_mask;


// Supported hardware versions
#define HW3 3
#define HW4 4
// Define for which version this is built
#define TOAD_HW_VERSION HW4

// Form the version literals
#if TOAD_HW_VERSION== HW3
#define HW_VERSION_STRING "3"
#endif

#if TOAD_HW_VERSION== HW4
#define HW_VERSION_STRING "4"
#endif

#define FW_VERSION_STRING "1.5.2"

void initIO();

#if TOAD_HW_VERSION==HW3

extern u8 DUMMY_STEP_4;
extern u8 DUMMY_DIR_4;
extern u8 DUMMY_HOME_4;

#define STEP_X 			LATDbits.LATD0
#define STEP_X_TRIS 	TRISDbits.TRISD0

#define STEP_Y 			LATDbits.LATD1
#define STEP_Y_TRIS 	TRISDbits.TRISD1

#define STEP_Z 			LATDbits.LATD2
#define STEP_Z_TRIS 	TRISDbits.TRISD2

#define STEP_4 			DUMMY_STEP_4

#define DIR_X 			LATDbits.LATD3
#define DIR_X_TRIS 		TRISDbits.TRISD3

#define DIR_Y 			LATDbits.LATD4
#define DIR_Y_TRIS 		TRISDbits.TRISD4

#define DIR_Z 			LATDbits.LATD5
#define DIR_Z_TRIS 		TRISDbits.TRISD5

#define DIR_4			DUMMY_DIR_4

#define MODE1_X 		LATDbits.LATD6 // same as MODE1_Y
#define MODE1_X_TRIS 	TRISDbits.TRISD6

#define MODE1_Y 		LATDbits.LATD6
#define MODE1_Y_TRIS 	TRISDbits.TRISD6

#define MODE1_Z 		LATDbits.LATD7
#define MODE1_Z_TRIS 	TRISDbits.TRISD7

#define ENABLE_X 		LATCbits.LATC0
#define ENABLE_X_TRIS 	TRISCbits.TRISC0

#define ENABLE_Y 		LATCbits.LATC1
#define ENABLE_Y_TRIS 	TRISCbits.TRISC1

#define ENABLE_Z 		LATCbits.LATC2
#define ENABLE_Z_TRIS 	TRISCbits.TRISC2

#define TQ1_X 			LATBbits.LATB2 // same as TQ1_Y
#define TQ1_X_TRIS 	    TRISBbits.TRISB2

#define TQ2_X			LATBbits.LATB3 // same as TQ2_Y
#define TQ2_X_TRIS		TRISBbits.TRISB3

#define TQ1_Y			LATBbits.LATB2
#define TQ1_Y_TRIS		TRISBbits.TRISB2

#define TQ2_Y 			LATBbits.LATB3
#define TQ2_Y_TRIS		TRISBbits.TRISB3

#define TQ1_Z 			LATBbits.LATB0
#define TQ1_Z_TRIS 		TRISBbits.TRISB0

#define TQ2_Z 			LATBbits.LATB1
#define TQ2_Z_TRIS 		TRISBbits.TRISB1

#define DCY1_X 			LATBbits.LATB6 // same as DCY1_Y
#define DCY1_X_TRIS 	TRISBbits.TRISB6

#define DCY1_Y 			LATBbits.LATB6
#define DCY1_Y_TRIS 	TRISBbits.TRISB6

#define DCY1_Z 			LATBbits.LATB7
#define DCY1_Z_TRIS 	TRISBbits.TRISB7

#define HOME_X 			PORTAbits.RA3
#define HOME_X_TRIS 	TRISAbits.TRISA3

#define HOME_Y 			PORTAbits.RA5
#define HOME_Y_TRIS 	TRISAbits.TRISA5

#define HOME_Z 			PORTEbits.RE1
#define HOME_Z_TRIS 	TRISEbits.TRISE1

#define HOME_4			DUMMY_HOME_4

//#define LED_PIN   		PORTBbits.RB4
#define LED_PIN   		LATBbits.LATB4
#define LED_TRIS  		TRISBbits.TRISB4

#define RELAY_PIN   	LATEbits.LATE0
#define RELAY_TRIS  	TRISEbits.TRISE0

#define PROBE 			PORTAbits.RA1
#define PROBE_TRIS	 	TRISAbits.TRISA1

#endif

#if TOAD_HW_VERSION==HW4

// Note that you can't willy nilly re-arrange outputs as there is intricate details
// in how the step and dir signals are updated in the high priority TMR2 interrupt
// for example all STEP_n signals must be in the same port and all DIR_n in an
// other port.

#define STEP_OUTPUT_PORT LATD
#define STEP_OUTPUT_ALL 0x0f

#define STEP_X 				LATDbits.LATD0
#define STEP_X_TRIS 		TRISDbits.TRISD0

#define STEP_Y 				LATDbits.LATD1
#define STEP_Y_TRIS 		TRISDbits.TRISD1

#define STEP_Z 				LATDbits.LATD2
#define STEP_Z_TRIS 		TRISDbits.TRISD2

#define STEP_4 				LATDbits.LATD3
#define STEP_4_TRIS 		TRISDbits.TRISD3

#define DIR_X 				LATCbits.LATC0
#define DIR_X_TRIS 			TRISCbits.TRISC0

#define DIR_Y 				LATCbits.LATC1
#define DIR_Y_TRIS 			TRISCbits.TRISC1

#define DIR_Z 				LATCbits.LATC2
#define DIR_Z_TRIS 			TRISCbits.TRISC2

#define DIR_4 				LATBbits.LATB6
#define DIR_4_TRIS 			TRISBbits.TRISB6

#define ENABLE_X 			LATDbits.LATD4
#define ENABLE_X_TRIS 		TRISDbits.TRISD4

#define ENABLE_Y 			LATDbits.LATD5
#define ENABLE_Y_TRIS 		TRISDbits.TRISD5

#define ENABLE_Z 			LATDbits.LATD6
#define ENABLE_Z_TRIS 		TRISDbits.TRISD6

#define ENABLE_4 			LATDbits.LATD7
#define ENABLE_4_TRIS 		TRISDbits.TRISD7

#define HOME_X 				PORTAbits.RA2
#define HOME_X_TRIS 		TRISAbits.TRISA2

#define HOME_Y 				PORTAbits.RA3
#define HOME_Y_TRIS 		TRISAbits.TRISA3

#define HOME_Z 				PORTAbits.RA4
#define HOME_Z_TRIS 		TRISAbits.TRISA4

#define HOME_4 				PORTAbits.RA5
#define HOME_4_TRIS 		TRISAbits.TRISA5

#define LED_PIN   			LATBbits.LATB4
#define LED_TRIS  			TRISBbits.TRISB4

#define PROBE 				PORTAbits.RA1
#define PROBE_TRIS	 		TRISAbits.TRISA1

#define SPEED_POT_TRIS 		PORTAbits.RA0
#define SPEED_POT_TRIS_TRIS	TRISAbits.TRISA0

#define TORQUE_X 			PORTBbits.RB0
#define TORQUE_X_TRIS		TRISBbits.TRISB0

#define TORQUE_Y 			PORTBbits.RB1
#define TORQUE_Y_TRIS		TRISBbits.TRISB1

#define TORQUE_Z 			PORTBbits.RB2
#define TORQUE_Z_TRIS		TRISBbits.TRISB2

#define TORQUE_4 			PORTBbits.RB7
#define TORQUE_4_TRIS		TRISBbits.TRISB7

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

#endif /* TOAD3_H_ */
