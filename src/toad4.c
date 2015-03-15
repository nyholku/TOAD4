/*
 * File: toad4.c
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
#include "toad4.h"


uint8_t g_sync_mask;


#if TOAD_HW_VERSION==HW3

u8 DUMMY_STEP_4;
u8 DUMMY_DIR_4;
u8 DUMMY_HOME_4;

void initIO() {
	// set pin to output
	LED_TRIS = 0;
	LED_PIN = 0;

	STEP_X = 0;
	DIR_X = 0;
	ENABLE_X = 0;

	STEP_Y = 0;
	DIR_Y = 0;
	ENABLE_Y = 0;

	STEP_Z = 0;
	DIR_Z = 0;
	ENABLE_Z = 0;

	MODE1_X = 0;
	TQ1_X = 0;
	TQ2_X = 0;
	DCY1_X = 0;

	MODE1_Y = 0;
	TQ1_Y = 0;
	TQ2_Y = 0;
	DCY1_Y = 0;

	MODE1_Z = 0;
	TQ1_Z = 0;
	TQ2_Z = 0;
	DCY1_Z = 1;

	STEP_X_TRIS = 0;
	DIR_X_TRIS = 0;
	ENABLE_X_TRIS = 0;
	MODE1_X_TRIS = 0;
	TQ1_X_TRIS = 0;
	TQ2_X_TRIS = 0;
	DCY1_X_TRIS = 0;
	HOME_X_TRIS = 1;

	STEP_Y_TRIS = 0;
	DIR_Y_TRIS = 0;
	ENABLE_Y_TRIS = 0;
	MODE1_Y_TRIS = 0;
	TQ1_Y_TRIS = 0;
	TQ2_Y_TRIS = 0;
	DCY1_Y_TRIS = 0;
	HOME_Y_TRIS = 1;

	STEP_Z_TRIS = 0;
	DIR_Z_TRIS = 0;
	ENABLE_Z_TRIS = 0;
	MODE1_Z_TRIS = 0;
	TQ1_Z_TRIS = 0;
	TQ2_Z_TRIS = 0;
	DCY1_Z_TRIS = 0;
	HOME_Z_TRIS = 1;

	RELAY_PIN = 0;
	RELAY_TRIS = 0;

	PROBE_TRIS = 1;

	ADCON1 = 0x0f;

}

#endif

#if TOAD_HW_VERSION==HW4


void initIO() {
	LED_PIN = 0;
	LED_TRIS = 0;

	STEP_X =0;
	STEP_X_TRIS = 0;

	STEP_Y =0;
	STEP_Y_TRIS = 0;

	STEP_Z =0;
	STEP_Z_TRIS = 0;

	STEP_4 =0;
	STEP_4_TRIS = 0;

	DIR_X =0;
	DIR_X_TRIS = 0;

	DIR_Y =0;
	DIR_Y_TRIS = 0;

	DIR_Z =0;
	DIR_Z_TRIS = 0;

	DIR_4 =0;
	DIR_4_TRIS = 0;

	ENABLE_X =0;
	ENABLE_X_TRIS = 0;

	ENABLE_Y =0;
	ENABLE_Y_TRIS = 0;

	ENABLE_Z =0;
	ENABLE_Z_TRIS = 0;

	ENABLE_4 =0;
	ENABLE_4_TRIS = 0;

	TORQUE_X =0;
	TORQUE_X_TRIS = 0;

	TORQUE_Y =0;
	TORQUE_Y_TRIS = 0;

	TORQUE_Z =0;
	TORQUE_Z_TRIS = 0;

	TORQUE_4 =0;
	TORQUE_4_TRIS = 0;

	SPINDLE_FWD = 0;
	SPINDLE_FWD_TRIS = 0;

	SPINDLE_REV = 0;
	SPINDLE_REV_TRIS = 0;

	COOLANT =0;
	COOLANT_TRIS = 0;

	OVERHEAT =1;
	OVERHEAT_TRIS = 0;


	HOME_X_TRIS = 1;

	HOME_Y_TRIS = 1;

	HOME_Z_TRIS = 1;

	HOME_4_TRIS = 1;

	PROBE_TRIS = 1;

	SPEED_POT_TRIS =1;

	ADCON1 = 0x0f;

	// PWM setup
    T2CONbits.TMR2ON = 0;  // Turn the timer off

    PR2 = 255; // period 8 high bits

    CCPR2L = 0; // duty cycle 8 high bits
    CCP2CONbits.DC2B1 = 0; // low bits of duty cycle low bits
    CCP2CONbits.DC2B0 = 0;

    SPINDLE_PWM_TRIS = 0;  // PORTB3 = CCP2 = OUTPUT

    CCP2CONbits.CCP2M3 = 1; // 1100  = 0x0C =  PWM mode
    CCP2CONbits.CCP2M2 = 1;
    CCP2CONbits.CCP2M1 = 0;
    CCP2CONbits.CCP2M0 = 0;

    T2CONbits.T2CKPS1 = 1; // TMR2 PRESCALER = 2 => DIVIDE BY 4
    T2CONbits.T2CKPS0 = 0;

    T2CONbits.TMR2ON = 1; // Turn the PWM on

	}

#endif
