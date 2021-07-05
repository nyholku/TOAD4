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

typedef struct {
	uint8_t fw_version_major;
	uint8_t fw_version_minor;
	uint8_t fw_version_bugfix;
	uint8_t number_of_motors;
	uint32_t nco_frequency;
} g_toad4_config_t;

__code g_toad4_config_t g_toad4_config = {
		FW_VERSION_MAJOR,
		FW_VERSION_MINOR,
		FW_VERSION_BUGFIX,
		NUMBER_OF_MOTORS,
		NCO_FREQUENCY
};

#if TOAD_HW_VERSION==HW4

void initIO() {
	ANSELA = 0x00;
	ANSELB = 0x00;
	ANSELC = 0x00;
	ANSELD = 0x00;
	ANSELE = 0x00;

	LED_PIN = 0;
	LED_TRIS = 0;

	STEP_X =0;
	STEP_X_TRIS = 0;

	STEP_Y =0;
	STEP_Y_TRIS = 0;

	STEP_Z =0;
	STEP_Z_TRIS = 0;

	STEP_A =0;
	STEP_A_TRIS = 0;

	DIR_X =0;
	DIR_X_TRIS = 0;

	DIR_Y =0;
	DIR_Y_TRIS = 0;

	DIR_Z =0;
	DIR_Z_TRIS = 0;

	DIR_A =0;
	DIR_A_TRIS = 0;

	ENABLE_X =0;
	ENABLE_X_TRIS = 0;

	ENABLE_Y =0;
	ENABLE_Y_TRIS = 0;

	ENABLE_Z =0;
	ENABLE_Z_TRIS = 0;

	ENABLE_A =0;
	ENABLE_A_TRIS = 0;

	TORQUE_X =0;
	TORQUE_X_TRIS = 0;

	TORQUE_Y =0;
	TORQUE_Y_TRIS = 0;

	TORQUE_Z =0;
	TORQUE_Z_TRIS = 0;

	TORQUE_A =0;
	TORQUE_A_TRIS = 0;

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

	HOME_A_TRIS = 1;

	PROBE_TRIS = 1;


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

    // ADC Port configuration

    //ANSELAbits.ANSA0=1; // AN0
    //TRISAbits.TRISA0=1; // RA0

	//SPEED_ANSEL = 1;
	SPEED_TRIS = 1;
    // ADC channel selection

    // NOTE! sdcc 3.4.0 include file "pic18f45k50.h" is missing the definition for ADCON0bits.CHS so it must be patched there
    ADCON0 =  (ADCON0&0x83) || (SPEED_CHANNEL<<2);

    // ADC voltage reference

    ADCON1bits.PVCFG0 =0; // Vref+ = AVdd
    ADCON1bits.NVCFG1 =0; // Vref- = AVss

    // ADC conversion clock source

    ADCON2bits.ADCS=6; // timing 1.3 usec @ Fosc 48 MHz
    ADCON2bits.ACQT=1;

    // Results formatting

    ADCON2bits.ADFM=0;  // ADRES = ADRESH = 8 MSB of 10 bit result

    // Turn ADC  on

    ADCON0bits.ADON=1;
	}

#endif

#if TOAD_HW_VERSION==HW5

void initIO() {
	ANSELA = 0x00;
	ANSELB = 0x00;
	ANSELC = 0x00;
	ANSELD = 0x00;
	ANSELE = 0x00;

	LED_PIN = LED_ON;
	LED_TRIS = 0;

	STEP_X =0;
	STEP_X_TRIS = 0;

	STEP_Y =0;
	STEP_Y_TRIS = 0;

	STEP_Z =0;
	STEP_Z_TRIS = 0;

	STEP_A =0;
	STEP_A_TRIS = 0;

	STEP_B =0;
	STEP_B_TRIS = 0;

	DIR_X =0;
	DIR_X_TRIS = 0;

	DIR_Y =0;
	DIR_Y_TRIS = 0;

	DIR_Z =0;
	DIR_Z_TRIS = 0;

	DIR_A =0;
	DIR_A_TRIS = 0;

	DIR_B =0;
	DIR_B_TRIS = 0;

	ENABLE_X =0;
	ENABLE_X_TRIS = 0;

	ENABLE_Y =0;
	ENABLE_Y_TRIS = 0;

	ENABLE_Z =0;
	ENABLE_Z_TRIS = 0;

	ENABLE_A =0;
	ENABLE_A_TRIS = 0;

	ENABLE_B =0;
	ENABLE_B_TRIS = 0;

	SPINDLE_FWD = 0;
	SPINDLE_FWD_TRIS = 0;

	SPINDLE_REV = 0;
	SPINDLE_REV_TRIS = 0;

	COOLANT =0;

	COOLANT_TRIS = 0;

	HOME_X_TRIS = 1;

	HOME_Y_TRIS = 1;

	HOME_Z_TRIS = 1;

	HOME_A_TRIS = 1;

	HOME_B_TRIS = 1;

	PROBE_TRIS = 1;

	SW_UART_TRIS = 1;

	ARC_START_TRIS = 0;

	ARC_TRANSFER_TRIS = 1;

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

__code void* get_toad4_config() {
	return &g_toad4_config;
}
