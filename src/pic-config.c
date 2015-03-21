/*
 * File: pic-config.c
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


// These oscillator settings are for 4 MHz XTAL
#pragma	config PLLDIV = 1
#pragma	config CPUDIV = OSC1_PLL2
#pragma	config USBDIV = 2
#pragma	config FOSC = HSPLL_HS

#pragma	config WDT = OFF
#pragma	config WDTPS = 1
#pragma	config LPT1OSC = OFF
#pragma	config CCP2MX = OFF
#pragma	config PWRT = ON
#pragma	config BOR = OFF
#pragma	config BORV = 0
#pragma	config VREGEN = ON
#pragma	config FCMEN = OFF
#pragma	config IESO = OFF
#pragma	config MCLRE = ON
#pragma	config PBADEN = OFF
#pragma	config STVREN = OFF
#pragma	config LVP = OFF
#pragma	config ICPRT = OFF
#pragma	config XINST = OFF
#pragma	config DEBUG = OFF

#pragma	config CP0 = OFF
#pragma	config CP1 = OFF
#pragma	config CP2 = OFF
#pragma	config CPB = OFF
#pragma	config CPD = OFF
#pragma	config WRT0 = OFF
#pragma	config WRT1 = OFF
#pragma	config WRT2 = OFF
#pragma	config WRTB = OFF
#pragma	config WRTC = OFF
#pragma	config WRTD = OFF
#pragma	config EBTR0 = OFF
#pragma	config EBTR1 = OFF
#pragma	config EBTR2 = OFF
#pragma	config EBTRB = OFF

/*
__code char __at 0x300000 CONFIG1L = 0x20; // USBDIV=1, CPUDIV=00, PLLDIV = 000

__code char __at 0x300001 CONFIG1H = 0x0E; // IESO=0, FCMEN=0, FOSC = 1110

__code char __at 0x300002 CONFIG2L = 0x20; // Brown out off, PWRT On

__code char __at 0x300003 CONFIG2H = 0x00; // WDT off

__code char __at 0x300004 CONFIG3L = 0xff; // Unused configuration bits

__code char __at 0x300005 CONFIG3H = 0x80; // MCLR enabled , PORTB digital, CCP2 - RB2

__code char __at 0x300006 CONFIG4L = 0x80; // ICD off, ext off, LVP off, stk ovr off

__code char __at 0x300007 CONFIG4H = 0xff; // Unused configuration bits

__code char __at 0x300008 CONFIG5L = 0xff; // No __code read protection

__code char __at 0x300009 CONFIG5H = 0xff; // No data/boot read protection

__code char __at 0x30000A CONFIG6L = 0xff; // No __code write protection

__code char __at 0x30000B CONFIG6H = 0xff; // No data/boot/table protection

__code char __at 0x30000C CONFIG7L = 0xff; // No table read protection

__code char __at 0x30000D CONFIG7H = 0xff; // No boot table protection
*/
