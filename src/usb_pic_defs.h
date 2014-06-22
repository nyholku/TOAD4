/*
 * File: usb_pic_defs.h
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

#ifndef USB_PIC_DEFS_H__
#define USB_PIC_DEFS_H__

#define UOWN   0x80 // USB Own Bit
#define DTS    0x40 // Data Toggle Synchronization Bit
#define KEN    0x20 // BD Keep Enable Bit
#define INCDIS 0x10 // Address Increment Disable Bit
#define DTSEN  0x08 // Data Toggle Synchronization Enable Bit
#define BSTALL 0x04 // Buffer Stall Enable Bit
#define BC9    0x02 // Byte count bit 9
#define BC8    0x01 // Byte count bit 8

typedef struct _BDT
{
    unsigned char STAT;
    unsigned char CNT;
    unsigned int ADDR;
} BDT;

typedef struct _setup_packet_struct
{
    unsigned char bmrequesttype;
    unsigned char brequest;
    unsigned char wvalue0;       // LSB of wValue
    unsigned char wvalue1;       // MSB of wValue
    unsigned char windex0;       // LSB of wIndex
    unsigned char windex1;       // MSB of wIndex
    unsigned short wlength;
    unsigned char extra[56]; // why is this so big????
} setup_packet_struct_t;

#define USTAT_IN (0x04)
#define USTAT_OUT (0x00)

extern __at (0x0400+0*8) volatile BDT ep0_o;
extern __at (0x0404+0*8) volatile BDT ep0_i;
extern __at (0x0400+1*8) volatile BDT ep1_o;
extern __at (0x0404+1*8) volatile BDT ep1_i;
extern __at (0x0400+2*8) volatile BDT ep2_o;
extern __at (0x0404+2*8) volatile BDT ep2_i;
extern __at (0x0400+3*8) volatile BDT ep3_o;
extern __at (0x0404+3*8) volatile BDT ep3_i;


#endif
