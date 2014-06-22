/*
 * File: usb_cdc.h
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

#ifndef USBCDC_H_
#define USBCDC_H_

extern unsigned char usbcdc_device_state;

#define USBCDC_BUFFER_LEN 32
#define USBCDC_SELF_POWERED 1
#define USBCDC_MAXPOWER 100

void usbcdc_init(void);

void cdc_wait_config();

void usbcdc_handler(void);

void usbcdc_putchar(char c) __wparam;

void usbcdc_flush();

char usbcdc_getchar() ;

#pragma udata usbram5 setup_packet control_transfer_buffer cdc_rx_buffer cdc_tx_buffer cdcint_buffer
extern volatile unsigned char cdc_tx_buffer[];
char usbcdc_wr_busy();
void usbcdc_write(unsigned char len) __wparam;

extern volatile unsigned char cdc_rx_buffer[];
unsigned char usbcdc_rd_ready();
void usbcdc_read();
#define usbcdc_rd_len() (ep2_o.CNT)

#endif
