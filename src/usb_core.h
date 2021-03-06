/*
 * File: usb_core.h
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

#ifndef __USB_CORE_H__
#define __USB_CORE_H__

#include "usb_defs.h"
#include "usb_user_config.h"

typedef struct {
	union {
		uint8_t uint8[64];
		uint16_t uint16[32];
		uint32_t uint32[16];
		int8_t int8[64];
		int16_t int16[32];
		int32_t int32[16];
	};
} hid_buffer_t;



extern volatile hid_buffer_t hid_rx_buffer;
extern volatile hid_buffer_t hid_tx_buffer;

extern uint8_t device_state;
extern uint8_t device_address;
extern uint8_t current_configuration;

void usb_core_init(void);
void usb_core_handler(void);


char usbcdc_wr_busy();

unsigned char usbcdc_rd_ready();

void usbcdc_write(unsigned char len)__wparam;

void usbcdc_flush();

void usbcdc_read();

char usbcdc_getchar();

#endif /* USB_CORE_H_ */
