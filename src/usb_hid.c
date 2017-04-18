/*
 * File: usb_hid.c
 *
 * Copyright (c) 2014, Kustaa Nyholm / SpareTimeLabs
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

#include "usb_hid.h"
#include <pic18fregs.h>
#include "usb_pic_defs.h"
#include "usb_defs.h"
#include "toad4.h" // DEBUG ONLY
#include "usb_core.h"

//uint8_t g_hid_i_cnt; // device -> host
//uint8_t g_hid_o_cnt; // host -> device

void test_hid() {
	if ((ep2_i.STAT & UOWN) == 0) {
		//g_hid_i_cnt++;
		ep2_i.CNT = 64;
		if (ep2_i.STAT & DTS)
			ep2_i.STAT = UOWN | DTSEN;
		else
			ep2_i.STAT = UOWN | DTS | DTSEN;
	}

	if (!(ep2_o.STAT & UOWN)) {
		//g_hid_o_cnt++;
		ep2_o.CNT = 64;
		if (ep2_o.STAT & DTS)
			ep2_o.STAT = UOWN | DTSEN;
		else
			ep2_o.STAT = UOWN | DTS | DTSEN;

	}
	/*
	 if (!(ep3_i.STAT & UOWN)) {
	 ep3_i.CNT = 8;
	 if (ep3_i.STAT & DTS)
	 ep3_i.STAT = UOWN | DTSEN;
	 else
	 ep3_i.STAT = UOWN | DTS | DTSEN;
	 }
	 */

}
