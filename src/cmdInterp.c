/*
 * File: cmdInterp.c
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

#include "types.h"
#include "cmdinterp.h"
#include "stepper.h"
#include "toad4.h"
#include "usbcdc.h"

__code char version[16] = FW_VERSION_STRING"-"HW_VERSION_STRING;

#define ERR_OK 0
#define ERR_QUEUE_FULL 1
#define ERR_MISSING 2
#define ERR_RESP_TOO_LONG 3
#define ERR_BAD_COMMAND 11
#define ERR_LINE_TOO_LONG 12

#define MAX_RESP 48
#define MAX_LINE_LEN 32

#define CMD_MOVE_DISTANCE 1
#define CMD_JOG_REVERSE 2
#define CMD_JOG_FORWARD 3
#define CMD_RESET_QUEUE 8
#define CMD_ENABLE_QUEUE 9
#define CMD_QUEUE_STATE 10
#define CMD_MOTOR_STATE 11
#define CMD_SET_MODE 12
#define CMD_SEEK_HOME 13
#define CMD_CTRL_JOG 14
#define CMD_SET_POS 15
#define CMD_GET_POS 16
#define CMD_GET_VERSION 17
#define CMD_ARM_PROBE 18
#define CMD_GET_PROBE_POS 19
#define CMD_CONFIG_PROBE 20
#define CMD_SET_OUTPUT 21
#define CMD_GET_INPUT 22

static u8 __at( 0x0700 ) rbuffer[MAX_RESP];

static volatile u8 linebuffer[MAX_LINE_LEN];
static volatile __data u8* lptr;
static volatile u8 linelen = 0;
static volatile u8 lineTooLong;
static volatile u8 rlen;


u8 getMessage()
__wparam {

	u8 len, chr, sum, cnt;
	lineTooLong = 0;
	lptr = linebuffer;

	do {
		linelen = 0;
		chr = 0x55;
		sum = 0;
		len = 0;
		cnt = 0;
		rx_timeout = 255;
		while (len == 0) {
			if (usbcdc_rd_ready()) {
				len = usbcdc_getchar();
				if (len > 0x80)
				len &= 0x7F;
				else
				len = 0;
			}
			if (!rx_timeout)
			return 0;
		}
		rx_timeout = 255;
		while ((linelen < len)) {
			if (usbcdc_rd_ready()) {
				cnt++;
				sum += chr;
				chr = usbcdc_getchar();

				if (linelen < sizeof(linebuffer)) {
					linebuffer[linelen] = chr;
					linelen++;
				} else
				lineTooLong = 1;
				rx_timeout = 255;
			}
			if (!rx_timeout)
			return 0;
		}
	}while (chr != sum);
	linelen--;
	return 1;
}


uint16 parseByte() {
	uint16 r = (*lptr++);
	linelen--;
	return r;
}

uint16 parseDoubleByte() {
	uint16 r = ((*lptr++) << 8) + (*lptr++);
	linelen -= 2;
	return r;
}

static union {
	struct {
		uint32 as_uint32;
	};
	struct {
		uint16 as_uint16[2];
	};
	struct {
		u8 as_u8[4];
	};
} temp;

uint32 parseQuadByte() {
	temp.as_u8[3] = parseByte();
	temp.as_u8[2] = parseByte();
	temp.as_u8[1] = parseByte();
	temp.as_u8[0] = parseByte();
	return temp.as_uint32;
}

void binaryOut(u8 d) {
	if (rlen < MAX_RESP)
		rbuffer[rlen++] = d;
	else
		rbuffer[MAX_RESP - 1] = ERR_RESP_TOO_LONG;
}

u8 getInput(u8 input) {
	switch (input) {
	case 0:
		return HOME_X;
	case 1:
		return HOME_Y;
	case 2:
		return HOME_Z;
	case 3:
		return HOME_4;
	case 16:
		return PROBE;
	default:
		return 0;
	}
}

void processMessage() {
	rlen = 0;

	if (lineTooLong)
		binaryOut(ERR_LINE_TOO_LONG);
	else {
		while (linelen--) {
			u8 forward;
			u8 rampup;
			u8 motor = *lptr++;
			u8 cmd = motor >> 3;
			motor &= 0x07;
			//		printft("%d ",linelen);
			//		if (DEBUG_CMD) printft("MOTOR %d ",motor);
			switch (cmd) {
			case CMD_SET_OUTPUT:
				if (linelen < 2)
					goto missing_data;
				binaryOut(ERR_OK);
				stepperSetOutput(parseByte(), parseByte());
				break;
			case CMD_GET_INPUT:
				if (linelen < 1)
					goto missing_data;
				binaryOut(ERR_OK);
				binaryOut(getInput(parseByte()));
				break;
			case CMD_CONFIG_PROBE:
				if (linelen < 2)
					goto missing_data;
				stepperConfigProbe(parseByte(), parseByte());
				binaryOut(ERR_OK);
				break;

			case CMD_ARM_PROBE:
				if (linelen < 4)
					goto missing_data;
				stepperArmProbe(motor, parseDoubleByte(), parseDoubleByte());
				binaryOut(ERR_OK);
				break;

			case CMD_GET_PROBE_POS:
				temp.as_uint32 = stepperGetProbePosition(motor);
				binaryOut(ERR_OK);
				binaryOut(temp.as_u8[3]);
				binaryOut(temp.as_u8[2]);
				binaryOut(temp.as_u8[1]);
				binaryOut(temp.as_u8[0]);
				break;

			case CMD_QUEUE_STATE:
				binaryOut(ERR_OK);
				binaryOut(stepperGetState(motor));
				binaryOut(stepperQueueSize(motor));
				binaryOut(stepperQueueCapacity(motor));
				break;

			case CMD_MOTOR_STATE:
				binaryOut(ERR_OK);
				binaryOut(stepperGetState(motor));
				break;

			case CMD_RESET_QUEUE:
				binaryOut(ERR_OK);
				stepperAbort(motor);
				break;

			case CMD_ENABLE_QUEUE:
				binaryOut(ERR_OK);
				stepperGo(motor);
				break;

			case CMD_SET_MODE:
				if (linelen < 5)
					goto missing_data;
				stepperSetMode(motor, parseByte(), parseByte(), parseByte(),
						parseByte(), parseByte());
				binaryOut(ERR_OK);
				break;

			case CMD_SEEK_HOME:
				if (linelen < 10)
					goto missing_data;

				if (!stepperSeekHome(motor, parseDoubleByte(),
						parseDoubleByte(), parseDoubleByte(), parseQuadByte()))
					goto queue_full;
				binaryOut(ERR_OK);
				break;

			case CMD_JOG_REVERSE:
				forward = FALSE;
				//binaryOut(ERR_OK);
				goto jog;

			case CMD_JOG_FORWARD:
				forward = TRUE;
				//binaryOut(ERR_OK);
				goto jog;

			case CMD_CTRL_JOG:
				if (linelen < 2)
					goto missing_data;
				stepperSetJogFlag(motor, parseDoubleByte());
				binaryOut(ERR_OK);
				break;

			case CMD_SET_POS:
				if (linelen < 4)
					goto missing_data;
				binaryOut(ERR_OK);
				stepperSetPosition(motor, parseQuadByte());
				break;

			case CMD_GET_POS:
				temp.as_uint32 = stepperGetPosition(motor);
				binaryOut(ERR_OK);
				binaryOut(temp.as_u8[3]);
				binaryOut(temp.as_u8[2]);
				binaryOut(temp.as_u8[1]);
				binaryOut(temp.as_u8[0]);
				break;

			case CMD_MOVE_DISTANCE:
				goto move;

			case CMD_GET_VERSION: {
				char i;
				__code
				char* p = version;
				binaryOut(ERR_OK);

				for (i = 0; i < 16; i++)
					binaryOut(*p++);
			}
				break;

				move: if (linelen < 4)
					goto missing_data;
				if (!stepperMove(motor, parseDoubleByte(), parseDoubleByte()))
					goto queue_full;
				binaryOut(ERR_OK);
				break;

				jog: if (linelen < 12)
					goto missing_data;
				if (!stepperJog(motor, parseDoubleByte(), parseDoubleByte(),
						parseDoubleByte(), forward, parseDoubleByte(),
						parseDoubleByte(), parseQuadByte()))
					goto queue_full;
				binaryOut(ERR_OK);
				break;

				missing_data: binaryOut(ERR_MISSING);
				break;

				queue_full: binaryOut(ERR_QUEUE_FULL);
				break;

			default:
				binaryOut(ERR_BAD_COMMAND);
				binaryOut(cmd);
				break;
			}
		}
	}

	{ // output response
		__data u8* p = rbuffer;
		u8 sum = 0x55;
		usbcdc_putchar((rlen + 1) | 0x80);

		while (rlen--) {
			u8 t = *p++;
			usbcdc_putchar(t);
			sum += t;
		}
		usbcdc_putchar(sum);
		usbcdc_flush();
	}
}

