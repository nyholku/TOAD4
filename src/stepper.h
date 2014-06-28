/*
 * File: stepper.h
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

#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "types.h"
#include <stdint.h>

#define QUEUE_CAPACITY 16

#define STATE_STOP 0
#define STATE_TIMEOUT 1
#define STATE_PROCESS_QUEUE 2
#define STATE_RUN 3

#define STATE_SEEK_HOME_RAMP_UP 4
#define STATE_SEEK_HOME_RUN 5
#define STATE_SEEK_HOME_RAMP_DOWN 6

#define STATE_SEEK_NOT_HOME_RAMP_UP 7
#define STATE_SEEK_NOT_HOME_RUN 8
#define STATE_SEEK_NOT_HOME_RAMP_DOWN 9

#define STATE_SEEK_CRAWL_HOME 10
#define STATE_SEEK_CRAWL_NOT_HOME 11

#define STATE_JOG_PORCH 12
#define STATE_JOG_CRAWL 13
#define STATE_JOG_RUN 14

#define STATE_PROBE_DECELERATION 15

#define STATE_COMMAND_PENDING 16

#define ERROR_TIMEOUT 1

#define NUM_OF_MOTORS 4

#define MOTOR_X 0
#define MOTOR_Y 1
#define MOTOR_Z 2
#define MOTOR_4 3

#define CMD_NONE 0
#define CMD_RAMP 1
#define CMD_MOVE 2
#define CMD_SEEK 3
#define CMD_JOG 4

#define WAIT_NONE 0
#define WAIT_HOME 1
#define WAIT_JOG 2

typedef struct { // size 4 bytes
	volatile int16 moveDistance;
	volatile uint16 moveSpeed;
} stepperCmd;

typedef struct { // size 16 * 4 = 64 bytes
	volatile stepperCmd queue[QUEUE_CAPACITY];
} cmdQueue;

typedef __data stepperCmd* pStepperCmd;

#define QUEUE_FULL(qp)  (qp.size >= QUEUE_CAPACITY)

#define QUEUE_EMPTY(qp)  (qp.size == 0)

#define QUEUE_PUSH(qp) {\
		u8 rear = qp.rear;\
		if (rear < QUEUE_CAPACITY - 1)\
			rear++;\
		else\
			rear = 0;\
		qp.rear = rear;\
		qp.size++;\
		}\


#define QUEUE_POP(qp) {\
		u8 front = qp.front;\
		if (front < QUEUE_CAPACITY - 1)\
			front++;\
		else\
			front = 0;\
		qp.front = front;\
		qp.size--;\
	}\

#define QUEUE_REAR(qp) (&QUEUE.queue[qp.rear])

#define QUEUE_FRONT(qp) (&QUEUE.queue[qp.front])

#define QUEUE_SIZE(qp) (qp.size)

#define QUEUE_CLEAR(qp) {\
		qp.front = syncCounter;\
		qp.rear = syncCounter;\
		qp.size = 0;\
	}\


typedef __data cmdQueue* pCmdQueue;

extern volatile cmdQueue __at ( 0x0300 ) queues[];

typedef struct {
	uint16_t nco; // offset 0
	uint16_t speed; // offset 2
	uint16_t next_speed; // offset 4
	uint8_t steps; // offset 6
	uint8_t next_steps; // offset 7
	struct { //offset 8
		unsigned has_next :1;
		unsigned next_dir :1;
		unsigned last_dir :1;
		unsigned reserve_b3 :1;
		unsigned reserve_b4 :1;
		unsigned reserve_b5 :1;
		unsigned reserve_b6 :1;
		unsigned reserve_b7 :1;
	};
	uint8_t reserve[7];
} stepper_state_t;

typedef struct {
	struct { // packing 'booleans' like this into one bit fields allows faster code generation on SDCC
		unsigned irq_flag :1; // High priority interrupt clears this
		unsigned reserve_b1;
		unsigned reserve_b2 :1;
		unsigned reserve_b3 :1;
		unsigned reserve_b4 :1;
		unsigned reserve_b5 :1;
		unsigned reserve_b6 :1;
		unsigned reserve_b7 :1;
	};
} irq_flags_t;

extern stepper_state_t g_stepper_states[4];
extern irq_flags_t g_irq_flags;

#endif /* STEPPER_H */

