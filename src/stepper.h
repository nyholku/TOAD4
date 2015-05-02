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

typedef struct { // 31 bytes
	volatile u8 state;
	volatile u8 home; // value of home input when 'at home'
	volatile u8 forwardDir;
	volatile uint16 accelNCO;
	union {
		volatile uint16 speedNCO;
		struct {
			volatile u8 speedNCO_lo;
			volatile u8 speedNCO_hi;
		};
	};
	volatile uint32 position;
	volatile uint32 stepCounter; // was 16 bit
	volatile uint16 motorSpeed;
	volatile u8 forward;
	volatile boolean accelerate;
	volatile uint16 jogFlag;
	volatile u8 probeArmed; //OBSOLETE
	volatile uint16 acceleration;
	volatile uint32 probePosition;
	volatile uint16 probeStopSpeed; //OBSOLETE
	volatile uint16 probeDeceleration; //OBSOLETE

	volatile u8 pop;
	volatile u8 rear;
	volatile u8 front;
	volatile u8 size;

	volatile u8 cmd;
	volatile u8 syncMask;
	union {
		struct { // Seek home command params
			uint32 seekTimeout;
			uint16 seekLoSpeed;
			uint16 seekHiSpeed;
			uint16 seekAcceleration;
		};
		struct { // Ramp command
			boolean rampForward :1;
			boolean rampAccelerate :1;
			uint16 rampDistance;
			uint16 rampAcceleration;
		};
		struct { // Jog command params
			boolean jogForward;
			uint16 jogPorch;
			uint16 jogCrawl;
			uint16 jogAcceleration;
			uint16 jogLoSpeed;
			uint16 jogHiSpeed;
			uint32 jogTimeout;
		};
	};

} stepperState;

typedef __data stepperState* pStepperState;

void stepperInit(volatile unsigned char motorNo);

void stepperAbort(u8 motorNo);

u8 stepperGetState(u8 motorNo);

void stepperGo(u8 motorNo);

u8 stepperArmProbe(u8 motorNo, uint16 stopSpeed, uint16 deceleration);

uint32 stepperGetProbePosition(u8 motorNo);

u8 stepperMove(u8 motorNo, int16 distance, uint16 speed);

uint32 stepperGetPosition(u8 motorNo);

void stepperSetPosition(u8 motorNo, uint32 position);

u8 stepperQueueSize(u8 motorNo);

u8 stepperQueueCapacity(u8 motorNo);

void stepperSetMode(u8 motorNo, u8 syncFlags, u8 enable, u8 torque, u8 dir, u8 home);

void stepperSetJogFlag(u8 motorNo, uint16 flag);

u8 stepperGetJogFlag(u8 motorNo);

u8 stepperGetHome(u8 motorNo);

u8 stepperSeekHome(u8 motorNo, uint16 loSpeed, uint16 hiSpeed,
		uint16 acceleration, uint32 timeout);

u8 stepperJog(u8 motorNo, uint16 loSpeed, uint16 hiSpeed, uint16 acceleration, u8 forward, uint16 porch, uint16 crawl, uint32 timeout);

u8 stepperConfigProbe(u8 input, u8 trigValue);

u8 stepperSetOutput(u8 outputPin, u8 outputState);

extern volatile stepperState __at( 0x0600 ) steppers[];
extern volatile cmdQueue __at ( 0x0300 ) queues[];
extern volatile unsigned char rx_timeout;
extern volatile unsigned char probeTrigValue;
extern volatile u8 syncCounter;
extern volatile unsigned char g_probeArmed;
extern volatile unsigned char g_probeTriggered;

#endif /* STEPPER_H */

