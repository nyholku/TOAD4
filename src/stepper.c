/*
 * File: stepper.c
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
#ifndef __STEPPER_C__

#include <stdio.h>
#include <string.h>
#include "stepper.h"
#include "toad4.h"
#include "critical.h"

volatile unsigned char rx_timeout = 0;
volatile unsigned char probeTrigValue = 0;

#define STEPPER (*__stepper)
#define QUEUE (*__queue)

#define USE_STEPPER(CODE) do { \
	__data stepperState* __stepper=&steppers[motorNo]; \
	__data cmdQueue* __queue=&queues[motorNo]; \
	do { CODE; } while (0); \
	} while(0)

void stepperInit(volatile unsigned char motorNo) {
	//	 __data stepperState* stepper=&steppers[motorNo];
	USE_STEPPER(
			//uint32 pos=STEPPER.position; // don't lose the position on reset
			memset(&(STEPPER),0,sizeof(stepperState));
			//pos=STEPPER.position=pos;
			STEPPER.state = STATE_STOP;
	);
}

void stepperAbort(u8 motorNo) {
	char t;
	USE_STEPPER(CRITICAL(
					STEPPER.state = STATE_STOP;
					STEPPER.stepCounter = 0;
					QUEUE_CLEAR(STEPPER);
			));
}

u8 stepperGetState(u8 motorNo) {
	char t;
	USE_STEPPER(CRITICAL(
			if (STEPPER.cmd!=CMD_NONE)
					t = STATE_COMMAND_PENDING;
			else
					t = STEPPER.state;
			));
	return t;
}

uint32 stepperGetPosition(u8 motorNo) {
	uint32 t;
	USE_STEPPER(CRITICAL(
					t=STEPPER.position;
			));
	return t;
}

void stepperSetPosition(u8 motorNo, uint32 position) {
	uint32 t;
	USE_STEPPER(CRITICAL(
					STEPPER.position = position;
			));
}

void stepperGo(u8 motorNo) {
	USE_STEPPER(CRITICAL(
			if (STEPPER.state < STATE_PROCESS_QUEUE)
					STEPPER.state = STATE_PROCESS_QUEUE;
			));
}

u8 stepperQueueSize(u8 motorNo) {
	u8 size;
	USE_STEPPER(CRITICAL(
					size=QUEUE_SIZE(STEPPER);
			));
	return size;
}

u8 stepperQueueCapacity(u8 motorNo) {
	u8 capacity;
	USE_STEPPER(CRITICAL(
					capacity=QUEUE_CAPACITY;
			));
	return capacity;
}

u8 stepperMove(u8 motorNo, int16 distance, uint16 speed) {
	u8 ok = 0;
	USE_STEPPER(CRITICAL(
					if (!QUEUE_FULL(STEPPER)) {
						pStepperCmd cp=QUEUE_REAR(STEPPER);
						cp->moveSpeed = speed;
						cp->moveDistance = distance;
						QUEUE_PUSH(STEPPER);
						ok=1;
					}

			));
	return ok;
}

void stepperSetJogFlag(u8 motorNo, uint16 flag) {
	USE_STEPPER(CRITICAL(
					STEPPER.jogFlag = flag;
			));
}

u8 stepperGetJogFlag(u8 motorNo) {
	u8 flag;
	USE_STEPPER(CRITICAL(
					flag= STEPPER.jogFlag;
			));
	return flag;
}

u8 stepperGetHome(u8 motorNo) {
	switch (motorNo) {
	case MOTOR_X:
		return HOME_X == steppers[MOTOR_X].home;
	case MOTOR_Y:
		return HOME_Y == steppers[MOTOR_Y].home;
	case MOTOR_Z:
		return HOME_Z == steppers[MOTOR_Z].home;
	case MOTOR_4:
		return HOME_4 == steppers[MOTOR_4].home;
	default:
		return 0;
	}
}

/*

 Step mode selection

 M2 M1   mode 		pulses for full cycle (Toad3 motor Z, M2=M1)
 0  0    2          4
 0  1    1-2        8
 1  0    W1-2      16
 1  1    2W1-2     32 (this can't be right)

 Note, TOAD3 proto 1, after mods:
 motor X,Y  M2 = 0 => M1=> stepsize 4/8
 motor Z, M1==M2 => stepsize 4 / 32

 Y2 Y1    decay mode  (Toad3 Y2=Y1)
 0  0    Normal (slow) decay
 0  1    25% fast decay
 1  0    50% fast decay
 1  1    Fast decay

 Q2 Q1
 0  0    100% current
 0  1    75% current
 1  0    50% current
 1  1    20% current



 */
void stepperSetMode(u8 motorNo, u8 syncFlags, u8 enable, u8 torque, u8 dir,
		u8 home) {

#if TOAD_HW_VERSION==HW3
	u8 tq1, tq2, dcy;
	u8 current=20;
	u8 steps = 1;
	u8 decay=0;
	// map current to outputs
	if (torque) {
		tq2 = 0;// => 100%
		tq1 = 0;
		}
	else {
		tq2 = 1;// => 20%
		tq1 = 1;
	}

	dcy = 1;

	switch (motorNo) {
		case MOTOR_X:
			steppers[MOTOR_X].syncMask = syncFlags;
			steppers[MOTOR_X].forwardDir = dir;
			steppers[MOTOR_X].home = home;
			DCY1_X = dcy;
			TQ1_X = tq1;
			TQ2_X = tq2;
			MODE1_X = 1;
			ENABLE_X = enable;
			break;
		case MOTOR_Y:
			steppers[MOTOR_Y].syncMask = syncFlags;
			steppers[MOTOR_Y].forwardDir = dir;
			steppers[MOTOR_Y].home = home;
			DCY1_Y = dcy;
			TQ1_Y = tq1;
			TQ2_Y = tq2;
			MODE1_Y = 1;
			ENABLE_Y = enable;
			break;
		case MOTOR_Z:
			steppers[MOTOR_Z].syncMask = syncFlags;
			steppers[MOTOR_Z].forwardDir = dir;
			steppers[MOTOR_Z].home = home;
			DCY1_Z = dcy;
			TQ1_Z = tq1;
			TQ2_Z = tq2;
			MODE1_Z = 0;
			ENABLE_Z = enable;
			break;
		case MOTOR_4:
		break;
		default:
		;
	}
#endif

#if TOAD_HW_VERSION==HW4

	switch (motorNo) {
	case MOTOR_X:
		steppers[MOTOR_X].syncMask = syncFlags;
		steppers[MOTOR_X].forwardDir = dir;
		steppers[MOTOR_X].home = home;
		TORQUE_X = !torque;
		ENABLE_X = enable;
		break;
	case MOTOR_Y:
		steppers[MOTOR_Y].syncMask = syncFlags;
		steppers[MOTOR_Y].forwardDir = dir;
		steppers[MOTOR_Y].home = home;
		TORQUE_Y = !torque;
		ENABLE_Y = enable;
		break;
	case MOTOR_Z:
		steppers[MOTOR_Z].syncMask = syncFlags;
		steppers[MOTOR_Z].forwardDir = dir;
		steppers[MOTOR_Z].home = home;
		TORQUE_Z = !torque;
		ENABLE_Z = enable;
		break;
	case MOTOR_4:
		steppers[MOTOR_4].syncMask = syncFlags;
		steppers[MOTOR_4].forwardDir = dir;
		steppers[MOTOR_4].home = home;
		TORQUE_4 = !torque;
		ENABLE_4 = enable;
		break;
	default:
		;
	}

#endif

}


u8 stepperSeekHome(u8 motorNo, uint16 loSpeed, uint16 hiSpeed,
		uint16 acceleration, uint32 timeout) {
	u8 ok = 0;
	USE_STEPPER(CRITICAL(
					pStepperState mp=&STEPPER;
					if (mp->state==STATE_TIMEOUT)
					mp->state=STATE_PROCESS_QUEUE;
					if (mp->state==STATE_PROCESS_QUEUE && mp->cmd==CMD_NONE) {
						mp->cmd = CMD_SEEK;
						mp->seekLoSpeed=loSpeed;
						mp->seekHiSpeed=hiSpeed;
						mp->seekTimeout=timeout;
						mp->seekAcceleration=acceleration;
						ok=1;
					}
			));
	return ok;
}

u8 stepperJog(u8 motorNo, uint16 loSpeed, uint16 hiSpeed, uint16 acceleration,
		u8 forward, uint16 porch, uint16 crawl,uint32 timeout) {
	u8 ok = 0;
	USE_STEPPER(CRITICAL(
					pStepperState mp=&STEPPER;
					if (mp->state==STATE_TIMEOUT)
					mp->state=STATE_PROCESS_QUEUE;
					if (mp->state==STATE_PROCESS_QUEUE && mp->cmd==CMD_NONE) {
						mp->cmd = CMD_JOG;
						mp->jogForward=forward;
						mp->jogPorch=porch;
						mp->jogCrawl=crawl;
						mp->jogAcceleration=acceleration;
						mp->jogLoSpeed=loSpeed;
						mp->jogHiSpeed=hiSpeed;
						mp->jogTimeout=timeout;
						ok=1;
					}
			));
	return ok;
}

u8 stepperArmProbe(u8 motorNo, uint16 stopSpeed, uint16 deceleration) {
	u8 ok = 0;
	USE_STEPPER(CRITICAL(
					STEPPER.probeArmed=TRUE;
					STEPPER.probeStopSpeed=stopSpeed;
					STEPPER.probeDeceleration=deceleration;
			));
	return ok;
}

u8 stepperConfigProbe(u8 input, u8 trigValue) {
	u8 ok = 0;
	probeTrigValue = trigValue;
	return ok;
}

uint32 stepperGetProbePosition(u8 motorNo) {
	uint32 t;
	USE_STEPPER(CRITICAL(
					if (STEPPER.probeArmed) {
						STEPPER.probeArmed = FALSE;
						t = 0x80000000;
					}
					else
					t=STEPPER.probePosition;
			));
	return t;
}

u8 stepperSetOutput(u8 outputPin, u8 outputState) {
	u8 ok = 0;

#if TOAD_HW_VERSION==HW3
	switch (outputPin) {
	case 0:
		RELAY_PIN = outputState;
		break;
	default:
		break;
	}
#endif

#if TOAD_HW_VERSION==HW4
	switch (outputPin) {
	case 0:
		SPINDLE_FWD=outputState;
		break;
	case 1 :
		SPINDLE_REV=outputState;
		break;
	case 2:
		CCPR2L = outputState;
		break;
	case 3:
		COOLANT = outputState;
		break;
	default:
		break;
	}
#endif

	return ok;
}

#endif /* __STEPPER_C__ */
