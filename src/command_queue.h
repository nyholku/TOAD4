/*
 * File: command_queue.h
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

#ifndef __COMMAND_QUEUE_H__
#define __COMMAND_QUEUE_H__

#include <stdint.h>
#include "types.h"
#include "toad4.h"
#include <pic18fregs.h>

#define QUEUE_CAPACITY 8

#define NUM_OF_MOTORS 4

#define MOTOR_X 0
#define MOTOR_Y 1
#define MOTOR_Z 2
#define MOTOR_4 3


typedef volatile struct { // size 4 bytes
	volatile int16_t move_distance;
	volatile uint16_t move_speed;
} command_queue_element_t;

typedef volatile struct { // size 8 * 4 = 32 bytes
	volatile command_queue_element_t queue[QUEUE_CAPACITY];
} command_queue_t;

extern volatile command_queue_t __at( 0x0300) g_command_queues[NUMBER_OF_MOTORS];

#define QUEUE_FULL(i)  (g_stepper_states[i].queue_size >= QUEUE_CAPACITY)

#define QUEUE_EMPTY(i)  (g_stepper_states[i].queue_size == 0)

#define QUEUE_PUSH(i) { \
		u8 rear = g_stepper_states[i].queue_rear; \
		if (rear < QUEUE_CAPACITY - 1) \
			rear++; \
		else \
			rear = 0; \
		g_stepper_states[i].queue_rear = rear; \
		g_stepper_states[i].queue_size++; \
		} \


#define QUEUE_POP(i) { \
		u8 front = g_stepper_states[i].queue_front; \
		if (front < QUEUE_CAPACITY - 1) \
			front++; \
		else \
			front = 0; \
		g_stepper_states[i].queue_front = front; \
		g_stepper_states[i].queue_size--; \
	} \

#define QUEUE_REAR(i) (&g_command_queues[i].queue[g_stepper_states[i].queue_rear])

#define QUEUE_FRONT(i) (&g_command_queues[i].queue[g_stepper_states[i].queue_front])

#define QUEUE_SIZE(i) (g_stepper_states[i].queue_size)

#define QUEUE_CLEAR(i) { \
		g_stepper_states[i].queue_front = 0; \
		g_stepper_states[i].queue_rear = 0; \
		g_stepper_states[i].queue_size = 0; \
	} \


typedef __data command_queue_t* command_queue_ptr_t;

#endif
