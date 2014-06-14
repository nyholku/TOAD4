/*
 * File: stepperirq-inc.c
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

#define	STEPPER steppers[MOTOR_X]
#define QUEUE queues[MOTOR_X]
#define STEP_OUTPUT STEP_X
#define DIR_OUTPUT DIR_X
#define ENABLE ENABLE_X
#define HOME_INPUT HOME_X
#define STEPS_LEFT stepsLeft.motorX
#include "stepperirq-inc-inc.c"

#define	STEPPER steppers[MOTOR_Y]
#define QUEUE queues[MOTOR_Y]
#define STEP_OUTPUT STEP_Y
#define DIR_OUTPUT DIR_Y
#define ENABLE ENABLE_Y
#define HOME_INPUT HOME_Y
#define STEPS_LEFT stepsLeft.motorY
#include "stepperirq-inc-inc.c"

#define	STEPPER steppers[MOTOR_Z]
#define QUEUE queues[MOTOR_Z]
#define STEP_OUTPUT STEP_Z
#define DIR_OUTPUT DIR_Z
#define ENABLE ENABLE_Z
#define HOME_INPUT HOME_Z
#define STEPS_LEFT stepsLeft.motorZ
#include "stepperirq-inc-inc.c"

#define DUMMY dummy
#define	STEPPER steppers[MOTOR_4]
#define QUEUE queues[MOTOR_4]
#define STEP_OUTPUT STEP_4
#define DIR_OUTPUT DIR_4
#define HOME_INPUT HOME_4
#define STEPS_LEFT stepsLeft.motor4
#include "stepperirq-inc-inc.c"

#undef STEP_GENERATION
#undef QUEUE_PROCESSING
