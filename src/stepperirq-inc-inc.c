/*
 * File: stepperirq-inc-inc.c
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
#define HOME (STEPPER.home ? HOME_INPUT==1 : HOME_INPUT==0)

#define FORWARD (STEPPER.forwardDir)
#define REVERSE (!STEPPER.forwardDir)

// Following declarations make the CDT/Eclipse parser happy, necessary
// because  it does not understand how this file included from an included file
// Note that this is all fake in the sense that when compiled none of this is
// used, this is just to make editing in Eclipse cleaner
#ifdef __CDT_PARSER__
#define STEP_GENERATION
#define QUEUE_PROCESSING
#define	STEPPER steppers[MOTOR_X]
#define QUEUE queues[MOTOR_X]
#define STEP_OUTPUT STEP_X
#define DIR_OUTPUT DIR_X
#define ENABLE ENABLE_X
#define HOME_INPUT HOME_X
#define STEPS_LEFT stepsLeft.motorX
void dummy_function_declaration()
#endif

#ifdef STEP_GENERATION
{
	//qp = &QUEUE;

	STEP_OUTPUT = 0; // Step pulse off in case it was on

	if (STEPPER.probeArmed && probeInput) {
		STEPPER.probeArmed = FALSE;
		STEPPER.probePosition = STEPPER.position;
		STEPPER.state = STATE_PROBE_DECELERATION;
		STEPPER.acceleration = STEPPER.probeDeceleration;
		STEPPER.accelerate=FALSE;
	}

	if (STEPPER.stepCounter) {
		// Do the step pulse generation
		oldv = STEPPER.speedNCO;
		STEPPER.speedNCO += STEPPER.motorSpeed;
		if (STEPPER.speedNCO < oldv) { // speed NCO overflow => generate pulse
			if (STEPPER.motorSpeed) {
				STEP_OUTPUT = 1;
#ifdef BLINK_ON_STEP
				LED_PIN=1;
				LED_PIN=0;
#endif
				STEPPER.stepCounter--;
				if (STEPPER.forward)
				STEPPER.position++;
				else
				STEPPER.position--;
			}
		}
		// Do the acceleration
		oldv = STEPPER.accelNCO;
		//STEPPER.acceleration = 10000;
		STEPPER.accelNCO += STEPPER.acceleration;
		if (STEPPER.accelNCO < oldv) { // acceleration NCO overflow => accelerate
			if (STEPPER.accelerate) {
				if (STEPPER.motorSpeed <= 65535-4) // FIXME do we need this guard
				STEPPER.motorSpeed+=4;
				else
				STEPPER.motorSpeed=65535;
			} else {
				if (STEPPER.motorSpeed > 4) {
					STEPPER.motorSpeed-=4;
				}
				else
				STEPPER.motorSpeed=1;
			}
		}

	}

	if (STEPPER.jogFlag)
	STEPPER.jogFlag--;

	stepsleft = STEPPER.stepCounter != 0;
	STEPS_LEFT = stepsleft;

	switch (STEPPER.state) {
		case STATE_TIMEOUT:
		STEPPER.stepCounter = 0;
		break;
		case STATE_RUN:
		if (!stepsleft) {
			STEPPER.state = STATE_PROCESS_QUEUE;
			QUEUE_POP(STEPPER);
		}
		break;
		case STATE_PROBE_DECELERATION:
		if (STEPPER.motorSpeed < STEPPER.probeStopSpeed) {
			STEPPER.state = STATE_PROCESS_QUEUE;
			QUEUE_CLEAR(STEPPER);
			STEPPER.stepCounter = 0;
		}
		break;
		case STATE_SEEK_HOME_RAMP_UP:
		if (HOME || (STEPPER.motorSpeed >= STEPPER.seekHiSpeed)) {
			STEPPER.motorSpeed = STEPPER.seekHiSpeed;
			STEPPER.state = STATE_SEEK_HOME_RUN;
			STEPPER.acceleration = 0;
		}
		break;
		case STATE_SEEK_HOME_RUN:
		if (HOME) {
			STEPPER.state = STATE_SEEK_HOME_RAMP_DOWN;
			STEPPER.accelerate = FALSE;
			STEPPER.acceleration = STEPPER.seekAcceleration;
		} else {
			if (!stepsleft)
			STEPPER.state = STATE_TIMEOUT;
		}
		break;
		case STATE_SEEK_HOME_RAMP_DOWN:
		if (STEPPER.motorSpeed <= STEPPER.seekLoSpeed) {
			STEPPER.state = STATE_SEEK_CRAWL_NOT_HOME;
			STEPPER.motorSpeed = STEPPER.seekLoSpeed;
			STEPPER.acceleration = 0;
			STEPPER.forward = FALSE;
			DIR_OUTPUT = REVERSE;
		}
		break;
		case STATE_SEEK_NOT_HOME_RAMP_UP:
		if (!HOME || (STEPPER.motorSpeed >= STEPPER.seekHiSpeed)) {
			STEPPER.motorSpeed = STEPPER.seekHiSpeed;
			STEPPER.state = STATE_SEEK_NOT_HOME_RUN;
			STEPPER.acceleration = 0;
		}
		break;
		case STATE_SEEK_NOT_HOME_RUN:
		if (!HOME) {
			STEPPER.state = STATE_SEEK_NOT_HOME_RAMP_DOWN;
			STEPPER.accelerate = FALSE;
			STEPPER.acceleration = STEPPER.seekAcceleration;
		} else {
			if (!stepsleft)
			STEPPER.state = STATE_TIMEOUT;
		}
		break;
		case STATE_SEEK_NOT_HOME_RAMP_DOWN:
		if (STEPPER.motorSpeed <= STEPPER.seekLoSpeed) {
			STEPPER.state = STATE_SEEK_CRAWL_HOME;
			STEPPER.motorSpeed = STEPPER.seekLoSpeed;
			STEPPER.acceleration = 0;
			STEPPER.forward = TRUE;
			DIR_OUTPUT = FORWARD;
		}
		break;

		case STATE_SEEK_CRAWL_HOME:
		if (HOME) {
			STEPPER.state = STATE_SEEK_CRAWL_NOT_HOME;
			STEPPER.forward = FALSE;
			DIR_OUTPUT = REVERSE;
		} else if (!stepsleft)
		STEPPER.state = STATE_TIMEOUT;
		break;
		case STATE_SEEK_CRAWL_NOT_HOME:
		if (!HOME) {
			STEPPER.state = STATE_PROCESS_QUEUE;
			STEPPER.stepCounter = 0;
		} else if (!stepsleft)
		STEPPER.state = STATE_TIMEOUT;
		break;
		case STATE_JOG_PORCH:
		if (STEPPER.jogPorch != 0) {
			if (!stepsleft)
			STEPPER.jogPorch--;
			STEPPER.stepCounter = 1;
			break;
		}
		case STATE_JOG_CRAWL:
		if (STEPPER.jogCrawl != 0) {
			if (!stepsleft) {
				STEPPER.jogCrawl--;
				if (STEPPER.jogFlag == 0) {
					STEPPER.stepCounter = 0;
					STEPPER.state = STATE_PROCESS_QUEUE;
					break;
				}
			}
			STEPPER.stepCounter = 1;
			break;
		}
		STEPPER.state = STATE_JOG_RUN; // fall through to next case
		case STATE_JOG_RUN:
		if (STEPPER.jogTimeout != 0) {
			//if (!stepsleft) {
			STEPPER.jogTimeout--;
			STEPPER.stepCounter = 1;
			//}
		}
		{ //if (!stepsleft) {
			if (STEPPER.jogFlag == 0 || STEPPER.jogTimeout == 0) {
				if (STEPPER.motorSpeed > STEPPER.jogLoSpeed) {
					STEPPER.accelerate = FALSE;
					STEPPER.acceleration = STEPPER.jogAcceleration;
					STEPPER.stepCounter = 1;
				} else {
					STEPPER.state = STATE_PROCESS_QUEUE;
					STEPPER.acceleration = 0;
					STEPPER.stepCounter = 0;
				}
			} else {
				if (STEPPER.motorSpeed < STEPPER.jogHiSpeed) {
					STEPPER.accelerate = TRUE;
					STEPPER.acceleration = STEPPER.jogAcceleration;
					STEPPER.stepCounter = 1;
				} else {
					STEPPER.acceleration = 0;
					STEPPER.stepCounter = 1;
				}
			}
		}
		break;
		default:
		;
	}
	STEPPER.pop = FALSE;
	if (STEPPER.syncMask) {
		syncN++;
		if (STEPPER.state == STATE_PROCESS_QUEUE && STEPPER.front == syncCounter && STEPPER.size != 0) {
			STEPPER.pop = TRUE;
			syncC++;
		}
	}

}
#endif

#ifdef __CDT_PARSER__
void dummy_function_declaration2()
#endif

#ifdef QUEUE_PROCESSING
{
	if (inSync && STEPPER.pop) {
		STEPPER.state = STATE_RUN;
		STEPPER.cmd = CMD_NONE;
		cp = QUEUE_FRONT(STEPPER);
		move = cp->moveDistance;
		if (move < 0) {
			move = -move;
			forward = 0;
		} else
		forward = 1;
		DIR_OUTPUT = forward ? FORWARD : REVERSE;
		STEPPER.forward = forward;
		STEPPER.motorSpeed = cp->moveSpeed;
		STEPPER.stepCounter = move;
		STEPPER.acceleration = 0;
		STEPPER.accelerate = 0;
	}

	if (STEPPER.state == STATE_PROCESS_QUEUE) {
		switch (STEPPER.cmd) {
			default:
			case CMD_NONE:
			break;
			case CMD_JOG:
			STEPPER.state = STATE_JOG_PORCH;
			STEPPER.cmd = CMD_NONE;
			DIR_OUTPUT = STEPPER.jogForward ? FORWARD : REVERSE;
			STEPPER.forward = STEPPER.jogForward;
			STEPPER.stepCounter = 1;
			STEPPER.motorSpeed = STEPPER.jogLoSpeed;
			STEPPER.acceleration = 0;
			break;
			case CMD_SEEK:
			if (HOME) {
				STEPPER.state = STATE_SEEK_NOT_HOME_RAMP_UP;
				STEPPER.cmd = CMD_NONE;
				DIR_OUTPUT = REVERSE;
				STEPPER.forward = FALSE;
				STEPPER.stepCounter = STEPPER.seekTimeout;
				STEPPER.acceleration = STEPPER.seekAcceleration;
				STEPPER.accelerate = TRUE;
			} else {
				STEPPER.state = STATE_SEEK_HOME_RAMP_UP;
				STEPPER.cmd = CMD_NONE;
				DIR_OUTPUT = FORWARD;
				STEPPER.forward = TRUE;
				STEPPER.stepCounter = STEPPER.seekTimeout;
				STEPPER.acceleration = STEPPER.seekAcceleration;
				STEPPER.accelerate = TRUE;
			}
			break;
		}
	}
}
#endif

#undef STEPPER
#undef QUEUE
#undef STEP_OUTPUT
#undef DIR_OUTPUT
#undef ENABLE
#undef TORQUE_1
#undef TORQUE_2
#undef MODE
#undef HOME
#undef HOME_INPUT
#undef STEPS_LEFT
