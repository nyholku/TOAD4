/*
 * File: hispeedirq.h
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

#ifndef HISPEEDIRQ_H_
#define STEPPERIRQ_H_

#include <stdint.h>

// group all global varibles to a single struct to hint SDCC that there is no need
// for all those BANKSEL statements...improves the code size and speed a lot

typedef struct {
	union {
		uint16_t nco;
		struct {
			unsigned nco_low_bits :15;
			unsigned nco_hi_bit :1;
		};
	};
	uint16_t speed;
	uint8_t steps;
	struct { // packing 'booleans' like this into one bit fields allows faster code generation on SDCC
		unsigned has_next :1;
		unsigned next_dir:1;
		unsigned reserve_b2 :1;
		unsigned reserve_b3 :1;
		unsigned reserve_b4 :1;
		unsigned reserve_b5 :1;
		unsigned reserve_b6 :1;
		unsigned reserve_b7 :1;
	};
	uint16_t next_speed;
	uint8_t next_steps;
} motor_t;
/*
typedef struct uint8_as_bits_t {
	unsigned b0 :1;
	unsigned b1 :1;
	unsigned b2 :1;
	unsigned b3 :1;
	unsigned b4 :1;
	unsigned b5 :1;
	unsigned b6 :1;
	unsigned b7 :1;
} uint8_as_bits_t;
union {
	uint8_t port_A;
	uint8_as_bits_t port_A_bits;
};
union {
	uint8_t port_B;
	uint8_as_bits_t port_B_bits;
};
union {
	uint8_t port_C;
	uint8_as_bits_t port_C_bits;
};
union {
	uint8_t port_D;
	uint8_as_bits_t port_D_bits;
};
*/
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
	motor_t motor[4];
} globals_t;

extern globals_t g;

void init_stepperirq_test();

#endif /* HISPEEDIRQ_H_ */
