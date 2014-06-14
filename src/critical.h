/*
 * File: critical.h
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

#ifndef CRITICAL_H_
#define CRITICAL_H_

extern volatile u8 waitInt; // defined in 'stepperirq.c'

#define CRITICAL(CODE) do { \
   static unsigned char __sdcc_iflags; \
   waitInt=1; \
   while(waitInt); \
   /*__sdcc_iflags = (INTCON & 0xC0);*/  \
   /* INTCON &= ~0xC0; */ \
   do { CODE; } while (0); \
   /* INTCON |= __sdcc_iflags; */ \
} while(0)

#define ENTER_CRITICAL_SECTION() do { \
   static unsigned char __sdcc_iflags; \
   __sdcc_iflags = (INTCON & 0xC0);  \
   INTCON &= ~0xC0; \
   do {

#define EXIT_CRITICAL_SECTION()  } while (0); \
   INTCON |= __sdcc_iflags; \
} while(0)

#endif /* CRITICAL_H_ */
