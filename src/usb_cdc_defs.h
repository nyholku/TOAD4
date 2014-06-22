/*
 * File: usb_cdc_defs.h
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

#ifndef USB_CDC_DEFS_H_
#define USB_CDC_DEFS_H_

#include "usb_defs.h"

#define PTR16(x) ((unsigned int)(((unsigned long)x) & 0xFFFF))

// Size of the buffer for endpoint 0
#define E0SZ 16

//
// CDC
//

#define CDC_COMM_INTF_ID        0x00
#define CDC_COMM_UEP            UEP2
#define CDC_INT_BD_IN           ep2Bi
#define CDC_INT_EP_SIZE         8

#define CDC_DATA_INTF_ID        0x01
#define CDC_DATA_UEP            UEP3
#define CDC_BULK_BD_OUT         ep3Bo
#define CDC_BULK_OUT_EP_SIZE    8
#define CDC_BULK_BD_IN          ep3Bi
#define CDC_BULK_IN_EP_SIZE     8

//
// Class-Specific Requests
//

#define SEND_ENCAPSULATED_COMMAND   0x00
#define GET_ENCAPSULATED_RESPONSE   0x01
#define SET_COMM_FEATURE            0x02
#define GET_COMM_FEATURE            0x03
#define CLEAR_COMM_FEATURE          0x04
#define SET_LINE_CODING             0x20
#define GET_LINE_CODING             0x21
#define SET_CONTROL_LINE_STATE      0x22
#define SEND_BREAK                  0x23

#define NETWORK_CONNECTION          0x00
#define RESPONSE_AVAILABLE          0x01
#define SERIAL_STATE                0x20

//
// Device Class Code
//
#define CDC_DEVICE                  0x02

//
// Communication Interface Class Code
//
#define COMM_INTF                   0x02

//
// Communication Interface Class SubClass Codes
//
#define ABSTRACT_CONTROL_MODEL      0x02

//
// Communication Interface Class Control Protocol Codes
//
#define V25TER                      0x01

//
// Data Interface Class Codes
//
#define DATA_INTF                   0x0A

//
// Data Interface Class Protocol Codes
//
#define NO_PROTOCOL                 0x00

//
// Communication Feature Selector Codes
//

#define ABSTRACT_STATE              0x01
#define COUNTRY_SETTING             0x02

//
// Functional Descriptors
//

#define CS_INTERFACE                0x24
#define CS_ENDPOINT                 0x25

//
// bDscSubType in Functional Descriptors
//

#define DSC_FN_HEADER               0x00
#define DSC_FN_CALL_MGT             0x01
#define DSC_FN_ACM                  0x02    // ACM - Abstract Control Management
#define DSC_FN_DLM                  0x03    // DLM - Direct Line Managment
#define DSC_FN_TELEPHONE_RINGER     0x04
#define DSC_FN_RPT_CAPABILITIES     0x05
#define DSC_FN_UNION                0x06
#define DSC_FN_COUNTRY_SELECTION    0x07
#define DSC_FN_TEL_OP_MODES         0x08
#define DSC_FN_USB_TERMINAL         0x09

//
// CDC Bulk IN transfer states
//
#define CDC_TX_READY                0
#define CDC_TX_BUSY                 1
#define CDC_TX_BUSY_ZLP             2       // ZLP: Zero Length Packet
#define CDC_TX_COMPLETING           3

//
// Line Coding Structure
//

typedef struct {
	uint8_t bLinecoding[7];
} line_coding_t;

//
// Header Functional Descriptor
//

typedef struct {
	uint8_t bFNLength; //
	uint8_t bDscType; //
	uint8_t bDscSubType; //
	uint16_t bcdCDC; //
} usb_cdc_header_fn_desc_t; //

//
// Abstract Control Management Functional Descriptor
//

typedef struct {
	uint8_t bFNLength; //
	uint8_t bDscType; //
	uint8_t bDscSubType; //
	uint8_t bmCapabilities; //
} usb_cdc_acm_fn_desc_t;

//
// Union Functional Descriptor
//

typedef struct {
	uint8_t bFNLength; //
	uint8_t bDscType; //
	uint8_t bDscSubType; //
	uint8_t bMasterIntf; //
	uint8_t bSaveIntf0; //
} usb_cdc_union_fn_desc_t;

//
// Call Management Functional Descriptor
//

typedef struct {
	uint8_t bFNLength; //
	uint8_t bDscType; //
	uint8_t bDscSubType; //
	uint8_t bmCapabilities; //
	uint8_t bDataInterface; //
} usb_cdc_call_mgt_fn_desc_t;

#endif /* USB_CDC_DEFS_H */
