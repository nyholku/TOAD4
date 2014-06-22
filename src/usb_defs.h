/*
 * File: usb_defs.h
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

#ifndef USB_DEFS_H_
#define USB_DEFS_H_

#include <stdint.h>

// Define following SDCC extension as empty when parsing with Eclipse/CDT parser so that the parsing will not fail
// ... this may not be the most orthodox file to do this in but since this file is include everywhere this is handy location

#ifdef __CDT_PARSER__
#ifndef __SDCC_CDT_DEFS__
#define __SDCC_CDT_DEFS__
typedef unsigned char * va_list;
#define __at(x)
#define __sfr
#define __code
#define __data
#define __wparam
#define __interrupt(x)
#endif
#endif


// These need to go some where
typedef __code unsigned char* codePtr;
typedef __data unsigned char* dataPtr;

//
// Descriptor Types
//

#define DSC_DEV     0x01
#define DSC_CFG     0x02
#define DSC_STR     0x03
#define DSC_INTF    0x04
#define DSC_EP      0x05
#define DSC_HID     0x21
#define DSC_RPT     0x22
#define DSC_PHY     0x23

//
// USB Endpoint Definitions
//

#define _EP01_OUT   0x01
#define _EP01_IN    0x81
#define _EP02_OUT   0x02
#define _EP02_IN    0x82
#define _EP03_OUT   0x03
#define _EP03_IN    0x83
#define _EP04_OUT   0x04
#define _EP04_IN    0x84
#define _EP05_OUT   0x05
#define _EP05_IN    0x85
#define _EP06_OUT   0x06
#define _EP06_IN    0x86
#define _EP07_OUT   0x07
#define _EP07_IN    0x87
#define _EP08_OUT   0x08
#define _EP08_IN    0x88
#define _EP09_OUT   0x09
#define _EP09_IN    0x89
#define _EP10_OUT   0x0A
#define _EP10_IN    0x8A
#define _EP11_OUT   0x0B
#define _EP11_IN    0x8B
#define _EP12_OUT   0x0C
#define _EP12_IN    0x8C
#define _EP13_OUT   0x0D
#define _EP13_IN    0x8D
#define _EP14_OUT   0x0E
#define _EP14_IN    0x8E
#define _EP15_OUT   0x0F
#define _EP15_IN    0x8F

//
// Configuration Attributes
//

#define _DEFAULT    0x01<<7         //Default Value (Bit 7 is set)
#define _SELF       0x01<<6         //Self-powered (Supports if set)
#define _RWU        0x01<<5         //Remote Wakeup (Supports if set)

//
// Endpoint Transfer Type
//

#define _CTRL       0x00            //Control Transfer
#define _ISO        0x01            //Isochronous Transfer
#define _BULK       0x02            //Bulk Transfer
#define _INT        0x03            //Interrupt Transfer

//
// Isochronous Endpoint Synchronization Type
//

#define _NS         0x00<<2         //No Synchronization
#define _AS         0x01<<2         //Asynchronous
#define _AD         0x02<<2         //Adaptive
#define _SY         0x03<<2         //Synchronous

//
// Isochronous Endpoint Usage Type
//

#define _DE         0x00<<4         //Data endpoint
#define _FE         0x01<<4         //Feedback endpoint
#define _IE         0x02<<4         //Implicit feedback Data endpoint

//
//  USB Device Descriptor Structure
//

typedef struct {
	uint8_t bLength; //
	uint8_t bDscType; //
	uint16_t bcdUSB; //
	uint8_t bDevCls; //
	uint8_t bDevSubCls; //
	uint8_t bDevProtocol; //
	uint8_t bMaxPktSize0; //
	uint16_t idVendor; //
	uint16_t idProduct; //
	uint16_t bcdDevice; //
	uint8_t iMFR; //
	uint8_t iProduct; //
	uint8_t iSerialNum; //
	uint8_t bNumCfg; //
} usb_dev_desc_t; // size 18

//
// USB Configuration Descriptor Structure
//

typedef struct {
	uint8_t bLength; //
	uint8_t bDscType; //
	uint16_t wTotalLength; //
	uint8_t bNumIntf; //
	uint8_t bCfgValue; //
	uint8_t iCfg; //
	uint8_t bmAttributes; //
	uint8_t bMaxPower; //
} usb_cfg_desc_t; // size 9

//
// USB Interface Descriptor Structure
//

typedef struct {
	uint8_t bLength;  //
	uint8_t bDscType; //
	uint8_t bIntfNum; //
	uint8_t bAltSetting; //
	uint8_t bNumEPs; //
	uint8_t bIntfCls; //
	uint8_t bIntfSubCls;  //
	uint8_t bIntfProtocol; //
	uint8_t iIntf; //
} usb_intf_desc_t; // size 9

//
//  USB Endpoint Descriptor Structure
//

typedef struct {
	uint8_t bLength; //
	uint8_t bDscType; //
	uint8_t bEPAdr; //
	uint8_t bmAttributes; //
	uint16_t wMaxPktSize; //
	uint8_t bInterval; //
} usb_ep_desc_t;

//
// Standard Request Codes
//
#define GET_STATUS         0
#define CLEAR_FEATURE      1
#define SET_FEATURE        3
#define SET_ADDRESS        5
#define GET_DESCRIPTOR     6
#define SET_DESCRIPTOR     7
#define GET_CONFIGURATION  8
#define SET_CONFIGURATION  9
#define GET_INTERFACE     10
#define SET_INTERFACE     11
#define SYNCH_FRAME       12

//
// Standard Feature Selectors
//
#define DEVICE_REMOTE_WAKEUP    0x01
#define ENDPOINT_HALT           0x00

//
// Descriptor Types
//
#define DEVICE_DESCRIPTOR        0x01
#define CONFIGURATION_DESCRIPTOR 0x02
#define STRING_DESCRIPTOR        0x03
#define INTERFACE_DESCRIPTOR     0x04
#define ENDPOINT_DESCRIPTOR      0x05
#define QUALIFIER_DESCRIPTOR     0x06

//
// Device states
//
#define DETACHED     0
#define ATTACHED     1
#define POWERED      2
#define DEFAULT      3
#define ADDRESS      4
#define CONFIGURED   5

#endif /* USB_DEFS_H_ */
