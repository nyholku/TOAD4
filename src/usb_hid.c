/*
 * File: usb_hid.c
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

#include "types.h"
#include "pic18f4550.h"
#include "usbpic_defs.h"
#include "usb_defs.h"
#include "toad4.h"

#define	HID_GET_REPORT  0x1
#define	HID_GET_IDLE 0x2
#define	HID_GET_PROTOCOL 0x3
#define	HID_SET_REPORT 0x9
#define	HID_SET_IDLE 0xa
#define	HID_SET_PROTOCOL 0xb

/*
 *                #pragma config PLLDIV   = 5
 #pragma config CPUDIV   = OSC1_PLL2
 #pragma config USBDIV   = 2
 #pragma config FOSC     = HSPLL_HS
 #pragma config FCMEN    = OFF
 #pragma config IESO     = OFF
 #pragma config PWRT     = OFF
 #pragma config BOR      = ON
 #pragma config BORV     = 3
 #pragma config VREGEN   = ON
 #pragma config WDT      = OFF
 #pragma config WDTPS    = 32768
 #pragma config MCLRE    = ON
 #pragma config LPT1OSC  = OFF
 #pragma config PBADEN   = OFF
 #pragma config STVREN   = ON
 #pragma config LVP      = OFF
 #pragma config XINST    = OFF
 #pragma config CP0      = OFF
 #pragma config CP1      = OFF
 #pragma config CPB      = OFF
 #pragma config WRT0     = OFF
 #pragma config WRT1     = OFF
 #pragma config WRTB     = OFF
 #pragma config WRTC     = OFF
 #pragma config EBTR0    = OFF
 #pragma config EBTR1    = OFF
 #pragma config EBTRB    = OFF


 #if defined(__18F2550) || defined(__18F2553)||  defined(__18F4550) || defined(__18F4553)

 #pragma config CP3      = OFF
 #pragma config WRT3     = OFF
 #pragma config EBTR3    = OFF
 *
 */
typedef __code unsigned char* codePtr;
typedef __data unsigned char* dataPtr;

// /Users/nyholku/usbcdcacm-junk/USB_CDC_skeleton_15/USB_Descriptor.h
#define HID_INTF                    0x03
#define E0SZ                   		8

typedef struct _USB_HID_DSC {
	unsigned char bLength;
	unsigned char bDescriptorType;
	unsigned short bcdHID;
	unsigned char bCountryCode;
	unsigned char bNumDescriptors;
	unsigned char bReportDescriptorType;
	unsigned short wReportDescriptorLength;
} USB_HID_DSC;

typedef struct {
	USB_CFG_DSC cd01;
	USB_INTF_DSC i01a00;
	USB_HID_DSC hid_01a00;
	USB_EP_DSC ep01i_i01a00;
	USB_EP_DSC ep02i_i01a00;
} config_struct;

__code USB_DEV_DSC device_descriptor = { //
		sizeof(USB_DEV_DSC),    		// bLength
				DSC_DEV,                // bDescriptorType
				0x0200,                 // bcdUSB lsb, bcdUSB msb
				0x00,                   // bDeviceClass
				0x00,                   // bDeviceSubClass
				0x00,                   // bDeviceProtocl
				E0SZ,                   // bMaxPacketSize
				0x1D50,				    // idVendor
				0x6020,				    // idProduct
				0x0100,                 // bcdDevice
				0x01,                   // iManufacturer
				0x02,                   // iProduct
				0x00,                   // iSerialNumber
				0x01                    // bNumConfigurations
		};

// Configuration 1 Descriptor
__code config_struct config_descriptor = { //
		{
		// Configuration Descriptor
				0x09,// Size of this descriptor in bytes
				DSC_CFG,	                    // CONFIGURATION descriptor type
				sizeof(config_struct),			// Total length of data for this cfg
				1,								// bNumInterfaces
				1,								// bConfigurationValue
				0,								// iConfiguration
				0x80 | _SELF,					// bmAttributes
				1,		    					// bMaxPower
				}, {
				// Interface Descriptor
						sizeof(USB_INTF_DSC),			// Size of this descriptor in bytes
						DSC_INTF,		                // INTERFACE descriptor type
						0,								// Interface Number
						0,								// Alternate Setting Number
						2,								// Number of endpoints in this intf
						HID_INTF,						// Class code
						0,								// Subclass code
						0,								// Protocol code
						0,								// Interface string index
				}, {
				// HID Class-Specific Descriptor
						sizeof(USB_HID_DSC),			// Size of this descriptor in bytes (sizeof(USB_HID_DSC)+3)
						DSC_HID,						// HID descriptor type
						0x0111,							// HID Spec Release Number in BCD format (1.11)
						0x00,							// Country Code (0x00 for Not supported)
						1,					            // Number of class descriptors, see usbcfg.h
						DSC_RPT,						// Report descriptor type
						sizeof(hid_rpt01),			   	// Size of the report descriptor (sizeof(hid_rpt01))
				}, {
				// Endpoint Descriptor
						sizeof(USB_EP_DSC),				// sizeof(USB_EP_DSC)
						DSC_EP,		                    // Endpoint Descriptor
						_EP02_IN,				        // Endpoint Address
						_INT,						    // Attributes
						0x40,						    // size
						0x01,							// Interval
				}, {
				// Endpoint Descriptor
						sizeof(USB_EP_DSC),				// sizeof(USB_EP_DSC)
						DSC_EP,		                    // Endpoint Descriptor
						_EP02_OUT,				        // EndpointAddress
						_INT,						    // Attributes
						0x40,						    // size
						0x01							// Interval
				} };

__code unsigned char string_descriptor_0[] = { // available languages  descriptor
		0x04, STRING_DESCRIPTOR, //
				0x09, 0x04, //English (United States)
		};

__code unsigned char string_descriptor_1[] = { // Manufacturer
		sizeof(string_descriptor_1), STRING_DESCRIPTOR, // bLength, bDscType
				'S', 0x00, //
				'p', 0x00, //
				'a', 0x00, //
				'r', 0x00, //
				'e', 0x00, //
				'T', 0x00, //
				'i', 0x00, //
				'm', 0x00, //
				'e', 0x00, //
				'L', 0x00, //
				'a', 0x00, //
				'b', 0x00, //
				's', 0x00, //
		};

__code unsigned char string_descriptor_2[] = { // Product
		sizeof(string_descriptor_2), STRING_DESCRIPTOR, // bLength, bDscType
				'T', 0x00, //
				'O', 0x00, //
				'A', 0x00, //
				'D', 0x00, //
				'4', 0x00, //
		};

// Class specific descriptor - HID
__code unsigned char hid_rpt01[] = { // 28 bytes
		0x06, 0x00, 0xFF,       // Usage Page = 0xFF00 (Vendor Defined Page 1)
				0x09, 0x01,             // Usage (Vendor Usage 1)
				0xA1, 0x01,             // Collection (Application)
				0x19, 0x01,             //      Usage Minimum
				0x29, 0x40,             //      Usage Maximum
				0x15, 0x00,             //      Logical Minimum
				0x26, 0xFF, 0x00,     	//      Logical Maximum
				0x75, 0x08,             //      Report Size: 8-bit field size
				0x95, 0x40,             //      Report Count: Make sixty-four 8-bit fields
				0x81, 0x02,             //      Input (Data, Array, Abs)
				0x19, 0x01,             //      Usage Minimum
				0x29, 0x40,             //      Usage Maximum
				0x91, 0x02,             //      Output (Data, Array, Abs)
				0xC0                   // End Collection
		};

//Raw Descriptor (hex)    0000: 06 00 FF 09 01 A1 01 19  01 29 40 15 00 26 FF 00
//Raw Descriptor (hex)    0010: 75 08 95 40 81 02 19 01  29 40 91 02 C0

// Global variables
static __code char const_values_0x00_0x01[] = { 0, 1 };
static __code char device_status[] = { 0, 0 };
static __code char interface_status[] = { 0, 1 };
static __code char endpoint_status_stall[] = { 1, 0 };
static __code char endpoint_status_no_stall[] = { 0, 0 };

volatile unsigned char hid_rx_buffer[64];
volatile unsigned char hid_tx_buffer[64];

unsigned char device_state;

static unsigned char device_address;
static unsigned char current_configuration; // 0 or 1
static unsigned char idx; // loop counter for data transfers loops
static unsigned char control_stage; // Holds the current stage in a control transfer
static unsigned char request_handled; // Set to 1 if request was understood and processed.
static dataPtr data_ptr; // Data to host from RAM
static codePtr code_ptr; // Data to host from FLASH
static dataPtr in_ptr; // Data from the host
static unsigned char dlen; // Number of unsigned chars of data

// See USB spec chapter 5
#define SETUP_STAGE    0
#define DATA_OUT_STAGE 1
#define DATA_IN_STAGE  2
#define STATUS_STAGE   3

// Put endpoint 0 buffers into dual port RAM
// Put USB I/O buffers into dual port RAM.
#pragma udata usbram5 setup_packet control_transfer_buffer hid_rx_buffer hid_tx_buffer

static volatile setup_packet_struct setup_packet;
static volatile unsigned char control_transfer_buffer[E0SZ];

// Notes:
// File:

//if(SetupPkt.Recipient != USB_SETUP_RECIPIENT_INTERFACE_BITFIELD) return;
//if(SetupPkt.bIntfID != HID_INTF_ID) return;

/*
 Files:

 /Users/nyholku/microchip/mla/v2013_12_20/apps/usb/device/bootloaders/firmware/pic18_non_j/src/usb_config.h
 /Users/nyholku/microchip/mla/v2013_12_20/framework/usb/usb_ch9.h
 /Users/nyholku/microchip/mla/v2013_12_20/framework/usb/src/usb_device_hid.c
 /Users/nyholku/microchip/mla/v2013_12_20/apps/usb/device/bootloaders/firmware/pic18_non_j/src/usb_device.h

 struct __attribute__ ((packed))
 {
 uint8_t bmRequestType; //from table 9-2 of USB2.0 spec
 uint8_t bRequest; //from table 9-2 of USB2.0 spec
 uint16_t wValue; //from table 9-2 of USB2.0 spec
 uint16_t wIndex; //from table 9-2 of USB2.0 spec
 uint16_t wLength; //from table 9-2 of USB2.0 spec
 };
 struct __attribute__ ((packed))
 {
 unsigned Recipient:5;   //Device,Interface,Endpoint,Other
 unsigned RequestType:2; //Standard,Class,Vendor,Reserved
 unsigned DataDir:1;     //Host-to-device,Device-to-host
 unsigned :8;
 uint8_t bFeature;          //DEVICE_REMOTE_WAKEUP,ENDPOINT_HALT
 struct __attribute__ ((packed))
 {
 unsigned :8;
 unsigned :8;
 uint8_t bAltID;            //Alternate Setting Value 0-255
 uint8_t bAltID_H;          //Must equal zero
 uint8_t bIntfID;           //Interface Number Value 0-255
 uint8_t bIntfID_H;         //Must equal zero

 #define USB_SETUP_RECIPIENT_INTERFACE           0x01    // Device Request bmRequestType recipient - interface
 #define USB_SETUP_RECIPIENT_INTERFACE_BITFIELD  (USB_SETUP_RECIPIENT_INTERFACE)     // Device Request bmRequestType recipient - interface
 #define HID_INTF_ID             0x00
 #define HID_UEP                 UEP1
 #define HID_BD_OUT              ep1Bo
 #define HID_INT_OUT_EP_SIZE     64
 #define HID_BD_IN               ep1Bi
 #define HID_INT_IN_EP_SIZE      64
 #define HID_NUM_OF_DSC          1       //Just the Report descriptor (no physical descriptor present)
 #define HID_RPT01_SIZE          29      //Make sure this matches the size of your HID report descriptor

 // Section: Standard Device Requests

 #define USB_REQUEST_GET_STATUS                  0       // Standard Device Request - GET STATUS
 #define USB_REQUEST_CLEAR_FEATURE               1       // Standard Device Request - CLEAR FEATURE
 #define USB_REQUEST_SET_FEATURE                 3       // Standard Device Request - SET FEATURE
 #define USB_REQUEST_SET_ADDRESS                 5       // Standard Device Request - SET ADDRESS
 #define USB_REQUEST_GET_DESCRIPTOR              6       // Standard Device Request - GET DESCRIPTOR
 #define USB_REQUEST_SET_DESCRIPTOR              7       // Standard Device Request - SET DESCRIPTOR
 #define USB_REQUEST_GET_CONFIGURATION           8       // Standard Device Request - GET CONFIGURATION
 #define USB_REQUEST_SET_CONFIGURATION           9       // Standard Device Request - SET CONFIGURATION
 #define USB_REQUEST_GET_INTERFACE               10      // Standard Device Request - GET INTERFACE
 #define USB_REQUEST_SET_INTERFACE               11      // Standard Device Request - SET INTERFACE
 #define USB_REQUEST_SYNCH_FRAME                 12      // Standard Device Request - SYNCH FRAME

 void USBCheckHIDRequest(void)
 {
 {
 if ((setup_packet.bmrequesttype &0x1F)!=0x01) ...
 if(SetupPkt.Recipient != USB_SETUP_RECIPIENT_INTERFACE_BITFIELD) return;
 if (setup_packet.windex0 != 0x00) ...
 if(SetupPkt.bIntfID != HID_INTF_ID) return;


 // There are two standard requests that hid.c may support.
 // 1. GET_DSC(DSC_HID,DSC_RPT,DSC_PHY);
 // 2. SET_DSC(DSC_HID,DSC_RPT,DSC_PHY);

 if(setup_packet.breques == ....
 if(SetupPkt.bRequest == USB_REQUEST_GET_DESCRIPTOR)
 {
 switch(SetupPkt.bDescriptorType)
 {
 case DSC_HID: //HID Descriptor
 if(USBActiveConfiguration == 1)
 {
 USBEP0SendROMPtr(
 (const uint8_t*)&configDescriptor1 + 18,		//18 is a magic number.  It is the offset from start of the configuration descriptor to the start of the HID descriptor.
 sizeof(USB_HID_DSC)+3,
 USB_EP0_INCLUDE_ZERO);
 }
 break;
 case DSC_RPT:  //Report Descriptor
 //if(USBActiveConfiguration == 1)
 {
 USBEP0SendROMPtr(
 (const uint8_t*)&hid_rpt01,
 HID_RPT01_SIZE,     //See usbcfg.h
 USB_EP0_INCLUDE_ZERO);
 }
 break;
 case DSC_PHY:  //Physical Descriptor
 //Note: The below placeholder code is commented out.  HID Physical Descriptors are optional and are not used
 //in many types of HID applications.  If an application does not have a physical descriptor,
 //then the device should return STALL in response to this request (stack will do this automatically
 //if no-one claims ownership of the control transfer).
 //If an application does implement a physical descriptor, then make sure to declare
 //hid_phy01 (rom structure containing the descriptor data), and hid_phy01 (the size of the descriptors in uint8_ts),
 //and then uncomment the below code.
 //if(USBActiveConfiguration == 1)
 //{
 //    USBEP0SendROMPtr((const uint8_t*)&hid_phy01, sizeof(hid_phy01), USB_EP0_INCLUDE_ZERO);
 //}
 break;
 }//end switch(SetupPkt.bDescriptorType)
 }//end if(SetupPkt.bRequest == GET_DSC)

 if(SetupPkt.RequestType != USB_SETUP_TYPE_CLASS_BITFIELD)
 {
 return;
 }

 switch(SetupPkt.bRequest)
 {
 case GET_REPORT:
 #if defined USER_GET_REPORT_HANDLER
 USER_GET_REPORT_HANDLER();
 #endif
 break;
 case SET_REPORT:
 #if defined USER_SET_REPORT_HANDLER
 USER_SET_REPORT_HANDLER();
 #endif
 break;
 case GET_IDLE:
 USBEP0SendRAMPtr(
 (uint8_t*)&idle_rate,
 1,
 USB_EP0_INCLUDE_ZERO);
 break;
 case SET_IDLE:
 USBEP0Transmit(USB_EP0_NO_DATA);
 idle_rate = ((USB_SETUP_SET_IDLE_RATE*)&SetupPkt)->duration;
 USB_DEVICE_HID_IDLE_RATE_CALLBACK(((USB_SETUP_SET_IDLE_RATE*)&SetupPkt)->reportId, idle_rate);
 break;
 case GET_PROTOCOL:
 USBEP0SendRAMPtr(
 (uint8_t*)&active_protocol,
 1,
 USB_EP0_NO_OPTIONS);
 break;
 case SET_PROTOCOL:
 USBEP0Transmit(USB_EP0_NO_DATA);
 active_protocol = ((USB_SETUP_SET_PROTOCOL*)&SetupPkt)->protocol;
 break;
 }//end switch(SetupPkt.bRequest)

 } }//end USBCheckHIDRequest

 */

static void get_descriptor(void) {
	if (setup_packet.bmrequesttype == 0x80) {
		unsigned char descriptorType = setup_packet.wvalue1;
		unsigned char descriptorIndex = setup_packet.wvalue0;

		if (descriptorType == DEVICE_DESCRIPTOR) {
			request_handled = 1;
			code_ptr = (codePtr) &device_descriptor;
			dlen = *code_ptr;
		} else if (descriptorType == CONFIGURATION_DESCRIPTOR) {
			request_handled = 1;
			code_ptr = (codePtr) &config_descriptor;
			dlen = *(code_ptr + 2);
		} else if (descriptorType == STRING_DESCRIPTOR) {
			request_handled = 1;
			if (descriptorIndex == 0) {
				code_ptr = (codePtr) &string_descriptor_0;
			} else if (descriptorIndex == 1) {
				code_ptr = (codePtr) &string_descriptor_1;
			} else if (descriptorIndex == 2) {
				code_ptr = (codePtr) &string_descriptor_2;
			} else {
				request_handled = 0;
			}
			dlen = *code_ptr;
		}
	}
}

static void get_status(void) {
	unsigned char recipient = setup_packet.bmrequesttype & 0x1F;

	// See where the request goes
	if (recipient == 0x00) {
		// Device
		request_handled = 1;
		code_ptr = (codePtr) &device_status; // hard coded device status

	} else if (recipient == 0x01) {
		// Interface
		code_ptr = (codePtr) &interface_status;
		request_handled = 1;
	} else if (recipient == 0x02) {
		// Endpoint
		unsigned char endpointNum = setup_packet.windex0 & 0x0F;
		unsigned char endpointDir = setup_packet.windex0 & 0x80;
		request_handled = 1;
		in_ptr = (dataPtr) &ep0_o + (endpointNum * 8);
		if (endpointDir)
			in_ptr += 4;
		if (*in_ptr & BSTALL)
			code_ptr = (codePtr) &endpoint_status_stall;
		else
			code_ptr = (codePtr) &endpoint_status_no_stall;
	}
	if (request_handled) {
		dlen = 2;
	}
}

static void set_feature(void) {
	unsigned char recipient = setup_packet.bmrequesttype & 0x1F;
	unsigned char feature = setup_packet.wvalue0;

	if (recipient == 0x02) {
		// Endpoint
		unsigned char endpointNum = setup_packet.windex0 & 0x0F;
		unsigned char endpointDir = setup_packet.windex0 & 0x80;
		if ((feature == ENDPOINT_HALT) && (endpointNum != 0)) {
			char temp; // FIXME can't we find a global variable

			// Halt endpoint (as long as it isn't endpoint 0)
			request_handled = 1;
			// Endpoint descriptors are 8 unsigned chars long, with each in and out taking 4 unsigned chars
			// within the endpoint. (See PIC datasheet.)
			in_ptr = (dataPtr) &ep0_o + (endpointNum * 8);
			if (endpointDir)
				in_ptr += 4;
			// FIXME figure out what this is
			if (setup_packet.brequest == SET_FEATURE)
				temp = 0x84;
			else {
				if (endpointDir == 1)
					temp = 0x00;
				else
					temp = 0x88;
			}
			*in_ptr = temp;

		}
	}
}

// Prepare EP0 to transfer data from ROM
void in_data_stage(void) {

	unsigned char bufferSize;
	// Determine how many unsigned chars are going to the host
	if (dlen < E0SZ)
		bufferSize = dlen;
	else
		bufferSize = E0SZ;

	// Load the high two bits of the unsigned char dlen into BC8:BC9
	ep0_i.STAT &= ~(BC8 | BC9); // Clear BC8 and BC9
	//We only ever see <=64 byte data transfers so following is commented out
	//ep0_i.STAT |= (unsigned char) ((bufferSize & 0x0300) >> 8);
	//ep0_i.CNT = (unsigned char) (bufferSize & 0xFF);
	ep0_i.CNT = bufferSize;
	ep0_i.ADDR = (int) &control_transfer_buffer;
	// Update the number of unsigned chars that still need to be sent.  Getting
	// all the data back to the host can take multiple transactions, so
	// we need to track how far along we are.
	dlen = dlen - bufferSize;
	// Move data to the USB output buffer from wherever it sits now.
	in_ptr = (dataPtr) &control_transfer_buffer;

	//	for (idx = 0; idx < bufferSize; idx++)
	for (idx = bufferSize; idx--;)
		*in_ptr++ = *code_ptr++;
}

void prepare_for_setup_stage(void) {
	control_stage = SETUP_STAGE;
	ep0_o.CNT = E0SZ;
	ep0_o.ADDR = (int) &setup_packet;
	ep0_o.STAT = UOWN | DTSEN;
	ep0_i.STAT = 0x00;
	UCONbits.PKTDIS = 0;
}

// Note: Microchip says to turn off the UOWN bit on the IN direction as
// soon as possible after detecting that a SETUP has been received.
char cntr = 0;
void process_control_transfer(void) {
	if (USTAT == USTAT_OUT) {
		// FIXME PID = wasted memory, as is the >> op here
		unsigned char PID = (ep0_o.STAT & 0x3C) >> 2; // Pull PID from middle of BD0STAT
		if (PID == 0x0D) {
			unsigned char request = setup_packet.brequest;
			// Setup stage
			ep0_i.STAT &= ~UOWN;
			ep0_o.STAT &= ~UOWN;

			// Initialize the transfer process
			control_stage = SETUP_STAGE;
			request_handled = 0; // Default is that request hasn't been handled

			dlen = 0; // Default to no bytes transferred
			// See if this is a standard (as definded in USB chapter 9) request

			if ((setup_packet.bmrequesttype == 0x81) && (setup_packet.brequest == GET_DESCRIPTOR)) {
				switch (setup_packet.wvalue1) {
					case DSC_HID:
						code_ptr = (codePtr) &config_descriptor + 18;
						dlen = *code_ptr;
						request_handled = 1;
						break;
					case DSC_RPT:
						code_ptr = (codePtr) &hid_rpt01;
						dlen = sizeof(hid_rpt01);
						request_handled = 1;
						LED_PIN = 1; // DEBUG
						break;
					default:
						break;
				}

//							LED_PIN=1;

			}
			//if (cntr==0)
			//cntr++;

			if (setup_packet.bmrequesttype == 0xA1 /*&& setup_packet.brequest == HID_GET_REPORT*/) {
				//code_ptr = (codePtr) &config_descriptor;
				//dlen = 64;
				//request_handled = 1;
				LED_PIN = 1; // DEBUG

			}

			if (request == SET_ADDRESS) {
				// Set the address of the device.  All future requests
				// will come to that address.  Can't actually set UADDR
				// to the new address yet because the rest of the SET_ADDRESS
				// transaction uses address 0.

				request_handled = 1;
				device_state = ADDRESS;
				device_address = setup_packet.wvalue0;
			} else if (request == GET_DESCRIPTOR) {
				get_descriptor();
			} else if (request == SET_CONFIGURATION) {

				request_handled = 1;
				current_configuration = setup_packet.wvalue0;
				// TBD: ensure the new configuration value is one that
				// exists in the descriptor.
				if (current_configuration == 0) {
					// If configuration value is zero, device is put in
					// address state (USB 2.0 - 9.4.7)
					device_state = ADDRESS;
				} else {
					// Set the configuration.
					device_state = CONFIGURED;

					// Initialize the endpoints for all interfaces
					{ // Turn on both in and out for this endpoint

						UEP2 = 0x1E;

						ep2_o.CNT = sizeof(hid_rx_buffer);
						ep2_o.ADDR = (int) &hid_rx_buffer;
						ep2_o.STAT = UOWN | DTSEN; //set up to receive stuff as soon as we get something

						ep2_i.ADDR = (int) &hid_tx_buffer;
						ep2_i.STAT = DTS;
					}
				}
			} else if (request == GET_CONFIGURATION) { // Never seen in Windows
				request_handled = 1;
				code_ptr = (codePtr) &const_values_0x00_0x01[current_configuration];
				dlen = 1;
			} else if (request == GET_STATUS) {  // Never seen in Windows
				get_status();
			} else if ((request == CLEAR_FEATURE) || (request == SET_FEATURE)) {  // Never seen in Windows
				set_feature();
			} else if (request == GET_INTERFACE) { // Never seen in Windows
				// No support for alternate interfaces.  Send
				// zero back to the host.

				request_handled = 1;
				code_ptr = (codePtr) (&const_values_0x00_0x01);
				dlen = 1;
			} else if ((request == SET_INTERFACE)) {

				// No support for alternate interfaces - just ignore.

				request_handled = 1;
			} else {
//				 if(SetupPkt.Recipient != USB_SETUP_RECIPIENT_INTERFACE_BITFIELD) return;
//				 if (setup_packet.windex0 != 0x00) ...
//				 if(SetupPkt.bIntfID != HID_INTF_ID) return;

				// There are two standard requests that hid.c may support.
				// 1. GET_DSC(DSC_HID,DSC_RPT,DSC_PHY);
				// 2. SET_DSC(DSC_HID,DSC_RPT,DSC_PHY);

//				 if(setup_packet.breques == ....
//				 if(SetupPkt.bRequest == USB_REQUEST_GET_DESCRIPTOR)
//				 {
//				 switch(SetupPkt.bDescriptorType)
//				 {
//				 case DSC_HID: //HID Descriptor
//				 if(USBActiveConfiguration == 1)
//				 {
//				 USBEP0SendROMPtr(
//				 (const uint8_t*)&configDescriptor1 + 18,		//18 is a magic number.  It is the offset from start of the configuration descriptor to the start of the HID descriptor.
//				 sizeof(USB_HID_DSC)+3,
//				 USB_EP0_INCLUDE_ZERO);
//				 }
//				 break;
//				 case DSC_RPT:  //Report Descriptor
				//if(USBActiveConfiguration == 1)

			}

			if (!request_handled) {
				// If this service wasn't handled then stall endpoint 0
				ep0_o.CNT = E0SZ;
				ep0_o.ADDR = (int) &setup_packet;
				ep0_o.STAT = UOWN | BSTALL;
				ep0_i.STAT = UOWN | BSTALL;
			} else {
				if (setup_packet.bmrequesttype & 0x80) {
					// Device-to-host
					if (setup_packet.wlength < dlen)					//9.4.3, p.253
						dlen = setup_packet.wlength;

					in_data_stage();
					control_stage = DATA_IN_STAGE;
					// Reset the out buffer descriptor for endpoint 0
					ep0_o.CNT = E0SZ;
					ep0_o.ADDR = (int) &setup_packet;
					ep0_o.STAT = UOWN;
					// Set the in buffer descriptor on endpoint 0 to send data
					// NOT NEEDED ep0_i.ADDR = (int) &control_transfer_buffer;
					// Give to SIE, DATA1 packet, enable data toggle checks
					ep0_i.STAT = UOWN | DTS | DTSEN;
				} else {

					// Host-to-device
					control_stage = DATA_OUT_STAGE;
					// Clear the input buffer descriptor
					ep0_i.CNT = 0;
					ep0_i.STAT = UOWN | DTS | DTSEN;
					// Set the out buffer descriptor on endpoint 0 to receive data
					ep0_o.CNT = E0SZ;
					ep0_o.ADDR = (int) &control_transfer_buffer;
					// Give to SIE, DATA1 packet, enable data toggle checks
					ep0_o.STAT = UOWN | DTS | DTSEN;
				}
			}
			// Enable SIE token and packet processing
			UCONbits.PKTDIS = 0;

		} else if (control_stage == DATA_OUT_STAGE) {
			// Complete the data stage so that all information has
			// passed from host to device before servicing it.

			{

				unsigned char bufferSize;
				// We only ever see <=64 data transfer so following is unnecessary and commented out
				//bufferSize = ((0x03 & ep0_o.STAT) << 8) | ep0_o.CNT;
				bufferSize = ep0_o.CNT;

				// Accumulate total number of unsigned chars read
				dlen = dlen + bufferSize;
				data_ptr = (dataPtr) &control_transfer_buffer;
				for (idx = bufferSize; idx--;)
					*in_ptr++ = *data_ptr++;

			}

			// Turn control over to the SIE and toggle the data bit
			if (ep0_o.STAT & DTS)
				ep0_o.STAT = UOWN | DTSEN;
			else
				ep0_o.STAT = UOWN | DTS | DTSEN;
		} else {
			// Prepare for the Setup stage of a control transfer
			prepare_for_setup_stage();
		}
	} else if (USTAT == USTAT_IN) {
		// Endpoint 0:in

		//set address
		if ((UADDR == 0) && (device_state == ADDRESS)) {
			// TBD: ensure that the new address matches the value of
			// "device_address" (which came in through a SET_ADDRESS).
			UADDR = setup_packet.wvalue0;
			if (UADDR == 0) {
				// If we get a reset after a SET_ADDRESS, then we need
				// to drop back to the Default state.
				device_state = DEFAULT;
			}
		}

		if (control_stage == DATA_IN_STAGE) {
			// Start (or continue) transmitting data
			in_data_stage();
			// Turn control over to the SIE and toggle the data bit
			if (ep0_i.STAT & DTS)
				ep0_i.STAT = UOWN | DTSEN;
			else
				ep0_i.STAT = UOWN | DTS | DTSEN;
		} else {
			// Prepare for the Setup stage of a control transfer
			prepare_for_setup_stage();
		}
	}
}

void usbcdc_init() {
	UCFG = 0x14; // Enable pullup resistors; full speed mode

	device_state = DETACHED;
	//	remote_wakeup = 0x00;
	current_configuration = 0x00;

	// attach
	if (UCONbits.USBEN == 0) { //enable usb controller
		UCON = 0;
		UIE = 0;
		UCONbits.USBEN = 1;
		device_state = ATTACHED;

	}

	{ //Wait for bus reset
		UIR = 0;
		UIE = 0;
		UIEbits.URSTIE = 1;
		device_state = POWERED;
	}

	PIE2bits.USBIE = 1;
}
// Main entry point for USB tasks.  Checks interrupts, then checks for transactions.
void usbcdc_handler(void) {
	if ((UCFGbits.UTEYE == 1) || //eye test
			(device_state == DETACHED) || //not connected
			(UCONbits.SUSPND == 1)) //suspended
		return;

	// Process a bus reset
	if (UIRbits.URSTIF && UIEbits.URSTIE) {
		{ // bus_reset

			UEIR = 0x00;
			UIR = 0x00;
			UEIE = 0x9f;
			UIE = 0x2b;
			UADDR = 0x00;

			// Set endpoint 0 as a control pipe
			UEP0 = 0x16;

			// Flush any pending transactions
			while (UIRbits.TRNIF == 1)
				UIRbits.TRNIF = 0;

			// Enable packet processing
			UCONbits.PKTDIS = 0;

			// Prepare for the Setup stage of a control transfer
			prepare_for_setup_stage();

			current_configuration = 0; // Clear active configuration
			device_state = DEFAULT;

		}
		UIRbits.URSTIF = 0;
	}
	//nothing is done to start of frame
	if (UIRbits.SOFIF && UIEbits.SOFIE) {
		UIRbits.SOFIF = 0;
	}

	// stall processing
	if (UIRbits.STALLIF && UIEbits.STALLIE) {
		if (UEP0bits.EPSTALL == 1) {
			// Prepare for the Setup stage of a control transfer
			prepare_for_setup_stage();
			UEP0bits.EPSTALL = 0;
		}
		UIRbits.STALLIF = 0;
	}
	if (UIRbits.UERRIF && UIEbits.UERRIE) {
		// Clear errors
		UIRbits.UERRIF = 0;
	}
	// A transaction has finished.  Try default processing on endpoint 0.
	if (UIRbits.TRNIF && UIEbits.TRNIE) {
		process_control_transfer();
		// Turn off interrupt
		UIRbits.TRNIF = 0;
	}
}

void test_hid() {
	if ((ep2_i.STAT & UOWN) == 0) {
		LED_PIN=!LED_PIN;
		hid_tx_buffer[0]++;
		ep2_i.CNT = 64;
		if (ep2_i.STAT & DTS)
			ep2_i.STAT = UOWN | DTSEN;
		else
			ep2_i.STAT = UOWN | DTS | DTSEN;
	}

	if(! (ep2_o.STAT & UOWN)) {
		char i;
		ep2_o.CNT = 64;
		for (i=0; i<64; ++i)
			hid_tx_buffer[i] = hid_rx_buffer[i];
		if (ep2_o.STAT & DTS)
			ep2_o.STAT = UOWN | DTSEN;
		else
			ep2_o.STAT = UOWN | DTS | DTSEN;

	}


}
// 7.2.2
