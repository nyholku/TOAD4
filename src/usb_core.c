/*
 * File: usb_core.c
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
#include "usb_core.h"
#include "pic18f4550.h"
#include "usb_pic_defs.h"

#pragma udata usbram5 hid_rx_buffer hid_tx_buffer
volatile uint8_t hid_rx_buffer[64];
volatile uint8_t hid_tx_buffer[64];

#pragma udata usbram5 setup_packet control_transfer_buffer cdc_rx_buffer cdc_tx_buffer cdcint_buffer
static unsigned char tx_len = 0;
static unsigned char rx_idx = 0;

//static volatile unsigned char cdcint_buffer[USBCDC_BUFFER_LEN];
volatile unsigned char cdc_rx_buffer[16];
volatile unsigned char cdc_tx_buffer[16];
static volatile unsigned char cdcint_buffer[8];

// DESIGN NOTES:

// differences between CDC / HID
// -descriptors
// -EP initialization
// -buffers
// -read/write routines
// -response to class specific stuff

// desirable configuration stuff
// -descriptors (at least VID/PID and strings)
// -end points (perhaps)
// -buffer sizes (maybe not for HID?)
// -HID and/or CDC support

// question: is it really feasible to separate hid/cdc stuff when we are limited for space?
// ...further is it good to let the user config the descriptors as that is quite complex?
// ...or how to provide reasonable defaults and config

#define E0SZ 8

// See USB spec chapter 5
#define SETUP_STAGE    0
#define DATA_OUT_STAGE 1
#define DATA_IN_STAGE  2
#define STATUS_STAGE   3

uint8_t device_state;
uint8_t device_address;
uint8_t current_configuration;
static uint8_t idx; // loop counter for data transfers loops
static uint8_t control_stage; // Holds the current stage in a control transfer
static uint8_t request_handled; // Set to 1 if request was understood and processed.
static dataPtr data_ptr; // Data to host from RAM
static codePtr code_ptr; // Data to host from FLASH
static dataPtr in_ptr; // Data from the host
static uint8_t dlen; // Number of unsigned chars of data

// Put endpoint 0 buffers into dual port RAM
// Put USB I/O buffers into dual port RAM.
#pragma udata usbram5 setup_packet control_transfer_buffer

static volatile setup_packet_struct_t setup_packet;
static volatile uint8_t control_transfer_buffer[E0SZ];
static __code uint8_t const_values_0x00_0x01[] = { 0, 1 };
static __code uint8_t device_status[] = { 0, 0 };
static __code uint8_t interface_status[] = { 0, 1 };
static __code uint8_t endpoint_status_stall[] = { 1, 0 };
static __code uint8_t endpoint_status_no_stall[] = { 0, 0 };

__code uint16_t string_descriptor_0[] = { // available languages  descriptor
		sizeof(string_descriptor_0) + (STRING_DESCRIPTOR << 8),   //
		0x0409, //English (United States), note the endianness conversion
		};

__code uint16_t string_descriptor_1[] = { // Manufacturer
		sizeof(string_descriptor_1) + (STRING_DESCRIPTOR << 8),   //
		USB_MANUFACTURER_STRING  //
		};

__code uint16_t string_descriptor_2[] = { // Manufacturer
		sizeof(string_descriptor_2) + (STRING_DESCRIPTOR << 8),   //
		USB_PRODUCT_STRING //
		};

void prepare_for_setup_stage(void) {
	control_stage = SETUP_STAGE;
	ep0_o.CNT = E0SZ;
	ep0_o.ADDR = (int) &setup_packet;
	ep0_o.STAT = UOWN | DTSEN;
	ep0_i.STAT = 0x00;
	UCONbits.PKTDIS = 0;
}

static void get_descriptor(void) {
	if (setup_packet.bmrequesttype == 0x80) {
		unsigned char descriptorType = setup_packet.wvalue1;
		unsigned char descriptorIndex = setup_packet.wvalue0;

		if (descriptorType == DEVICE_DESCRIPTOR) {
			request_handled = 1;
			code_ptr = usb_user_get_device_descriptor();
			dlen = *code_ptr;
		} else if (descriptorType == CONFIGURATION_DESCRIPTOR) {
			request_handled = 1;
			code_ptr = usb_user_get_configuration_descriptor();
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

// Note: Microchip says to turn off the UOWN bit on the IN direction as
// soon as possible after detecting that a SETUP has been received.
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
						code_ptr = usb_user_get_hid_descriptor();
						dlen = *code_ptr;
						request_handled = 1;
						break;
					case DSC_RPT:
						code_ptr = usb_user_get_hid_report_descriptor();
						dlen = usb_user_get_hid_report_descriptor_length();
						request_handled = 1;
						break;
					default:
						break;
				}
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
					  // CDC
						UEP1 = 0x1E;

						ep1_i.ADDR = (int) &cdcint_buffer;
						ep1_i.STAT = DTS;

						UEP3 = 0x1E;

						ep3_o.CNT = sizeof(cdc_rx_buffer);
						ep3_o.ADDR = (int) &cdc_rx_buffer;
						ep3_o.STAT = UOWN | DTSEN; //set up to receive stuff as soon as we get something

						ep3_i.ADDR = (int) &cdc_tx_buffer;
						ep3_i.STAT = DTS;
						cdc_tx_buffer[0] = 'H';
						cdc_tx_buffer[1] = 'e';
						cdc_tx_buffer[2] = 'l';
						cdc_tx_buffer[3] = 'l';
						cdc_tx_buffer[4] = 'o';
						cdc_tx_buffer[5] = '!';
						cdc_tx_buffer[6] = '\r';
						cdc_tx_buffer[7] = '\n';
						// HID
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



void usbcdc_putchar(char c)__wparam
{
	while (usbcdc_wr_busy())
		/* wait */;
	cdc_tx_buffer[tx_len++] = c;
	if (tx_len >= sizeof(cdc_tx_buffer)) {
		usbcdc_flush();
	}
}

char usbcdc_wr_busy() {
	return (ep3_i.STAT & UOWN) != 0;
}

unsigned char usbcdc_rd_ready() {
	if (ep3_o.STAT & UOWN)
		return 0;
	if (rx_idx >= ep3_o.CNT) {
		usbcdc_read();
		return 0;
	}
	return 1;
}

void usbcdc_write(unsigned char len)__wparam
{
	if (len > 0) {
		ep3_i.CNT = len;
		if (ep3_i.STAT & DTS)
			ep3_i.STAT = UOWN | DTSEN;
		else
			ep3_i.STAT = UOWN | DTS | DTSEN;
	}
}

void usbcdc_flush() {
	usbcdc_write(tx_len);
	tx_len = 0;

}

void usbcdc_read() {
	rx_idx = 0;
	ep3_o.CNT = sizeof(cdc_rx_buffer);
	if (ep3_o.STAT & DTS)
		ep3_o.STAT = UOWN | DTSEN;
	else
		ep3_o.STAT = UOWN | DTS | DTSEN;
}

char usbcdc_getchar() {
	char c;
	while (!usbcdc_rd_ready())
		;

	c = cdc_rx_buffer[rx_idx++];
	if (rx_idx >= ep3_o.CNT) {
		usbcdc_read();
	}
	return c;
}

