/*
 * usb_user_config.c
 *
 */

#include "usb_user_config.h"
#include "usb_hid.h"
#include "usb_cdc.h"

// http://lists.apple.com/archives/usb/2010/May/msg00007.html
__code usb_dev_desc_t device_descriptor = { //
		sizeof(usb_dev_desc_t),    		// bLength
				DSC_DEV,                // bDescriptorType
				0x0200,                 // bcdUSB lsb, bcdUSB msb
				0x00,                   // bDeviceClass
				0x00,                   // bDeviceSubClass
				0x00,                   // bDeviceProtocl
				8,                      // bMaxPacketSize
				USB_VID,				// idVendor
				USB_PID,				// idProduct
				0x0100,                 // bcdDevice
				0x01,                   // iManufacturer
				0x02,                   // iProduct
				0x00,                   // iSerialNumber
				0x01                    // bNumConfigurations
		};

typedef struct { //
	usb_cfg_desc_t cd01; //
	usb_intf_desc_t i01a00; //
	usb_hid_desc_t hid_01a00; //
	usb_ep_desc_t ep01i_i01a00; //
	usb_ep_desc_t ep02i_i01a00; //
} config_struct_t;

__code config_struct_t config_descriptor = { //
		{
		// Configuration Descriptor
				0x09,// Size of this descriptor in bytes
				DSC_CFG,	                    // configurator descriptor type
				sizeof(config_struct_t),		// Total length of data for this configuration
				1,								// bNumInterfaces
				1,								// bConfigurationValue
				0,								// iConfiguration
				0x80 | _SELF,					// bmAttributes
				1,		    					// bMaxPower
				}, {
				// Interface Descriptor
						sizeof(usb_intf_desc_t),			// Size of this descriptor in bytes
						DSC_INTF,		                // INTERFACE descriptor type
						0,								// Interface Number
						0,								// Alternate Setting Number
						2,								// Number of endpoints in this interfacee
						HID_INTF,						// Class code
						0,								// Subclass code
						0,								// Protocol code
						0,								// Interface string index
				}, {
				// HID Class-Specific Descriptor
						sizeof(usb_hid_desc_t),			// Size of this descriptor in bytes
						DSC_HID,						// HID descriptor type
						0x0111,							// HID Spec Release Number in BCD format (1.11)
						0x00,							// Country Code (0x00 for Not supported)
						1,					            // Number of class descriptors, see usbcfg.h
						DSC_RPT,						// Report descriptor type
						sizeof(hid_rpt01),			   	// Size of the report descriptor
				}, {
				// Endpoint Descriptor
						sizeof(usb_ep_desc_t),				// Size of this descriptor in bytes
						DSC_EP,		                    // Endpoint Descriptor
						_EP02_IN,				        // Endpoint Address
						_INT,						    // Attributes
						0x40,						    // size
						0x01,							// Interval
				}, {
				// Endpoint Descriptor
						sizeof(usb_ep_desc_t),				// Size of this descriptor in bytes
						DSC_EP,		                    // Endpoint Descriptor
						_EP02_OUT,				        // EndpointAddress
						_INT,						    // Attributes
						0x40,						    // size
						0x01							// Interval
				}






};

// HID report descriptor
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

typedef struct {
	usb_cfg_desc_t cd01;
	usb_intf_desc_t i01a00;
	usb_cdc_header_fn_desc_t cdc_header_fn_i01a00;
	usb_cdc_call_mgt_fn_desc_t cdc_call_mgt_fn_i01a00;
	usb_cdc_acm_fn_desc_t cdc_acm_fn_i01a00;
	usb_cdc_union_fn_desc_t cdc_union_fn_i01a00;
	usb_ep_desc_t ep02i_i01a00;
	usb_intf_desc_t i02a00;
	usb_ep_desc_t ep03o_i02a00;
	usb_ep_desc_t ep03i_i02a00;
} config_struct2;

__code config_struct2 config_descriptor2 = { {
// Configuration descriptor
		sizeof(usb_cfg_desc_t),  //
		DSC_CFG, // bLength, bDescriptorType (Configuration)
		sizeof(config_struct2), // wTotalLength
		0x02,  //bNumInterfaces
		0x01, // bConfigurationValue
		0x00, 0x80 | (USBCDC_SELF_POWERED << 6), //_DEFAULT, // iConfiguration, bmAttributes ()
		USBCDC_MAXPOWER / 2, // bMaxPower
		},
		/* Interface Descriptor */
		{ sizeof(usb_intf_desc_t), // Size of this descriptor in unsigned chars
				DSC_INTF, // INTERFACE descriptor type
				0, // Interface Number
				0, // Alternate Setting Number
				1, // Number of endpoints in this intf
				COMM_INTF, // Class __code
				ABSTRACT_CONTROL_MODEL, // Subclass __code
				V25TER, // Protocol __code
				0 // Interface string index
		},
		/* CDC Class-Specific Descriptors */
		{ sizeof(usb_cdc_header_fn_desc_t),  //
				CS_INTERFACE,  //
				DSC_FN_HEADER,  //
				0x0110 // //
		},//
		{ sizeof(usb_cdc_call_mgt_fn_desc_t), //
				CS_INTERFACE, //
				DSC_FN_CALL_MGT,  //
				0x01,  //
				CDC_DATA_INTF_ID //
		},//
		{ sizeof(usb_cdc_acm_fn_desc_t),  //
				CS_INTERFACE,  //
				DSC_FN_ACM,  //
				0x02 //
		},//
		{ sizeof(usb_cdc_union_fn_desc_t), //
				CS_INTERFACE, //
				DSC_FN_UNION, //
				CDC_COMM_INTF_ID, //
				CDC_DATA_INTF_ID //
		},
		/* Endpoint Descriptor */
		{ //notification endpoint
		sizeof(usb_ep_desc_t), //
				DSC_EP, //
				_EP01_IN, //
				_INT, //
				CDC_INT_EP_SIZE,  //
				0x0A //
		},//
		/* Interface Descriptor */
		{ sizeof(usb_intf_desc_t), // Size of this descriptor in unsigned chars
				DSC_INTF, // INTERFACE descriptor type
				1, // Interface Number
				0, // Alternate Setting Number
				2, // Number of endpoints in this intf
				DATA_INTF, // Class __code
				0, // Subclass __code
				NO_PROTOCOL, // Protocol __code
				2, // Interface string index
		},
		/* Endpoint Descriptors */
		{ sizeof(usb_ep_desc_t), //
				DSC_EP,  //
				_EP02_OUT, //
				_BULK, USBCDC_BUFFER_LEN,  //
				0x00 //
		},//
		{ sizeof(usb_ep_desc_t),  //
				DSC_EP,  //
				_EP02_OUT, //
				_BULK,  //
				USBCDC_BUFFER_LEN,  //
				0x00 }, //
		};


typedef struct {
	usb_cfg_desc_t cd01;

	usb_intf_desc_t i02a00;
	usb_cdc_header_fn_desc_t cdc_header_fn_i02a00;
	usb_cdc_call_mgt_fn_desc_t cdc_call_mgt_fn_i02a00;
	usb_cdc_acm_fn_desc_t cdc_acm_fn_i02a00;
	usb_cdc_union_fn_desc_t cdc_union_fn_i02a00;
	usb_ep_desc_t ep02i_i02a00;
	usb_intf_desc_t i03a00;
	usb_ep_desc_t ep03o_i03a00;
	usb_ep_desc_t ep03i_i03a00;

	usb_intf_desc_t i01a00; //
	usb_hid_desc_t hid_01a00; //
	usb_ep_desc_t ep01i_i01a00; //
	usb_ep_desc_t ep02i_i01a00; //
} config_struct3;

__code config_struct3 config_descriptor3 = { {
// Configuration descriptor
		sizeof(usb_cfg_desc_t),  //
		DSC_CFG, // bLength, bDescriptorType (Configuration)
		sizeof(config_struct3), // wTotalLength
		0x03,  //bNumInterfaces
		0x01, // bConfigurationValue
		0x00, 0x80 | (USBCDC_SELF_POWERED << 6), //_DEFAULT, // iConfiguration, bmAttributes ()
		USBCDC_MAXPOWER / 2, // bMaxPower
		},




		/* Interface Descriptor */
		{ sizeof(usb_intf_desc_t), // Size of this descriptor in unsigned chars
				DSC_INTF, // INTERFACE descriptor type
				0, // Interface Number
				0, // Alternate Setting Number
				1, // Number of endpoints in this intf
				COMM_INTF, // Class __code
				ABSTRACT_CONTROL_MODEL, // Subclass __code
				V25TER, // Protocol __code
				0 // Interface string index
		},
		/* CDC Class-Specific Descriptors */
		{ sizeof(usb_cdc_header_fn_desc_t),  //
				CS_INTERFACE,  //
				DSC_FN_HEADER,  //
				0x0110 // //
		},//
		{ sizeof(usb_cdc_call_mgt_fn_desc_t), //
				CS_INTERFACE, //
				DSC_FN_CALL_MGT,  //
				0x01,  //
				CDC_DATA_INTF_ID //
		},//
		{ sizeof(usb_cdc_acm_fn_desc_t),  //
				CS_INTERFACE,  //
				DSC_FN_ACM,  //
				0x02 //
		},//
		{ sizeof(usb_cdc_union_fn_desc_t), //
				CS_INTERFACE, //
				DSC_FN_UNION, //
				CDC_COMM_INTF_ID, //
				CDC_DATA_INTF_ID //
		},
		/* Endpoint Descriptor */
		{ //notification endpoint
		sizeof(usb_ep_desc_t), //
				DSC_EP, //
				_EP01_IN, //
				_INT, //
				CDC_INT_EP_SIZE,  //
				0x0A //
		},//
		/* Interface Descriptor */
		{ sizeof(usb_intf_desc_t), // Size of this descriptor in unsigned chars
				DSC_INTF, // INTERFACE descriptor type
				1, // Interface Number
				0, // Alternate Setting Number
				2, // Number of endpoints in this intf
				DATA_INTF, // Class __code
				0, // Subclass __code
				NO_PROTOCOL, // Protocol __code
				2, // Interface string index
		},
		/* Endpoint Descriptors */
		{ sizeof(usb_ep_desc_t), //
				DSC_EP,  //
				_EP03_IN, //
				_BULK, //
				USBCDC_BUFFER_LEN,  //
				0x00 //
		},//
		{ sizeof(usb_ep_desc_t),  //
				DSC_EP,  //
				_EP03_OUT, //
				_BULK,  //
				USBCDC_BUFFER_LEN,  //
				0x00 }, //



				 {
								// Interface Descriptor
										sizeof(usb_intf_desc_t),			// Size of this descriptor in bytes
										DSC_INTF,		                // INTERFACE descriptor type
										2,								// Interface Number
										0,								// Alternate Setting Number
										2,								// Number of endpoints in this interfacee
										HID_INTF,						// Class code
										0,								// Subclass code
										0,								// Protocol code
										0,								// Interface string index
								}, {
								// HID Class-Specific Descriptor
										sizeof(usb_hid_desc_t),			// Size of this descriptor in bytes
										DSC_HID,						// HID descriptor type
										0x0111,							// HID Spec Release Number in BCD format (1.11)
										0x00,							// Country Code (0x00 for Not supported)
										1,					            // Number of class descriptors, see usbcfg.h
										DSC_RPT,						// Report descriptor type
										sizeof(hid_rpt01),			   	// Size of the report descriptor
								}, {
								// Endpoint Descriptor
										sizeof(usb_ep_desc_t),				// Size of this descriptor in bytes
										DSC_EP,		                    // Endpoint Descriptor
										_EP02_IN,				        // Endpoint Address
										_INT,						    // Attributes
										0x40,						    // size
										0x01,							// Interval
								}, {
								// Endpoint Descriptor
										sizeof(usb_ep_desc_t),				// Size of this descriptor in bytes
										DSC_EP,		                    // Endpoint Descriptor
										_EP02_OUT,				        // EndpointAddress
										_INT,						    // Attributes
										0x40,						    // size
										0x01							// Interval
								},




		};



codePtr usb_user_get_device_descriptor() {
	return (codePtr) &device_descriptor;

}


codePtr usb_user_get_configuration_descriptor() {
	return (codePtr) &config_descriptor3;
}
codePtr usb_user_get_hid_descriptor() {
	return (codePtr) &config_descriptor + 18;
}

codePtr usb_user_get_hid_report_descriptor() {
	return (codePtr) &hid_rpt01;
}

uint8_t usb_user_get_hid_report_descriptor_length() {
	return sizeof(hid_rpt01);
}
