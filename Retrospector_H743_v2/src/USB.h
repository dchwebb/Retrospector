#pragma once

#include "initialisation.h"
#include <functional>
#include <cstring>

// Enables capturing of debug data for output over STLink UART on dev boards
#define USB_DEBUG false
#if (USB_DEBUG)
#include "uartHandler.h"
extern bool USBDebug;
#define USB_DEBUG_COUNT 400
#endif


// USB Hardware Registers
#define USBx_PCGCCTL	*reinterpret_cast<__IO uint32_t*>(USB2_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)
#define USBx_DEVICE		reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB2_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE)
#define USBx_INEP(i)	reinterpret_cast<USB_OTG_INEndpointTypeDef*>(USB2_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE))
#define USBx_OUTEP(i)	reinterpret_cast<USB_OTG_OUTEndpointTypeDef*>(USB2_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE))
#define USBx_DFIFO(i)	*reinterpret_cast<uint32_t*>(USB2_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))

// USB Transfer status definitions
#define STS_GOUT_NAK					1U
#define STS_DATA_UPDT					2U
#define STS_XFER_COMP					3U
#define STS_SETUP_COMP					4U
#define STS_SETUP_UPDT					6U

// USB Request Recipient types
#define USB_REQ_RECIPIENT_DEVICE		0x00U
#define USB_REQ_RECIPIENT_INTERFACE		0x01U
#define USB_REQ_RECIPIENT_ENDPOINT		0x02U
#define USB_REQ_RECIPIENT_MASK			0x03U

#define EP_ADDR_MASK					0xFU
#define USB_REQ_DIRECTION_MASK			0x80U

// USB Request types
#define USB_REQ_TYPE_STANDARD			0x00U
#define USB_REQ_TYPE_CLASS				0x20U
#define USB_REQ_TYPE_VENDOR				0x40U
#define USB_REQ_TYPE_MASK				0x60U

#define USB_REQ_GET_STATUS				0x00U
#define USB_REQ_CLEAR_FEATURE			0x01U
#define USB_REQ_SET_FEATURE				0x03U
#define USB_REQ_SET_ADDRESS				0x05U
#define USB_REQ_GET_DESCRIPTOR			0x06U
#define USB_REQ_SET_DESCRIPTOR			0x07U
#define USB_REQ_GET_CONFIGURATION		0x08U
#define USB_REQ_SET_CONFIGURATION		0x09U

#define USB_DESC_TYPE_DEVICE			0x01U
#define USB_DESC_TYPE_CONFIGURATION		0x02U
#define USB_DESC_TYPE_STRING			0x03U
#define USB_DESC_TYPE_INTERFACE			0x04U
#define USB_DESC_TYPE_ENDPOINT			0x05U
#define USB_DESC_TYPE_DEVICE_QUALIFIER	0x06U
#define USB_DESC_TYPE_OTHER_SPEED_CFG	0x07U
#define USB_DESC_TYPE_IAD				0x0BU
#define USB_DESC_TYPE_BOS				0x0FU

// EP0 State
#define USBD_EP0_IDLE					0x00U
#define USBD_EP0_SETUP					0x01U
#define USBD_EP0_DATA_IN				0x02U
#define USBD_EP0_DATA_OUT				0x03U
#define USBD_EP0_STATUS_IN				0x04U
#define USBD_EP0_STATUS_OUT				0x05U
#define USBD_EP0_STALL					0x06U

//  Device Status
#define USBD_STATE_DEFAULT				0x01U
#define USBD_STATE_ADDRESSED			0x02U
#define USBD_STATE_CONFIGURED			0x03U
#define USBD_STATE_SUSPENDED			0x04U

// Index of string descriptors
#define USBD_IDX_LANGID_STR				0x00U
#define USBD_IDX_MFC_STR				0x01U
#define USBD_IDX_PRODUCT_STR			0x02U
#define USBD_IDX_SERIAL_STR				0x03U
#define USBD_IDX_CDC_STR				0x04U

#define USBD_VID						1155
#define USBD_LANGID_STRING				1033
#define USBD_MANUFACTURER_STRING		"Mountjoy Modular"
#define USBD_PID_FS						22352
#define USBD_PRODUCT_STRING				"Mountjoy Retrospector"
#define USBD_CDC_STRING					"Mountjoy Retrospector CDC"

#define CLASS_SPECIFIC_DESC_SIZE		50
#define USB_LEN_LANGID_STR_DESC			4
#define USB_CDC_CONFIG_DESC_SIZE		75

#define LOBYTE(x)  (static_cast<uint8_t>(x & 0x00FFU))
#define HIBYTE(x)  (static_cast<uint8_t>(x & 0xFF00U) >> 8)

class USB {
public:
	void USBInterruptHandler();
	void InitUSB();
	void SendData(const uint8_t *data, uint16_t len, uint8_t endpoint);
	void SendString(const char* s);
	void SendString(std::string s);

	std::function<void(uint8_t*,uint32_t)> cdcDataHandler;			// Declare data handler to store incoming CDC data

	enum EndPoint {CDC_In = 0x81, CDC_Out = 0x1, CDC_Cmd = 0x82, };
	enum EndPointType { Control = 0, Isochronous = 1, Bulk = 2, Interrupt = 3 };
	enum class Direction {in, out};
	volatile bool transmitting;

private:
	void USB_ActivateEndpoint(uint8_t endpoint, Direction direction, EndPointType eptype);
	void USB_ReadPacket(uint32_t* dest, uint16_t len);
	void USB_WritePacket(const uint8_t* src, uint8_t endpoint, uint16_t len);
	void USBD_GetDescriptor();
	void USBD_StdDevReq();
	void USB_EPStartXfer(Direction direction, uint8_t endpoint, uint32_t xfer_len);
	void USBD_CtlError();
	bool USB_ReadInterrupts(uint32_t interrupt);
	void IntToUnicode(uint32_t value, uint8_t* pbuf, uint8_t len);
	uint32_t USBD_GetString(const uint8_t* desc, uint8_t* unicode);

	//usbRequest req;
	const uint8_t ep_maxPacket = 0x40;
	uint32_t xfer_buff[64];			// in HAL there is a transfer buffer for each in and out endpoint
	uint32_t xfer_count;
	uint32_t xfer_rem;				// If transfer is larger than maximum packet size store remaining byte count
	const uint8_t* outBuff;			// FIXME - out misleading as this relates to the IN (ie device to Host transfers)??
	uint32_t outBuffSize;
	uint32_t outBuffCount;			// Number of bytes already sent to host from a large packet
	uint32_t ep0_state;
	uint8_t dev_state;
	uint8_t CmdOpCode;				// stores class specific operation codes (eg CDC set line config)

	struct usbRequest {
		uint8_t mRequest;
		uint8_t Request;
		uint16_t Value;
		uint16_t Index;
		uint16_t Length;

		void loadData(const uint8_t* data) {
			mRequest = data[0];
			Request = data[1];
			Value = static_cast<uint16_t>(data[2]) + (data[3] << 8);
			Index = static_cast<uint16_t>(data[4]) + (data[5] << 8);
			Length = static_cast<uint16_t>(data[6]) + (data[7] << 8);
		}
	} req;

	struct USBD_CDC_LineCodingTypeDef {
		uint32_t bitrate;    		// Data terminal rate in bits per sec.
		uint8_t format;      		// Stop Bits: 0-1 Stop Bit; 1-1.5 Stop Bits; 2-2 Stop Bits
		uint8_t paritytype;  		// Parity: 0 = None; 1 = Odd; 2 = Even; 3 = Mark; 4 = Space; 6 bDataBits 1 Data bits
		uint8_t datatype;    		// Data bits (5, 6, 7,	8 or 16)
	} USBD_CDC_LineCoding;

	// USB standard device descriptor - in usbd_desc.c
	const uint8_t USBD_FS_DeviceDesc[0x12] = {
			0x12,					// bLength
			USB_DESC_TYPE_DEVICE,	// bDescriptorType
			0x01,					// bcdUSB  - 0x01 if LPM enabled
			0x02,
			0xEF,					// bDeviceClass: (Miscellaneous)
			0x02,					// bDeviceSubClass (Interface Association Descriptor- with below)
			0x01,					// bDeviceProtocol (Interface Association Descriptor)
			ep_maxPacket,  			// bMaxPacketSize
			LOBYTE(USBD_VID),		// idVendor
			HIBYTE(USBD_VID),		// idVendor
			LOBYTE(USBD_PID_FS),	// idProduct
			HIBYTE(USBD_PID_FS),	// idProduct
			0x00,					// bcdDevice rel. 2.00
			0x02,
			USBD_IDX_MFC_STR,		// Index of manufacturer  string
			USBD_IDX_PRODUCT_STR,	// Index of product string
			USBD_IDX_SERIAL_STR,	// Index of serial number string
			0x01					// bNumConfigurations
	};

	const uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZE] = {
			// Configuration Descriptor
			0x09,								// bLength: Configuration Descriptor size
			USB_DESC_TYPE_CONFIGURATION,		// bDescriptorType: Configuration
			LOBYTE(USB_CDC_CONFIG_DESC_SIZE),	// wTotalLength
			HIBYTE(USB_CDC_CONFIG_DESC_SIZE),
			0x02,								// bNumInterfaces: 2 interfaces
			0x01,								// bConfigurationValue: Configuration value
			0x00,								// iConfiguration: Index of string descriptor describing the configuration
			0xC0,								// bmAttributes: self powered
			0x32,								// MaxPower 0 mA

			//---------------------------------------------------------------------------
	        // IAD Descriptor - Interface association descriptor for CDC class
			0x08,								// bLength (8 bytes)
			USB_DESC_TYPE_IAD,					// bDescriptorType
			0x00,								// bFirstInterface
			0x02,								// bInterfaceCount
			0x02,								// bFunctionClass (Communications and CDC Control)
			0x02,								// bFunctionSubClass
			0x01,								// bFunctionProtocol
			USBD_IDX_CDC_STR,					// iFunction (String Descriptor 6)

			// Interface Descriptor
			0x09,								// bLength: Interface Descriptor size
			USB_DESC_TYPE_INTERFACE,			// bDescriptorType: Interface
			0x00,								// bInterfaceNumber: Number of Interface
			0x00,								// bAlternateSetting: Alternate setting
			0x01,								// bNumEndpoints: One endpoints used
			0x02,								// bInterfaceClass: Communication Interface Class
			0x02,								// bInterfaceSubClass: Abstract Control Model
			0x01,								// bInterfaceProtocol: Common AT commands
			USBD_IDX_CDC_STR,					// iInterface

			// Header Functional Descriptor
			0x05,								// bLength: Endpoint Descriptor size
			0x24,								// bDescriptorType: CS_INTERFACE
			0x00,								// bDescriptorSubtype: Header Func Desc
			0x10,								// bcdCDC: spec release number
			0x01,

			// Call Management Functional Descriptor
			0x05,								// bFunctionLength
			0x24,								// bDescriptorType: CS_INTERFACE
			0x01,								// bDescriptorSubtype: Call Management Func Desc
			0x00,								// bmCapabilities: D0+D1
			0x01,								// bDataInterface: 1

			// ACM Functional Descriptor
			0x04,								// bFunctionLength
			0x24,								// bDescriptorType: CS_INTERFACE
			0x02,								// bDescriptorSubtype: Abstract Control Management desc
			0x02,								// bmCapabilities

			// Union Functional Descriptor
			0x05,								// bFunctionLength
			0x24,								// bDescriptorType: CS_INTERFACE
			0x06,								// bDescriptorSubtype: Union func desc
			0x00,								// bMasterInterface: Communication class interface
			0x01,								// bSlaveInterface0: Data Class Interface

			// Endpoint 2 Descriptor
			0x07,								// bLength: Endpoint Descriptor size
			USB_DESC_TYPE_ENDPOINT,				// bDescriptorType: Endpoint
			CDC_Cmd,							// bEndpointAddress
			Interrupt,							// bmAttributes: Interrupt
			0x08,								// wMaxPacketSize
			0x00,
			0x10,								// bInterval

			//---------------------------------------------------------------------------

			// Data class interface descriptor
			0x09,								// bLength: Endpoint Descriptor size
			USB_DESC_TYPE_INTERFACE,			// bDescriptorType:
			0x01,								// bInterfaceNumber: Number of Interface
			0x00,								// bAlternateSetting: Alternate setting
			0x02,								// bNumEndpoints: Two endpoints used
			0x0A,								// bInterfaceClass: CDC
			0x00,								// bInterfaceSubClass:
			0x00,								// bInterfaceProtocol:
			0x00,								// iInterface:

			// Endpoint OUT Descriptor
			0x07,								// bLength: Endpoint Descriptor size
			USB_DESC_TYPE_ENDPOINT,				// bDescriptorType: Endpoint
			CDC_Out,							// bEndpointAddress
			Bulk,								// bmAttributes: Bulk
			LOBYTE(ep_maxPacket),				// wMaxPacketSize:
			HIBYTE(ep_maxPacket),
			0x00,								// bInterval: ignore for Bulk transfer

			// Endpoint IN Descriptor
			0x07,								// bLength: Endpoint Descriptor size
			USB_DESC_TYPE_ENDPOINT,				// bDescriptorType: Endpoint
			CDC_In,								// bEndpointAddress
			Bulk,								// bmAttributes: Bulk
			LOBYTE(ep_maxPacket),				// wMaxPacketSize
			HIBYTE(ep_maxPacket),
			0x00								// bInterval: ignore for Bulk transfer
	};



	// Binary Object Store (BOS) Descriptor
	const uint8_t USBD_FS_BOSDesc[12] = {
			0x05,								// Length
			USB_DESC_TYPE_BOS,					// DescriptorType
			0x0C,								// TotalLength
			0x00, 0x01,							// NumDeviceCaps

			// USB 2.0 Extension Descriptor: device capability
			0x07,								// bLength
			0x10, 								// USB_DEVICE_CAPABITY_TYPE
			0x02,								// Attributes
			0x02, 0x00, 0x00, 0x00				// Link Power Management protocol is supported
	};


	uint8_t USBD_StringSerial[0x1A] = {
			0x1A,								// Length
			USB_DESC_TYPE_STRING, 				// DescriptorType
	};

	// USB lang indentifier descriptor
	const uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] = {
			USB_LEN_LANGID_STR_DESC,
			USB_DESC_TYPE_STRING,
			LOBYTE(USBD_LANGID_STRING),
			HIBYTE(USBD_LANGID_STRING)
	};

	uint8_t USBD_StrDesc[128];

public:
#if (USB_DEBUG)
	uint16_t usbDebugNo = 0;
	uint16_t usbDebugEvent = 0;

	struct usbDebugItem {
		uint16_t eventNo;
		uint32_t Interrupt;
		uint32_t IntData;
		usbRequest Request;
		uint8_t endpoint;
		uint16_t PacketSize;
		uint32_t xferBuff0;
		uint32_t xferBuff1;
	};
	usbDebugItem usbDebug[USB_DEBUG_COUNT];
	void OutputDebug();
#endif
};

