#include <tusb.h>

/*
 * since I'm not using tinyusb_board, I have to put a declaration here
 * so the function I define in main.c can be used
 */
size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_chars);

#include "pico/usb_reset_interface.h"

/* TUD_RPI_RESET_DESC_LEN and TUD_RPI_RESET_DESCRIPTOR are probably in that usb_reset_interface_device.h header I can't find */
#ifndef TUD_RPI_RESET_DESC_LEN
#define TUD_RPI_RESET_DESC_LEN	9
#endif

#ifndef TUD_RPI_RESET_DESCRIPTOR
#define TUD_RPI_RESET_DESCRIPTOR(_itfnum, _stridx) \
  /* Interface */\
  TUD_RPI_RESET_DESC_LEN, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_VENDOR_SPECIFIC, RESET_INTERFACE_SUBCLASS, RESET_INTERFACE_PROTOCOL, _stridx
#endif

/*
 * neat trick: _PID_MAP(interface_name, shift) can be used to provide customized product IDs based on interfaces
 * #define _PID_MAP(itf, n)  ((CFG_TUD_##itf) << (n))
 * #define USB_PID   (0x4100 | _PID_MAP(CDC, 0))
 */

/*
 * use the default rpi pico vid/pid because picotool is a little dumb
 * basically, picotool allows you to specify a vid:pid to reset, but when the target board
 * is reset, it has the default (2e8a:000a) vid:pid, and picotool is still looking for
 * the one you told it to look for.
 */
#define USB_VID		0x2e8a
#define USB_PID		0x000a

/* use a value of 2.10 so the MS_OS_20_DESCRIPTOR works */
#define USB_BCD		0x0210


/* String descriptor indices, referenced as ".i<whatever>" in the descriptor tables */
enum {
	STRID_LANGID = 0,				/* 0: supported language ID */
	STRID_MANUFACTURER,				/* 1: Manufacturer */
	STRID_PRODUCT,					/* 2: Product */
	STRID_SERIAL,					/* 3: Serial Number */
	STRID_CDC_0,					/* 4: CDC Interface 0 */
	STRID_CDC_1,					/* 5: CDC Interface 1 */
	STRID_RPI_RESET,				/* 6: Reset Interface */
};


/* array of pointer to the actual strings used in the descriptors */
char const *string_desc_arr[] = {
	(const char[]) { 0x09, 0x04 },			/* 0: English (0x0409, little-endian) */
	"Raspberry Pi",					/* 1: Manufacturer */
	"Pico",						/* 2: Product */
	NULL,						/* 3: Serial (NULL will pull the pico's unique ID) */
	"pico-sdk stdio interface",			/* 4: CDC Interface 0 */
	"secondary CDC interface",			/* 5: CDC Interface 1 */
	"Reset"						/* 6: Reset Interface */
};

/* device descriptor */
tusb_desc_device_t const desc_device = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = USB_BCD,

	.bDeviceClass = TUSB_CLASS_MISC,		/* CDC is a subclass of misc */
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,	/* CDC uses common subclass */
	.bDeviceProtocol = MISC_PROTOCOL_IAD,		/* CDC uses IAD */

	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,	/* 64 bytes */

	.idVendor = USB_VID,
	.idProduct = USB_PID,
	.bcdDevice = 0x0100,				/* Device release number */

	.iManufacturer = STRID_MANUFACTURER,		/* index of the manufacturer string */
	.iProduct = STRID_PRODUCT,			/* index of the product string */
	.iSerialNumber = STRID_SERIAL,			/* Index of serial number string */

	.bNumConfigurations = 0x01			/* 1 configuration */
};


/* interface definitions, used by the configuration descriptor */
enum {
	ITF_NUM_CDC_0 = 0,
	ITF_NUM_CDC_0_DATA,
	ITF_NUM_CDC_1,
	ITF_NUM_CDC_1_DATA,
	ITF_NUM_RPI_RESET,
	ITF_NUM_TOTAL
};

/* this saves you some headache -- the rpi reset interface number must match in CMakeLists.txt or it won't work. This is hell to debug */
static_assert(ITF_NUM_RPI_RESET == PICO_STDIO_USB_RESET_INTERFACE_MS_OS_20_DESCRIPTOR_ITF, "ITF_NUM_RPI_RESET must be equal to the PICO_STDIO_USB_RESET_INTERFACE_MS_OS_20_DESCRIPTOR_ITF set in CMakeLists.txt");

/* define the endpoint numbers for the configuration descriptor */
#define EPNUM_CDC_0_NOTIF	0x81	/* notification endpoint for CDC 0 */
#define EPNUM_CDC_0_OUT		0x02	/* out endpoint for CDC 0 */
#define EPNUM_CDC_0_IN		0x82	/* in endpoint for CDC 0 */

#define EPNUM_CDC_1_NOTIF	0x84	/* notification endpoint for CDC 1 */
#define EPNUM_CDC_1_OUT		0x05	/* out endpoint for CDC 1 */
#define EPNUM_CDC_1_IN		0x85	/* in endpoint for CDC 1 */

/* total length of configuration descriptor */
#define CONFIG_TOTAL_LEN	(TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN + TUD_RPI_RESET_DESC_LEN)

/* configuration descriptor - two CDC interfaces plus the picoloader reset interface */
uint8_t const desc_configuration[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_0, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, STRID_CDC_1, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),
	TUD_RPI_RESET_DESCRIPTOR(ITF_NUM_RPI_RESET, STRID_RPI_RESET)
};

/* device qualifier descriptor */
tusb_desc_device_qualifier_t const desc_device_qualifier = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = USB_BCD,

	.bDeviceClass = TUSB_CLASS_CDC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,

	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.bNumConfigurations = 0x01,
	.bReserved = 0x00
};

// --------------------------------------------------------------------+
// IMPLEMENTATION
// --------------------------------------------------------------------+

uint8_t const *tud_descriptor_device_cb(void)
{
	return (uint8_t const *)&desc_device;
}

uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
	return (uint8_t const *)&desc_device_qualifier;
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
	// avoid unused parameter warning and keep function signature consistent
	(void)index;

	return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	/* MUST BE STATIC - needs to survive beyond the function call lifetime */
	/* the size is + 1 byte to have room to record the string length and type at position 0 */
	static uint16_t buf[32 + 1];
	size_t len;

	(void)langid;

	/* return the requested string descriptor */
	switch (index) {
	case STRID_LANGID:
		memcpy(&buf[1], string_desc_arr[STRID_LANGID], 2);
		len = 1;
		break;

	case STRID_SERIAL:
		len = board_usb_get_serial(&buf[1], 32);
		break;

	default:
		/* COPYRIGHT NOTE: Based on TinyUSB example. Windows wants UTF16LE */

		/* if the host requested a string we don't have, return NULL */
		if (! (index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
			return NULL;
		}

		// Copy string descriptor into buf
		const char *str = string_desc_arr[index];
		len = strlen(str);
		size_t const max_count = sizeof(buf) / sizeof(buf[0]) - 1; /* -1 for the length */

		/* Cap at max char */
		if (len > max_count) {
			len = max_count;
		}

		/* Convert ASCII string into UTF-16 */
		for (size_t i = 0; i < len; i++) {
			buf[1 + i] = str[i];
		}
		break;
	}

	/* store the string length in the first byte and the string type in the second byte */
	buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (len * 2 + 2));
	return buf;
}
