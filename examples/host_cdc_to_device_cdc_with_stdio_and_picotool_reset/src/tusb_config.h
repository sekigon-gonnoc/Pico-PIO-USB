#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TINYUSB CONFIGURATION
 */

#define CFG_TUSB_OS			OPT_OS_PICO

/* enable tinyusb device support */
#define CFG_TUD_ENABLED			(1)

/* enable tinyusb host support (and point it to the PIO-USB interface) */
#define CFG_TUH_ENABLED			(1)
#define CFG_TUH_RPI_PIO_USB		(1)

/*
 * Tell Pico-USB-PIO where the D+/D- pins are located
 * The RP2040-PiZero I am using has GPIO pins 6/7 for D+/D-
 */
#define PIO_USB_DP_PIN_DEFAULT		(6)


/* used by tinyusb to tweak the placement of data structures */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

/* used by tinyusb to set alignment of data structures */
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN	__attribute__ ((aligned(4)))
#endif

/*
 * DEVICE CONFIGURATION
 */

/* Enable support for 2 CDC devices */
#define CFG_TUD_CDC			(2)

/* configure the CDC buffer sizes for the device */
#define CFG_TUD_CDC_RX_BUFSIZE		(64)
#define CFG_TUD_CDC_TX_BUFSIZE		(64)
#define CFG_TUD_CDC_EP_BUFSIZE		(64)

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE		(64)
#endif

/*
 * HOST CONFIGURATION
 */

/* enable USB hub support and set the number of devices the host will support */
#define CFG_TUH_HUB			(1)
#define CFG_TUH_DEVICE_MAX		(CFG_TUH_HUB ? 4 : 1) /* hubs typically have 4 ports */

/* enable up to two CDC devices to be driven by the host */
#define CFG_TUH_CDC			(2)

/* enable support for all the goofy serial devices that aren't actually CDC */
#define CFG_TUH_CDC_FTDI		(1)
#define CFG_TUH_CDC_CP210X		(1)
#define CFG_TUH_CDC_PL2303		(1)
#define CFG_TUH_CDC_CH34X		(1)

/* Set CDC FIFO buffer sizes for the host */
#define CFG_TUH_CDC_RX_BUFSIZE  (64)
#define CFG_TUH_CDC_TX_BUFSIZE  (64)
#define CFG_TUH_CDC_EP_BUFSIZE  (64)

/* set the enumeration buffer size */
#define CFG_TUH_ENUMERATION_BUFSIZE	(256)

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
