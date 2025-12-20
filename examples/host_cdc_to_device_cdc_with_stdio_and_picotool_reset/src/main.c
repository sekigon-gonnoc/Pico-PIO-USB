#include "pico/stdlib.h"
#include <pico/stdio.h>
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"
#include "pico/binary_info.h"
#include "pico/util/datetime.h"

#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/rtc.h"

#include <tusb.h>
#include "pio_usb.h"

#include <string.h>

/*
 * copied from tinyusb's hw/bsp/rp2040/family.c
 * I don't include tinyusb_board because it still uses the old/deprecated
 * tusb init code in board_init().
 *
 * RP2040 does not have a unique on-board ID, but this is a standard feature
 * on the NOR flash it boots from. There is a 1:1 association between RP2040
 * and the flash, so this can be used to get a 64 bit globally unique board ID
 * for an RP2040-based board, including Pico.
 * 
 * The pico_unique_id library retrieves this unique identifier during boot and
 * stores it in memory, where it can be accessed at any time without
 * disturbing the flash XIP hardware.
 */
size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_len)
{
	pico_unique_board_id_t id;
	size_t len = PICO_UNIQUE_BOARD_ID_SIZE_BYTES;

	pico_get_unique_board_id(&id);

	if (len > max_len) {
		len = max_len;
	}

	/* convert to UTF-16-LE */
	for (size_t i = 0; i < len; i++) {
		for (size_t j = 0; j < 2; j++) {
			const char nybble_to_hex[16] = {
				'0', '1', '2', '3', '4', '5', '6', '7',
				'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
			};

			uint8_t const nybble = (id.id[i] >> (j * 4)) & 0xf;
			desc_str1[i * 2 + (1 - j)] = nybble_to_hex[nybble];
		}
	}

	return 2 * len;
}


/* callback when one of the CDC devices created by the device stack receives data */
void tud_cdc_rx_cb(uint8_t idx)
{
	int len;
	uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

	/*
	 * read the available data
	 * IMPORTANT: you must always read the data (no matter the interface)
	 * or the CDC device will end up blocking and you won't be able to
	 * use printf()
	 */
	len = tud_cdc_n_read(idx, buf, sizeof(buf));

	/*
	 * on the first CDC interface, sdio_usb should be taking care of things.
	 * however we're using the second CDC interface to act as a transparent
	 * bridge between the PC and whatever the second CDC device we have connected
	 * on the host side (if any)
	 */
	if (idx == 1 && len > 0) {
		tuh_cdc_write(1, buf, len);
		tuh_cdc_write_flush(1);
	}
}


/* This function can be used for any additional processing
 * required for the second CDC device interface we've created
 * (e.g. we could see if characters from a UART were available or
 * perhaps an ADC has performed a conversion, etc.)
 *
 * since this example is just mirroring data between the second CDC
 * device connected to the pico's host interface and the second CDC
 * device it creates for the connected computer, nothing has to be
 * done here since it's all being handled in the callbacks.
 *
 * i.e.
 * tinyusb_device -> tinyusb_host is handled by tud_cdc_rx_cb()
 * tinyusb_host -> tinyusb_device is handled by tuh_cdc_rx_cb()
 */
static void cdc1_task(void)
{
	/* do nothing */
}


/* called when the host stack detects a USB device */
void tuh_mount_cb(uint8_t addr)
{
	uint16_t vid, pid;

	tuh_vid_pid_get(addr, &vid, &pid);
	printf("%s(0x%02hhx): 0x%04x:%04x connected\n", __func__, addr, vid, pid);
}


/* called when the host stack notices a device has been removed */
void tuh_umount_cb(uint8_t addr)
{
	printf("%s(0x%02hhx): device disconnected\n", __func__, addr);
}


/* called for every usb host event -- this is noisy and can be called within IRQ context */
void tuh_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr)
{
	/* shut up the compiler about unused variable warnings */
	(void)rhport;
	(void)eventid;
	(void)in_isr;

//	printf("%s(port %d, event 0x%08x, in_isr: %d)\n", __func__, rhport, eventid, in_isr);
}


/* called when a CDC device managed by the host stack is connected */
void tuh_cdc_mount_cb(uint8_t idx)
{
	uint16_t vid, pid;

	tuh_vid_pid_get(idx, &vid, &pid);
	printf("%s(idx %d): CDC Device 0x%04x:%04x connected\n", __func__, idx, vid, pid);

	/* set the baudrate and data format to 115200,N81 */
	tuh_cdc_set_baudrate(idx, 115200, NULL, 0);
	tuh_cdc_set_data_format(idx, 1, 0, 8, NULL, 0);
}


/* called when a CDC device managed by the host stack is disconnected */
void tuh_cdc_umount_cb(uint8_t idx)
{
	printf("%s(idx %d): CDC device disconnected\n", __func__, idx);
}


/*
 * called whenever a CDC device managed by the host stack has data available
 * NOTE: some devices don't stall when there's no data which means that
 * this function will be called over and over and tuh_cdc_read() will return 0 bytes read
 */
void tuh_cdc_rx_cb(uint8_t idx)
{
	int len;
	uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE], *p;

	if ((len = tuh_cdc_read(idx, buf, sizeof(buf))) > 0) {

		/*
		 * "idx" indicates WHICH CDC device has data
		 * for the first CDC device connected, I just dump what it wrote.
		 * for the second CDC device, I "pass through" the data to the second
		 * CDC device I created on the device side (ie the second serial port that
		 * your computer sees when you plug in this device)
		 */
		switch (idx) {
		case 0:
			printf("%s(idx %d): rx %d bytes:", __func__, idx, len);
			p = buf;
			while (len--) { printf(" %02hhx", *p++); }
			printf("\n");
			break;

		case 1:
			if (tud_cdc_n_connected(1)) {
				tud_cdc_n_write(1, buf, len);
				tud_cdc_n_write_flush(1);
			}
			break;

		default:
			/* do nothing */
			break;
		};
	}
}


/*
 * called whenever a device managed by the host stack has
 * finished sending the data sent with tuh_cdc_write()
 */
void tuh_cdc_tx_complete_cb(uint8_t idx)
{
	(void)idx;
	/* do nothing */
}


/*
 * this is called by the repeating timer
 * it performs the periodic processing that the tinyusb device stack needs to do
 */
static bool tud_task_cb(__unused struct repeating_timer *t)
{
	tud_task();
	return true;
}


/*
 * entry point for the second ARM core
 * all we're using CORE1 for is the tinyusb host stack processing
 * TODO: see if we can do it all on one core
 */
static void core1_main(void)
{
	sleep_ms(10);

	/*
	 * tuh_configure() must be called to "point" the host stack to the correct USB port
	 * we want to use. In our case, it is the PIO-USB port that Pico-PIO-USB is managing
	 *
	 * PIO_USB_DEFAULT_CONFIG is defined in src/pio_usb_configuration.h in the Pico-PIO-USB repo
	 * the default PIO pins for D+/D- are 0/1, set PIO_USB_DP_PIN_DEFAULT in either tusb_config.h
	 * or the CMakeLists.txt to change the pins used for the PIO USB port
	 * e.g.
	 *     #define PIO_USB_DP_PIN_DEFAULT	(6)	// use pin 6/7 for D+/D-
	 *
	 * in tusb_config.h, or
	 *     target_compile_definitions(${PROJECT_NAME} PUBLIC
         *         PIO_USB_DP_PIN_DEFAULT=6
         *
         * in CMakeLists.txt.
         *
         * There are other configuration options available, see pio_usb_configuration.h
         *
         * TUH_CFGID_RPI_PIO_USB_CONFIGURATION is a "magic number" that is used by
         * Pico-PIO-USB to recognize that we are running on an rpi pico. It is defined
         * in tinyusb's src/host/usbh.h.
         */
	pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
	tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

	/* the core that tuh_init() is called on is the core that the USB SOF interrupt will run on. */
	tuh_init(1);

	/* loop forever, running the periodic host task */
	while (1) {
		tuh_task();
	};
}


static void init_tusb_dev_and_host(void)
{
	/* USB requires the clock to be a multiple of 12MHz so we use 120MHz */
	set_sys_clock_khz(120000, true);

	sleep_ms(10);

	/*
	 * core1 is used for the USB host interface
	 * (TODO: see if everything can be done on one core)
	 */
	multicore_reset_core1();
	multicore_launch_core1(core1_main);

	/* initialize the USB device stack on port 0 (ie the dedicated USB peripheral on RP2040) */
	tud_init(0);

	stdio_init_all();
}


int main(void)
{
	struct repeating_timer tud_task_timer;

	/* just an arbitrary date */
	datetime_t dt = {
		.year  = 2025, .month = 12, .day   = 19, .dotw  = 5,	/* 0 is Sunday, so 5 is Friday */
		.hour  = 14, .min   = 10, .sec   = 00
	};

	if (watchdog_enable_caused_reboot()) {
	}

	/* 100ms timeout, pause wdt when debugging */
//	watchdog_enable(100, 1);

	rtc_init();
	rtc_set_datetime(&dt);

	init_tusb_dev_and_host();

	/* create a 1ms repeating timer which calls tud_task_cb() for the tinyusb device stack processing */
	add_repeating_timer_ms(1, tud_task_cb, NULL, &tud_task_timer);

	while (1) {
		static datetime_t dt2;

		/*
		 * get the RTC date and time
		 * if it's changed (i.e. a second has passed) then print the time
		 * to the first CDC device this example creates
		 */
		rtc_get_datetime(&dt);
		if (memcmp(&dt, &dt2, sizeof(dt)) != 0) {
			char buf[30];

			datetime_to_str(buf, sizeof(buf), &dt);
			printf("\r%s", buf);

			/* copy the new time so we detect the next time it changes */
			memcpy(&dt2, &dt, sizeof(dt));
		}

		/*
		 * any data received on the second CDC device interface
		 * (e.g. second COM port this creates for the host computer)
		 * is sent to the second CDC device connected on the pico's host side
		 */
		cdc1_task();

		watchdog_update();
	};

	cancel_repeating_timer(&tud_task_timer);
	return 0;
}
