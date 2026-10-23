/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Tim Cocks (Adafruit Industries)
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

// This example demonstrates the host isochronous IN support of Pico-PIO-USB.
//
// A USB Audio Class 1 (UAC1) microphone or headset is plugged into the PIO
// USB host port. The example finds the first audio streaming interface with
// an isochronous IN endpoint, selects it, opens the endpoint with the TinyUSB
// bare endpoint API and keeps a transfer queued on it. Once a second it
// prints stream statistics and a peak level meter over the native USB CDC
// port, so a live audio stream is visible without any audio output hardware.
//
// The host supports two isochronous IN modes, selected in CMakeLists.txt:
//
// PIO_USB_ISO_RING_SIZE > 0 (default here): the host polls the endpoint
// every frame from the moment the first transfer is queued and stores each
// packet in a ring buffer as a record:
//
//   2 bytes little-endian length, then the packet data
//   length 0xFFFF: one or more packets were lost (CRC error or ring full)
//
// A queued transfer completes as soon as at least one whole record fits in
// its buffer. The application walks the records in the completion callback
// and queues the next transfer. The host stops polling after 32 frames
// without a response and fails the pending transfer; queuing the next
// transfer restarts polling. It also fails the pending transfer when
// SET_INTERFACE is sent to the device. Typing 'p' in the terminal pauses and
// resumes the stream with SET_INTERFACE. The example stops polling before
// pausing.
//
// PIO_USB_HOST_ISOCHRONOUS=1 with PIO_USB_ISO_RING_SIZE=0: every frame
// completes the queued transfer with one packet (possibly empty) or an
// error. The application must re-queue within the frame to catch the next
// packet, so this mode misses packets when the host task is slow. It is
// useful to see exactly what the device does frame by frame.
//
// For TinyUSB roothub port0 is native usb controller, roothub port1 is
// pico-pio-usb.

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "pio_usb.h"
#include "tusb.h"

#if PIO_USB_ISO_RING_SIZE
#define ISO_RING_MODE 1
#else
#define ISO_RING_MODE 0
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// UAC1 constants (USB Device Class Definition for Audio Devices 1.0)
#define UAC1_SUBCLASS_AUDIOSTREAMING 0x02
#define UAC1_AS_GENERAL 0x01       // CS_INTERFACE subtype
#define UAC1_FORMAT_TYPE 0x02      // CS_INTERFACE subtype
#define UAC1_FORMAT_TYPE_I 0x01
#define UAC1_EP_GENERAL 0x01       // CS_ENDPOINT subtype
#define UAC1_EP_CTRL_SAMPLING_FREQ 0x01 // bmAttributes bit of EP_GENERAL
#define UAC1_REQ_SET_CUR 0x01
#define UAC1_SAMPLING_FREQ_CONTROL 0x01

// Buffer handed to the host for each transfer. Several ring records fit in
// it if the application falls behind.
#define ISO_XFER_BUF_SIZE 2048

// D+ pin of the PIO USB host port (D- is the next pin). Boards in the pico-sdk
// with a PIO USB host port define PICO_DEFAULT_PIO_USB_DP_PIN.
#ifndef PIO_USB_DP_PIN
#ifdef PICO_DEFAULT_PIO_USB_DP_PIN
#define PIO_USB_DP_PIN PICO_DEFAULT_PIO_USB_DP_PIN
#else
#define PIO_USB_DP_PIN PIO_USB_DP_PIN_DEFAULT
#endif
#endif

// Optional active-high enable for the host port's 5V supply.
#ifndef PIO_USB_VBUSEN_PIN
#ifdef ADAFRUIT_FRUIT_JAM_USB_HOST_5V_POWER_PIN
#define PIO_USB_VBUSEN_PIN ADAFRUIT_FRUIT_JAM_USB_HOST_5V_POWER_PIN
#endif
#endif

typedef struct {
  uint8_t daddr; // 0 while no microphone is streaming
  uint8_t itf_num;
  uint8_t itf_alt;
  uint16_t format_tag;
  uint8_t channels;
  uint8_t subframe_size; // bytes per sample
  uint8_t bit_resolution;
  uint32_t sample_rate; // first discrete rate from the descriptor, 0 if unknown
  bool has_freq_control;
  tusb_desc_endpoint_t ep_desc;
} mic_info_t;

// Counters updated on core1 (host) and read on core0 (printing).
typedef struct {
  volatile uint32_t transfers;
  volatile uint32_t packets;
  volatile uint32_t bytes;
  volatile uint32_t lost; // 0xFFFF markers seen
  volatile uint32_t errors; // transfers that completed with an error
  volatile uint32_t empty; // packet mode: transfers that completed with 0 bytes
  volatile uint16_t min_pkt;
  volatile uint16_t max_pkt;
  volatile uint16_t peak; // largest |sample| since last print
} stream_stats_t;

static mic_info_t mic;
static stream_stats_t stats;
static volatile bool mic_info_pending; // core1 -> core0: print mic info
static volatile bool pause_request; // core0 -> core1: toggle pause
static bool paused;
static bool xfer_pending;
static bool ep_opened;

// Messages from core1 (host) to core0 (printing). Only constant strings are
// queued, so a full queue just drops the newest message.
#define MSG_QUEUE_LEN 8
static const char *msg_queue[MSG_QUEUE_LEN];
static volatile uint8_t msg_head, msg_tail;

static void post_msg(const char *msg) {
  uint8_t const next = (uint8_t)((msg_tail + 1) % MSG_QUEUE_LEN);
  if (next != msg_head) {
    msg_queue[msg_tail] = msg;
    msg_tail = next;
  }
}

CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t cfg_desc_buf[CFG_TUH_ENUMERATION_BUFSIZE];
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t iso_buf[ISO_XFER_BUF_SIZE];
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t freq_buf[4];

static void print_task(void);
static void host_request_task(void);

#ifdef ISO_MIC_DEBUG
// Dump the low-level host state of the open endpoints once a second
#include "pio_usb_ll.h"
static void cdc_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void debug_dump(void) {
  root_port_t *root = PIO_USB_ROOT_PORT(0);
  cdc_printf("  sof %lu root init %u conn %u susp %u fs %u ints 0x%lx\r\n",
             (unsigned long)pio_usb_host_get_frame_number(), root->initialized, root->connected,
             root->suspended, root->is_fullspeed, (unsigned long)root->ints);
  for (int i = 0; i < PIO_USB_EP_POOL_CNT; i++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(i);
    if (ep->size == 0) {
      continue;
    }
    cdc_printf("  ep[%d] root %u dev %u num 0x%02x attr 0x%02x size %u tx %u xfer %u started %u aborted %u actual %u total %u\r\n",
               i, ep->root_idx, ep->dev_addr, ep->ep_num, ep->attr, ep->size, ep->is_tx,
               ep->has_transfer, ep->transfer_started, ep->transfer_aborted, ep->actual_len,
               ep->total_len);
  }
}
#endif

/*------------- MAIN -------------*/

// core1: handle host events
void core1_main() {
  sleep_ms(10);

  // Use tuh_configure() to pass pio configuration to the host stack
  // Note: tuh_configure() must be called before
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIO_USB_DP_PIN;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

  // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub
  // port1) on core1
  tuh_init(1);

  while (true) {
    tuh_task(); // tinyusb host task
    host_request_task();
  }
}

// core0: handle device events
int main(void) {
  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);

  sleep_ms(10);

#ifdef PIO_USB_VBUSEN_PIN
  // Power the host port
  gpio_init(PIO_USB_VBUSEN_PIN);
  gpio_set_dir(PIO_USB_VBUSEN_PIN, GPIO_OUT);
  gpio_put(PIO_USB_VBUSEN_PIN, 1);
#endif

  multicore_reset_core1();
  // all USB host task run in core1
  multicore_launch_core1(core1_main);

  // init device stack on native usb (roothub port0)
  tud_init(0);

  while (true) {
    tud_task(); // tinyusb device task
    print_task();
    tud_cdc_write_flush();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device CDC
//--------------------------------------------------------------------+

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
  (void)itf;

  char buf[64];
  uint32_t count = tud_cdc_read(buf, sizeof(buf));
  for (uint32_t i = 0; i < count; i++) {
    if (buf[i] == 'p' || buf[i] == 'P') {
      pause_request = true;
    }
  }
}

static void cdc_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void cdc_printf(const char *fmt, ...) {
  char tempbuf[160];
  va_list args;
  va_start(args, fmt);
  int count = vsnprintf(tempbuf, sizeof(tempbuf), fmt, args);
  va_end(args);
  if (count > 0) {
    if (count > (int)sizeof(tempbuf)) {
      count = sizeof(tempbuf);
    }
    tud_cdc_write(tempbuf, count);
  }
}

// core0: print pending messages and once a second the stream statistics
static void print_task(void) {
  static bool was_connected;
  static uint32_t last_stats_ms;
  static uint32_t last_waiting_ms;
  static uint32_t last_packets, last_bytes, last_lost, last_transfers, last_errors;
  static uint32_t last_empty;

  // Nothing is printed until a terminal is attached, so the startup
  // messages are not lost.
  bool const connected = tud_cdc_connected();
  if (!connected) {
    was_connected = false;
    return;
  }
  if (!was_connected) {
    was_connected = true;
    cdc_printf("\r\nPico-PIO-USB isochronous IN example (%s mode), 'p' pauses/resumes\r\n",
               ISO_RING_MODE ? "ring buffer" : "packet");
    if (mic.daddr) {
      mic_info_pending = true;
    }
  }

  while (msg_head != msg_tail) {
    const char *msg = msg_queue[msg_head];
    msg_head = (uint8_t)((msg_head + 1) % MSG_QUEUE_LEN);
    cdc_printf("%s\r\n", msg);
  }

  if (mic_info_pending) {
    mic_info_pending = false;
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(mic.daddr, &vid, &pid);
    cdc_printf("[%04x:%04x][%u] streaming interface %u alt %u, endpoint 0x%02x, max packet %u\r\n",
               vid, pid, mic.daddr, mic.itf_num, mic.itf_alt,
               mic.ep_desc.bEndpointAddress, tu_edpt_packet_size(&mic.ep_desc));
    cdc_printf("  format tag %u, %u channel(s), %u bytes/sample, %u bits, %lu Hz%s\r\n",
               mic.format_tag, mic.channels, mic.subframe_size, mic.bit_resolution,
               (unsigned long)mic.sample_rate,
               mic.has_freq_control ? " (sample rate set)" : "");
    // Start the per-second deltas from now, not from when streaming began
    last_packets = stats.packets;
    last_bytes = stats.bytes;
    last_lost = stats.lost;
    last_transfers = stats.transfers;
    last_errors = stats.errors;
    last_empty = stats.empty;
    stats.peak = 0;
    last_stats_ms = to_ms_since_boot(get_absolute_time());
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());

  if (!mic.daddr) {
    if (now - last_waiting_ms >= 5000) {
      last_waiting_ms = now;
      cdc_printf("Waiting for a USB Audio Class 1 microphone on the PIO USB port...\r\n");
    }
    return;
  }

  if (now - last_stats_ms < 1000) {
    return;
  }
  last_stats_ms = now;

  uint32_t packets = stats.packets, bytes = stats.bytes, lost = stats.lost;
  uint32_t transfers = stats.transfers, errors = stats.errors;
  uint32_t empty = stats.empty;
  uint16_t peak = stats.peak;
  stats.peak = 0;

  // Crude level meter: 0..32767 mapped to 0..20 characters
  char meter[24];
  unsigned level = (peak * 20u) / 32768u;
  for (unsigned i = 0; i < 20; i++) {
    meter[i] = (i < level) ? '#' : '.';
  }
  meter[20] = '\0';

  cdc_printf("%4lu pkt/s %6lu B/s  pkt %u..%u  xfers %lu  lost %lu  empty %lu  err %lu  peak %5u |%s|%s\r\n",
             (unsigned long)(packets - last_packets), (unsigned long)(bytes - last_bytes),
             stats.min_pkt == 0xffff ? 0 : stats.min_pkt, stats.max_pkt,
             (unsigned long)(transfers - last_transfers), (unsigned long)(lost - last_lost),
             (unsigned long)(empty - last_empty), (unsigned long)(errors - last_errors),
             peak, meter, paused ? " paused" : "");

  last_packets = packets;
  last_bytes = bytes;
  last_lost = lost;
  last_transfers = transfers;
  last_errors = errors;
  last_empty = empty;

#ifdef ISO_MIC_DEBUG
  debug_dump();
#endif
}

//--------------------------------------------------------------------+
// Host: configuration descriptor parsing
//--------------------------------------------------------------------+

// Find the first UAC1 audio streaming alternate setting that has an
// isochronous IN endpoint. Returns false if the device has none.
static bool find_microphone(uint8_t const *desc, uint16_t total_len, mic_info_t *out) {
  uint8_t const *p = desc;
  uint8_t const *end = desc + total_len;

  mic_info_t cand;
  bool in_streaming_itf = false;
  bool have_ep = false;
  memset(&cand, 0, sizeof(cand));

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t const len = p[0];
    uint8_t const type = p[1];

    if (type == TUSB_DESC_INTERFACE && len >= sizeof(tusb_desc_interface_t)) {
      if (have_ep) {
        break; // finished the alternate setting we are interested in
      }
      tusb_desc_interface_t const *itf = (tusb_desc_interface_t const *)p;
      in_streaming_itf = (itf->bInterfaceClass == TUSB_CLASS_AUDIO) &&
                         (itf->bInterfaceSubClass == UAC1_SUBCLASS_AUDIOSTREAMING) &&
                         (itf->bAlternateSetting != 0);
      memset(&cand, 0, sizeof(cand));
      cand.itf_num = itf->bInterfaceNumber;
      cand.itf_alt = itf->bAlternateSetting;
    } else if (in_streaming_itf && type == TUSB_DESC_CS_INTERFACE && len >= 7 &&
               p[2] == UAC1_AS_GENERAL) {
      cand.format_tag = (uint16_t)(p[5] | (p[6] << 8));
    } else if (in_streaming_itf && type == TUSB_DESC_CS_INTERFACE && len >= 8 &&
               p[2] == UAC1_FORMAT_TYPE && p[3] == UAC1_FORMAT_TYPE_I) {
      cand.channels = p[4];
      cand.subframe_size = p[5];
      cand.bit_resolution = p[6];
      // bSamFreqType: 0 = continuous (lower, upper), n = n discrete rates.
      // Either way the first 3-byte value is a valid rate.
      if (len >= 11) {
        cand.sample_rate = (uint32_t)(p[8] | (p[9] << 8) | (p[10] << 16));
      }
    } else if (in_streaming_itf && !have_ep && type == TUSB_DESC_ENDPOINT &&
               len >= sizeof(tusb_desc_endpoint_t)) {
      tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
      if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN &&
          ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
        memcpy(&cand.ep_desc, p, sizeof(tusb_desc_endpoint_t));
        have_ep = true;
      }
    } else if (in_streaming_itf && have_ep && type == TUSB_DESC_CS_ENDPOINT && len >= 4 &&
               p[2] == UAC1_EP_GENERAL) {
      cand.has_freq_control = (p[3] & UAC1_EP_CTRL_SAMPLING_FREQ) != 0;
    }

    p += len;
  }

  if (!have_ep) {
    return false;
  }
  *out = cand;
  return true;
}

//--------------------------------------------------------------------+
// Host: streaming
//--------------------------------------------------------------------+

// Peak |sample| of PCM data, treating the top two bytes of every sample as
// a signed 16-bit value. Channels are interleaved, so the peak is over all
// channels.
static void update_peak(uint8_t const *data, uint16_t len) {
  uint8_t const n = mic.subframe_size ? mic.subframe_size : 2;
  uint16_t peak = stats.peak;
  for (uint16_t i = 0; i + n <= len; i += n) {
    int16_t sample;
    if (n >= 2) {
      sample = (int16_t)(data[i + n - 2] | (data[i + n - 1] << 8));
    } else {
      sample = (int16_t)((int8_t)data[i] << 8);
    }
    uint16_t mag = (sample < 0) ? (uint16_t)(-(int32_t)sample) : (uint16_t)sample;
    if (mag > peak) {
      peak = mag;
    }
  }
  stats.peak = peak;
}

#if ISO_RING_MODE
// Walk the ring records delivered by one transfer
static void process_records(uint8_t const *buf, uint32_t len) {
  uint32_t pos = 0;
  while (pos + 2 <= len) {
    uint16_t const rec_len = (uint16_t)(buf[pos] | (buf[pos + 1] << 8));
    pos += 2;
    if (rec_len == 0xffff) {
      stats.lost++;
      continue;
    }
    if (pos + rec_len > len) {
      stats.errors++; // partial record: should not happen
      break;
    }
    stats.packets++;
    stats.bytes += rec_len;
    if (rec_len < stats.min_pkt) {
      stats.min_pkt = rec_len;
    }
    if (rec_len > stats.max_pkt) {
      stats.max_pkt = rec_len;
    }
    update_peak(buf + pos, rec_len);
    pos += rec_len;
  }
  stats.transfers++;
}
#endif

static void queue_iso_transfer(void);

#if !ISO_RING_MODE
// Packet mode: one transfer is one packet
static void process_packet(uint8_t const *buf, uint32_t len) {
  if (len == 0) {
    stats.empty++;
  } else {
    stats.packets++;
    stats.bytes += len;
    if (len < stats.min_pkt) {
      stats.min_pkt = (uint16_t)len;
    }
    if (len > stats.max_pkt) {
      stats.max_pkt = (uint16_t)len;
    }
    update_peak(buf, (uint16_t)len);
  }
  stats.transfers++;
}
#endif

// Every completion, including a failed one, queues the next transfer unless
// the stream is paused. A failed transfer means the host stopped polling a
// silent device; queuing again restarts polling.
static void iso_complete_cb(tuh_xfer_t *xfer) {
  xfer_pending = false;
  if (xfer->result == XFER_RESULT_SUCCESS) {
#if ISO_RING_MODE
    process_records(iso_buf, xfer->actual_len);
#else
    process_packet(iso_buf, xfer->actual_len);
#endif
  } else {
    stats.errors++;
  }
  if (mic.daddr && !paused) {
    queue_iso_transfer();
  }
}

static void queue_iso_transfer(void) {
  tuh_xfer_t xfer = {
      .daddr = mic.daddr,
      .ep_addr = mic.ep_desc.bEndpointAddress,
      .buflen = sizeof(iso_buf),
      .buffer = iso_buf,
      .complete_cb = iso_complete_cb,
      .user_data = 0,
  };
  xfer_pending = true;
  if (!tuh_edpt_xfer(&xfer)) {
    xfer_pending = false;
    post_msg("Failed to queue isochronous transfer");
    mic.daddr = 0;
  }
}

static void pause_cb(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    post_msg("SET_INTERFACE alt 0 failed");
  }
}

static void set_interface_cb(tuh_xfer_t *xfer);

// core1: pause or resume the device with SET_INTERFACE. Polling stops
// before the interface is switched.
static void host_request_task(void) {
  if (!pause_request) {
    return;
  }
  pause_request = false;
  if (!mic.daddr) {
    return;
  }
  if (!paused) {
    paused = true;
    if (xfer_pending) {
      tuh_edpt_abort_xfer(mic.daddr, mic.ep_desc.bEndpointAddress);
      xfer_pending = false;
    }
    post_msg("Paused: polling stopped, SET_INTERFACE alt 0");
    if (!tuh_interface_set(mic.daddr, mic.itf_num, 0, pause_cb, 0)) {
      post_msg("Failed to send SET_INTERFACE");
    }
  } else {
    post_msg("Resuming: SET_INTERFACE streaming alt, sampling frequency, polling");
    if (!tuh_interface_set(mic.daddr, mic.itf_num, mic.itf_alt, set_interface_cb, 0)) {
      post_msg("Failed to send SET_INTERFACE");
    }
  }
}

// Called after SET_INTERFACE (and the sampling frequency request) both at
// mount and at resume.
static void start_streaming(void) {
  if (!ep_opened) {
    if (!tuh_edpt_open(mic.daddr, &mic.ep_desc)) {
      post_msg("Failed to open isochronous endpoint");
      mic.daddr = 0;
      return;
    }
    ep_opened = true;
    memset(&stats, 0, sizeof(stats));
    stats.min_pkt = 0xffff;
    mic_info_pending = true;
  }
  paused = false;

  // The first transfer starts per-frame polling of the endpoint
  if (!xfer_pending) {
    queue_iso_transfer();
  }
}

static void set_sample_rate_cb(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    post_msg("Warning: SET_CUR sampling frequency failed, streaming anyway");
  }
  start_streaming();
}

static void set_interface_cb(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    post_msg("SET_INTERFACE failed");
    mic.daddr = 0;
    return;
  }

  if (mic.has_freq_control && mic.sample_rate) {
    // UAC1 endpoint request: SET_CUR SAMPLING_FREQ_CONTROL, 3-byte rate
    freq_buf[0] = (uint8_t)(mic.sample_rate & 0xff);
    freq_buf[1] = (uint8_t)((mic.sample_rate >> 8) & 0xff);
    freq_buf[2] = (uint8_t)((mic.sample_rate >> 16) & 0xff);

    tusb_control_request_t const request = {
        .bmRequestType_bit = {.recipient = TUSB_REQ_RCPT_ENDPOINT,
                              .type = TUSB_REQ_TYPE_CLASS,
                              .direction = TUSB_DIR_OUT},
        .bRequest = UAC1_REQ_SET_CUR,
        .wValue = tu_htole16(UAC1_SAMPLING_FREQ_CONTROL << 8),
        .wIndex = tu_htole16(mic.ep_desc.bEndpointAddress),
        .wLength = tu_htole16(3),
    };
    tuh_xfer_t ctrl = {
        .daddr = mic.daddr,
        .ep_addr = 0,
        .setup = &request,
        .buffer = freq_buf,
        .complete_cb = set_sample_rate_cb,
        .user_data = 0,
    };
    if (tuh_control_xfer(&ctrl)) {
      return;
    }
    post_msg("Warning: could not send sampling frequency request");
  }

  start_streaming();
}

static void config_desc_cb(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    post_msg("Failed to get configuration descriptor");
    return;
  }

  tusb_desc_configuration_t const *cfg = (tusb_desc_configuration_t const *)cfg_desc_buf;
  uint16_t total_len = tu_le16toh(cfg->wTotalLength);
  if (total_len > sizeof(cfg_desc_buf)) {
    total_len = sizeof(cfg_desc_buf); // truncated: parse what we have
  }

  mic_info_t found;
  if (!find_microphone(cfg_desc_buf, total_len, &found)) {
    post_msg("Device has no UAC1 audio streaming interface with an isochronous IN endpoint");
    return;
  }
  found.daddr = xfer->daddr;
  mic = found;

  // Alternate setting 0 carries no endpoint; select the streaming one.
  if (!tuh_interface_set(mic.daddr, mic.itf_num, mic.itf_alt, set_interface_cb, 0)) {
    post_msg("Failed to send SET_INTERFACE");
    mic.daddr = 0;
  }
}

//--------------------------------------------------------------------+
// Host: TinyUSB callbacks
//--------------------------------------------------------------------+

// Invoked when a device is mounted (configured)
void tuh_mount_cb(uint8_t daddr) {
  if (mic.daddr) {
    post_msg("Another device mounted; already streaming from one microphone");
    return;
  }
  if (!tuh_descriptor_get_configuration(daddr, 0, cfg_desc_buf, sizeof(cfg_desc_buf),
                                        config_desc_cb, 0)) {
    post_msg("Failed to request configuration descriptor");
  }
}

// Invoked when a device is unmounted. TinyUSB closes the device's endpoints,
// which also stops the host polling the isochronous endpoint.
void tuh_umount_cb(uint8_t daddr) {
  if (daddr == mic.daddr) {
    mic.daddr = 0;
    ep_opened = false;
    xfer_pending = false;
    paused = false;
    post_msg("Microphone unplugged");
  }
}
