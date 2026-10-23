/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "pio_usb.h"
#include "pio_usb_ll.h"
#include "usb_crc.h"

enum {
  TRANSACTION_MAX_RETRY = 3, // Number of times to retry a failed transaction
};

static alarm_pool_t *_alarm_pool = NULL;
static repeating_timer_t sof_rt;
// The sof_count may be incremented and then read on different cores.
static volatile uint32_t sof_count = 0;
static bool timer_active;

static volatile bool cancel_timer_flag;
static volatile bool start_timer_flag;
static __unused uint32_t int_stat;
static uint8_t sof_packet[4] = {USB_SYNC, USB_PID_SOF, 0x00, 0x10};
static uint8_t sof_packet_encoded[4 * 2 * 7 / 6 + 2];
static uint8_t sof_packet_encoded_len;
static uint8_t keepalive_encoded[1];

static bool sof_timer(repeating_timer_t *_rt);

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

static void start_timer(alarm_pool_t *alarm_pool) {
  if (timer_active) {
    return;
  }

  if (alarm_pool != NULL) {
    alarm_pool_add_repeating_timer_us(alarm_pool, -1000, sof_timer, NULL,
                                      &sof_rt);
  }

  timer_active = true;
}

static __unused void stop_timer(void) {
  cancel_repeating_timer(&sof_rt);
  timer_active = false;
}

// Detector cycles to wait after our EOP released it, so that its first sample
// sees SE0 through the GPIO input synchronizers rather than our last data bit
#define RX_ARM_DELAY 7

static void set_rx_arm_delay(pio_port_t *pp) {
  if (pp->pio_usb_tx != pp->pio_usb_rx) {
    return; // released by the CPU, see usb_transfer()
  }

  const pio_program_t *eop = pp->debug_pin_eop < 0 ? &usb_edge_detector_program
                                                   : &usb_edge_detector_debug_program;
  pp->pio_usb_rx->instr_mem[pp->offset_eop] =
      eop->instructions[0] | pio_encode_delay(RX_ARM_DELAY);
}

usb_device_t *pio_usb_host_init(const pio_usb_configuration_t *c) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t *root = PIO_USB_ROOT_PORT(0);

  pio_usb_bus_init(pp, c, root);
  root->mode = PIO_USB_MODE_HOST;
  set_rx_arm_delay(pp);

  float const cpu_freq = (float)clock_get_hz(clk_sys);
  pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 6000000,
                                  &pp->clk_div_ls_tx.div_int,
                                  &pp->clk_div_ls_tx.div_frac);

  pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 12000000,
                                  &pp->clk_div_ls_rx.div_int,
                                  &pp->clk_div_ls_rx.div_frac);

  sof_packet_encoded_len =
      pio_usb_ll_encode_tx_data(sof_packet, sizeof(sof_packet), sof_packet_encoded);
  pio_usb_ll_encode_tx_data(NULL, 0, keepalive_encoded);

  if (!c->skip_alarm_pool) {
    _alarm_pool = c->alarm_pool;
    if (!_alarm_pool) {
      _alarm_pool = alarm_pool_create(2, 1);
    }
  }
  start_timer(_alarm_pool);

  return &pio_usb_device[0];
}

void pio_usb_host_stop(void) {
  cancel_timer_flag = true;
  while (cancel_timer_flag) {
    continue;
  }
}

void pio_usb_host_restart(void) {
  start_timer_flag = true;
  while (start_timer_flag) {
    continue;
  }
}

//--------------------------------------------------------------------+
// Bus functions
//--------------------------------------------------------------------+

static void __no_inline_not_in_flash_func(override_pio_program)(PIO pio, const pio_program_t* program, uint offset) {
    for (uint i = 0; i < program->length; ++i) {
      uint16_t instr = program->instructions[i];
      pio->instr_mem[offset + i] =
          pio_instr_bits_jmp != _pio_major_instr_bits(instr) ? instr
                                                             : instr + offset;
    }
}

static __always_inline void override_pio_rx_program(PIO pio,
                                             const pio_program_t *program,
                                             const pio_program_t *debug_program,
                                             uint offset, int debug_pin) {
  if (debug_pin < 0) {
    override_pio_program(pio, program, offset);
  } else {
    override_pio_program(pio, debug_program, offset);
  }
}

static void
__no_inline_not_in_flash_func(configure_tx_program)(pio_port_t *pp,
                                                    root_port_t *port) {
  if (port->pinout == PIO_USB_PINOUT_DPDM) {
    pp->fs_tx_program = &usb_tx_dpdm_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dpdm_program;
    pp->ls_tx_program = &usb_tx_dmdp_program;
  } else {
    pp->fs_tx_program = &usb_tx_dmdp_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dmdp_program;
    pp->ls_tx_program = &usb_tx_dpdm_program;
  }
}

static void __no_inline_not_in_flash_func(configure_fullspeed_host)(
    pio_port_t *pp, root_port_t *port) {
  pp->low_speed = false;
  configure_tx_program(pp, port);
  pio_sm_clear_fifos(pp->pio_usb_tx, pp->sm_tx);
  override_pio_program(pp->pio_usb_tx, pp->fs_tx_program, pp->offset_tx);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);
  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);
  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_reset_instr);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dp);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);
}

static void __no_inline_not_in_flash_func(configure_lowspeed_host)(
    pio_port_t *pp, root_port_t *port) {
  pp->low_speed = true;
  configure_tx_program(pp, port);
  pio_sm_clear_fifos(pp->pio_usb_tx, pp->sm_tx);
  override_pio_program(pp->pio_usb_tx, pp->ls_tx_program, pp->offset_tx);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_ls_tx);
  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);
  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_reset_instr);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dm);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_ls_rx);
}

static void __no_inline_not_in_flash_func(configure_root_port)(
    pio_port_t *pp, root_port_t *root) {
  if (root->is_fullspeed) {
    configure_fullspeed_host(pp, root);
  } else {
    configure_lowspeed_host(pp, root);
  }
}

static void __no_inline_not_in_flash_func(restore_fs_bus)(pio_port_t *pp) {
  // change bus speed to full-speed
  pp->low_speed = false;
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
}

// Time about 1us ourselves so it lives in RAM.
static void __not_in_flash_func(busy_wait_1_us)(void) {
  uint32_t start = get_time_us_32();
  while (get_time_us_32() == start) {
      tight_loop_contents();
  }
}

static bool __no_inline_not_in_flash_func(connection_check)(root_port_t *port) {
  if (pio_usb_bus_get_line_state(port) == PORT_PIN_SE0) {
    busy_wait_1_us();

    if (pio_usb_bus_get_line_state(port) == PORT_PIN_SE0) {
      busy_wait_1_us();
      // device disconnect
      port->connected = false;
      port->suspended = true;
      port->ints |= PIO_USB_INTS_DISCONNECT_BITS;

      // failed/retired all queuing transfer in this root
      uint8_t root_idx = port - PIO_USB_ROOT_PORT(0);
      for (int ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++) {
        endpoint_t *ep = PIO_USB_ENDPOINT(ep_idx);
        if ((ep->root_idx == root_idx) && ep->size && ep->has_transfer) {
          pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
        }
      }

      return false;
    }
  }

  return true;
}

//--------------------------------------------------------------------+
// SOF
//--------------------------------------------------------------------+
static int usb_setup_transaction(pio_port_t *pp, endpoint_t *ep);
static int usb_in_transaction(pio_port_t *pp, endpoint_t *ep);
static int usb_out_transaction(pio_port_t *pp, endpoint_t *ep);

#if PIO_USB_ISO_RING_SIZE
static endpoint_t *iso_ring_ep;
static uint8_t iso_ring[PIO_USB_ISO_RING_SIZE];
static uint32_t iso_ring_head;
static uint32_t iso_ring_count;
static bool iso_ring_lost;
static uint8_t iso_ring_silent; // consecutive frames without a response

static void __no_inline_not_in_flash_func(iso_ring_copy_in)(uint8_t const *src, uint32_t len) {
  uint32_t const tail = (iso_ring_head + iso_ring_count) % PIO_USB_ISO_RING_SIZE;
  uint32_t first = PIO_USB_ISO_RING_SIZE - tail;
  if (first > len) {
    first = len;
  }
  memcpy(&iso_ring[tail], src, first);
  memcpy(iso_ring, src + first, len - first);
  iso_ring_count += len;
}

static void __no_inline_not_in_flash_func(iso_ring_copy_out)(uint8_t *dst, uint32_t len) {
  uint32_t first = PIO_USB_ISO_RING_SIZE - iso_ring_head;
  if (first > len) {
    first = len;
  }
  if (dst) {
    memcpy(dst, &iso_ring[iso_ring_head], first);
    memcpy(dst + first, iso_ring, len - first);
  }
  iso_ring_head = (iso_ring_head + len) % PIO_USB_ISO_RING_SIZE;
  iso_ring_count -= len;
}

// A record longer than what the ring holds means the ring is corrupted.
// Empty it and report a gap.
static bool __no_inline_not_in_flash_func(iso_ring_check_len)(uint32_t len) {
  if (len <= iso_ring_count) {
    return true;
  }
  iso_ring_head = 0;
  iso_ring_count = 0;
  iso_ring_lost = true;
  return false;
}

// Length of the record at the head of the ring, header included. A lost
// marker (0xffff) is a record of just its two header bytes.
static uint32_t __no_inline_not_in_flash_func(iso_ring_head_len)(void) {
  uint16_t const value = iso_ring[iso_ring_head] |
                         (iso_ring[(iso_ring_head + 1) % PIO_USB_ISO_RING_SIZE] << 8);
  return value == 0xffff ? 2 : value + 2u;
}

// Make room for a new record by dropping the oldest one, so the ring always
// holds the newest part of the stream when the application falls behind.
// One lost marker stays at the head to show where the gap is: it is put in
// front of the oldest surviving record, or kept there if already present.
static void __no_inline_not_in_flash_func(iso_ring_drop_oldest)(void) {
  if (iso_ring_count < 2 || !iso_ring_check_len(iso_ring_head_len())) {
    iso_ring_count = 0;
    iso_ring_lost = true;
    return;
  }
  if (iso_ring_head_len() == 2) {
    iso_ring_copy_out(NULL, 2); // the marker; put back below
    if (iso_ring_count < 2 || !iso_ring_check_len(iso_ring_head_len())) {
      return;
    }
  }
  iso_ring_copy_out(NULL, iso_ring_head_len());
  iso_ring_head = (iso_ring_head + PIO_USB_ISO_RING_SIZE - 2) % PIO_USB_ISO_RING_SIZE;
  iso_ring_count += 2;
  iso_ring[iso_ring_head] = 0xff;
  iso_ring[(iso_ring_head + 1) % PIO_USB_ISO_RING_SIZE] = 0xff;
}

static void __no_inline_not_in_flash_func(iso_ring_receive)(endpoint_t *ep, int len,
                                                           uint8_t pid, uint8_t const *data) {
  uint8_t const lost_marker[2] = {0xff, 0xff};
  if (len < 0) {
    if (pid == USB_PID_DATA0 || pid == USB_PID_DATA1) {
      iso_ring_lost = true; // packet arrived corrupted
    } else if (++iso_ring_silent >= 32) {
      // The device stopped streaming (alternate setting 0). Stop polling
      // until the application queues another transfer. A transfer pending
      // now fails on the next frame.
      iso_ring_ep = NULL;
    }
    len = 0;
  } else {
    iso_ring_silent = 0;
  }
  if (iso_ring_lost) {
    while (iso_ring_count + 2 > PIO_USB_ISO_RING_SIZE) {
      iso_ring_drop_oldest();
    }
    iso_ring_copy_in(lost_marker, 2);
    iso_ring_lost = false;
  }
  if (len > 0) {
    while (iso_ring_count + 2 + len > PIO_USB_ISO_RING_SIZE) {
      iso_ring_drop_oldest();
    }
    uint8_t const header[2] = {len & 0xff, len >> 8};
    iso_ring_copy_in(header, 2);
    iso_ring_copy_in(data, len);
  }

  if (!ep->transfer_started || !ep->has_transfer || ep->transfer_aborted) {
    return;
  }
  // Hand over as many whole records as fit in the transfer buffer.
  while (iso_ring_count >= 2) {
    uint32_t const rec_len = iso_ring_head_len();
    if (!iso_ring_check_len(rec_len)) {
      break;
    }
    if (rec_len > ep->total_len) {
      iso_ring_copy_out(NULL, rec_len); // can never fit; drop it
      iso_ring_lost = true;
      continue;
    }
    if (ep->actual_len + rec_len > ep->total_len) {
      break;
    }
    iso_ring_copy_out(ep->app_buf + ep->actual_len, rec_len);
    ep->actual_len += rec_len;
  }
  if (ep->actual_len > 0) {
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
  }
}
#endif

void __not_in_flash_func(pio_usb_host_frame)(void) {
  if (!timer_active) {
    return;
  }

  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  // Send SOF
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (!(root->initialized && root->connected && !root->suspended &&
          connection_check(root))) {
      continue;
    }
    configure_root_port(pp, root);
    if (root->is_fullspeed) {
      // Send SOF for full speed
      pio_usb_bus_usb_transfer(pp, sof_packet_encoded, sof_packet_encoded_len);
    } else {
      // Send Keep alive for low speed
      pio_usb_bus_usb_transfer(pp, keepalive_encoded, 1);
    }
  }

  // Carry out all queued endpoint transaction
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (!(root->initialized && root->connected && !root->suspended)) {
      continue;
    }

    configure_root_port(pp, root);

    for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT;
         ep_pool_idx++) {
      endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
      if ((ep->root_idx == root_idx) && ep->size) {
        bool const is_periodic = ((ep->attr & 0x03) == EP_ATTR_INTERRUPT);

        if (is_periodic && (ep->interval_counter > 0)) {
          ep->interval_counter--;
          continue;
        }

#if PIO_USB_ISO_RING_SIZE
        if ((ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS && (ep->ep_num & EP_IN) &&
            ep != iso_ring_ep) {
          if (ep->has_transfer && !ep->transfer_aborted) {
            pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
          }
          continue;
        }
        bool const live = ep->has_transfer && !ep->transfer_aborted;
        bool const active = live || ep == iso_ring_ep;
#else
        bool const live = ep->has_transfer && !ep->transfer_aborted;
        bool const active = live;
#endif
        if (active) {
          ep->transfer_started = live;

          if (ep->need_pre) {
            pp->need_pre = true;
          }

          if (ep->ep_num == 0 && ep->data_id == USB_PID_SETUP) {
            usb_setup_transaction(pp, ep);
          } else {
            if (ep->ep_num & EP_IN) {
              usb_in_transaction(pp, ep);
            } else {
              usb_out_transaction(pp, ep);
            }

            if (is_periodic) {
              ep->interval_counter = ep->interval - 1;
            }
          }

          if (ep->need_pre) {
            pp->need_pre = false;
            restore_fs_bus(pp);
          }

          ep->transfer_started = false;
        }
      }
    }
  }

  // check for new connection to root hub
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (root->initialized && !root->connected) {
      port_pin_status_t const line_state = pio_usb_bus_get_line_state(root);
      if (line_state == PORT_PIN_FS_IDLE || line_state == PORT_PIN_LS_IDLE) {
        root->is_fullspeed = (line_state == PORT_PIN_FS_IDLE);
        root->connected = true;
        root->suspended = true; // need a bus reset before operating
        root->ints |= PIO_USB_INTS_CONNECT_BITS;
      }
    }
  }

  // Invoke IRQHandler if interrupt status is set
  for (uint8_t root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    if (PIO_USB_ROOT_PORT(root_idx)->ints) {
      pio_usb_host_irq_handler(root_idx);
    }
  }

  sof_count++;

  // SOF counter is 11-bit
  uint16_t const sof_count_11b = sof_count & 0x7ff;
  sof_packet[2] = sof_count_11b & 0xff;
  sof_packet[3] = (calc_usb_crc5(sof_count_11b) << 3) | (sof_count_11b >> 8);
  sof_packet_encoded_len =
      pio_usb_ll_encode_tx_data(sof_packet, sizeof(sof_packet), sof_packet_encoded);
}

static bool __no_inline_not_in_flash_func(sof_timer)(repeating_timer_t *_rt) {
  (void)_rt;

  pio_usb_host_frame();

  return true;
}

//--------------------------------------------------------------------+
// Host Controller functions
//--------------------------------------------------------------------+

uint32_t pio_usb_host_get_frame_number(void) {
  return sof_count;
}

void pio_usb_host_port_reset_start(uint8_t root_idx) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);

  // bus is not operating while in reset
  root->suspended = true;

  // Force line state to SE0
  gpio_set_outover(root->pin_dp,  GPIO_OVERRIDE_LOW);
  gpio_set_outover(root->pin_dm,  GPIO_OVERRIDE_LOW);
  gpio_set_oeover(root->pin_dp,  GPIO_OVERRIDE_HIGH);
  gpio_set_oeover(root->pin_dm,  GPIO_OVERRIDE_HIGH);
}

void pio_usb_host_port_reset_end(uint8_t root_idx) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);

  // line state to input
  gpio_set_oeover(root->pin_dp,  GPIO_OVERRIDE_NORMAL);
  gpio_set_oeover(root->pin_dm,  GPIO_OVERRIDE_NORMAL);
  gpio_set_outover(root->pin_dp,  GPIO_OVERRIDE_NORMAL);
  gpio_set_outover(root->pin_dm,  GPIO_OVERRIDE_NORMAL);
  busy_wait_us(100); // TODO check if this is neccessary

  // bus back to operating
  root->suspended = false;
}

void pio_usb_host_close_device(uint8_t root_idx, uint8_t device_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    if ((ep->root_idx == root_idx) && (ep->dev_addr == device_address) &&
        ep->size) {
      ep->size = 0;
      ep->has_transfer = false;
#if PIO_USB_ISO_RING_SIZE
      if (ep == iso_ring_ep) {
        iso_ring_ep = NULL;
      }
#endif
    }
  }
}

static inline __force_inline endpoint_t * _find_ep(uint8_t root_idx, 
                                                   uint8_t device_address, uint8_t ep_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    // note 0x00 and 0x80 are matched as control endpoint of opposite direction
    if ((ep->root_idx == root_idx) && (ep->dev_addr == device_address) &&
        ep->size &&
        ((ep->ep_num == ep_address) ||
         (((ep_address & 0x7f) == 0) && ((ep->ep_num & 0x7f) == 0)))) {
      return ep;
    }
  }

  return NULL;
}

bool pio_usb_host_endpoint_open(uint8_t root_idx, uint8_t device_address,
                                uint8_t const *desc_endpoint, bool need_pre) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *)desc_endpoint;
  if (NULL != _find_ep(root_idx, device_address, d->epaddr)) {
    return true; // already opened
  }
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    // ep size is used as valid indicator
    if (ep->size == 0) {
      pio_usb_ll_configure_endpoint(ep, desc_endpoint);
      ep->root_idx = root_idx;
      ep->dev_addr = device_address;
      ep->need_pre = need_pre;
      ep->is_tx = (d->epaddr & 0x80) ? false : true; // host endpoint out is tx
      return true;
    }
  }

  return false;
}

bool pio_usb_host_endpoint_close(uint8_t root_idx, uint8_t device_address,
                                 uint8_t ep_address) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  if (!ep) {
    return false; // endpoint not opened
  }

  ep->size = 0; // mark as closed
#if PIO_USB_ISO_RING_SIZE
  if (ep == iso_ring_ep) {
    iso_ring_ep = NULL;
  }
#endif
  return true;
}

bool pio_usb_host_send_setup(uint8_t root_idx, uint8_t device_address,
                             uint8_t const setup_packet[8]) {
  endpoint_t *ep = _find_ep(root_idx, device_address, 0);
  if (!ep) {
    printf("cannot find ep 0x00\r\n");
    return false;
  }

  ep->ep_num = 0; // setup is is OUT
  ep->data_id = USB_PID_SETUP;
  ep->is_tx = true;

#if PIO_USB_ISO_RING_SIZE
  if (iso_ring_ep && iso_ring_ep->dev_addr == device_address &&
      (setup_packet[0] & 0x60) == 0 && (setup_packet[1] == 9 || setup_packet[1] == 11)) {
    iso_ring_ep = NULL;
  }
#endif

  return pio_usb_ll_transfer_start(ep, (uint8_t *)setup_packet, 8);
}

bool pio_usb_host_endpoint_transfer(uint8_t root_idx, uint8_t device_address,
                                    uint8_t ep_address, uint8_t *buffer,
                                    uint16_t buflen) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  if (!ep) {
    printf("no endpoint 0x%02X\r\n", ep_address);
    return false;
  }

  // Control endpoint, address may switch between 0x00 <-> 0x80
  // therefore we need to update ep_num and is_tx
  if ((ep_address & 0x7f) == 0) {
    ep->ep_num = ep_address;
    ep->is_tx = ep_address == 0;
    ep->data_id = 1; // data and status always start with DATA1
  }

#if PIO_USB_ISO_RING_SIZE
  if ((ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS && !ep->is_tx && ep != iso_ring_ep) {
    // Start receiving this endpoint's packets every frame. Only one
    // isochronous IN endpoint at a time uses the ring.
    iso_ring_ep = NULL;
    iso_ring_head = 0;
    iso_ring_count = 0;
    iso_ring_lost = false;
    iso_ring_silent = 0;
    iso_ring_ep = ep;
  }
#endif

  return pio_usb_ll_transfer_start(ep, buffer, buflen);
}

bool pio_usb_host_endpoint_abort_transfer(uint8_t root_idx, uint8_t device_address,
                                          uint8_t ep_address) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  if (!ep) {
    printf("no endpoint 0x%02X\r\n", ep_address);
    return false;
  }

  if (!ep->has_transfer) {
    return false; // no transfer to abort
  }

  // mark transfer as aborted
  ep->transfer_aborted = true;

  // Race potential: SOF timer can be called before transfer_aborted is actually set
  // and started the transfer. Wait 1 usb frame for transaction to complete.
  // On the next SOF timer, transfer_aborted will be checked and skipped
  for (int wait_ms = 0; wait_ms < 8 && ep->has_transfer && ep->transfer_started; wait_ms++) {
    busy_wait_ms(1);
  }

  // check if transfer is still active (could be completed)
  bool const still_active = ep->has_transfer;
  if (still_active) {
    ep->has_transfer = false;
  }
  ep->transfer_aborted = false;

  return still_active; // still active means transfer is successfully aborted
}

//--------------------------------------------------------------------+
// Transaction helper
//--------------------------------------------------------------------+
static void __no_inline_not_in_flash_func(end_transaction)(pio_port_t *pp) {
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
    pio_sm_exec(pp->pio_usb_rx, pp->sm_eop, pio_encode_jmp(pp->offset_eop));
  }
}

// Park the edge detector at its EOP wait and clear the RX flags while it is
// stopped. SM restart does not clear IRQ flags, so a DECODER_TRIGGER left by an
// unanswered transaction (our own SOF/token seen by the still armed detector)
// would otherwise shift a bit in before SYNC.
static void __no_inline_not_in_flash_func(park_receiver)(const pio_port_t *pp) {
  PIO pio = pp->pio_usb_rx;

  pio_sm_set_enabled(pio, pp->sm_eop, false);
  pio_sm_restart(pio, pp->sm_eop);
  pio_sm_exec(pio, pp->sm_eop, pio_encode_jmp(pp->offset_eop));
  pio->irq = IRQ_RX_ALL_MASK;

  pio_usb_bus_prepare_receive(pp);

  // Parks within one detector clock, long before our token has been sent
  pio_sm_set_enabled(pio, pp->sm_eop, true);
}

static int __no_inline_not_in_flash_func(usb_in_transaction)(pio_port_t *pp,
                                                             endpoint_t *ep) {
  int res = 0;
  uint8_t expect_pid = (ep->data_id == 1) ? USB_PID_DATA1 : USB_PID_DATA0;

  park_receiver(pp);
  pio_usb_bus_send_token(pp, USB_PID_IN, ep->dev_addr, ep->ep_num);

  bool const is_iso =
      PIO_USB_HOST_ISOCHRONOUS && (ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS;
  // Isochronous IN has no handshake and no data toggle
  int receive_len = pio_usb_bus_receive_packet_and_handshake(pp, is_iso ? 0 : USB_PID_ACK);
  uint8_t const receive_pid = pp->usb_rx_buffer[1];

  if (is_iso && receive_len > 0) {
    // Clamp to what usb_rx_buffer holds. A non-compliant or malicious device
    // could otherwise report a receive_len larger than the buffer, causing an
    // out-of-bounds memcpy.
    if ((uint16_t)receive_len > sizeof(pp->usb_rx_buffer) - 4) {
      receive_len = sizeof(pp->usb_rx_buffer) - 4;
    }
  }

#if PIO_USB_ISO_RING_SIZE
  if (is_iso) {
    if (ep == iso_ring_ep) {
      iso_ring_receive(ep, receive_len, receive_pid, &pp->usb_rx_buffer[2]);
    }
  } else
#endif
  if (is_iso) {
    // Every isochronous packet completes the transfer, even a full one, so
    // the caller sees packet boundaries. A bad or missing packet is not retried.
    if (receive_len >= 0) {
      uint16_t const room = ep->total_len - ep->actual_len;
      if (receive_len > room) {
        receive_len = room;
      }
      memcpy(ep->app_buf, &pp->usb_rx_buffer[2], receive_len);
      ep->actual_len += receive_len;
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
    } else {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }
  } else if (receive_len >= 0) {
    if (receive_pid == expect_pid) {
      // Clamp to the transaction length the host actually requested/allocated
      // a buffer for. A non-compliant or malicious device could otherwise
      // report a receive_len larger than ep->app_buf (or usb_rx_buffer),
      // causing an out-of-bounds memcpy.
      uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);
      if ((uint16_t)receive_len > xact_len) {
        receive_len = xact_len;
      }
      memcpy(ep->app_buf, &pp->usb_rx_buffer[2], receive_len);
      pio_usb_ll_transfer_continue(ep, receive_len);
    } else {
      // DATA0/1 mismatched, 0 for re-try next frame
    }
  } else if (receive_pid == USB_PID_NAK) {
    // NAK try again next frame
  } else if (receive_pid == USB_PID_STALL) {
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_STALLED_BITS);
  } else {
    res = -1;
    if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      res = -2;
    }

    if (++ep->failed_count >= TRANSACTION_MAX_RETRY) {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS); // failed after 3 consecutive retries
    }
  }

  if (res == 0) {
    ep->failed_count = 0; // reset failed count if we got a sound response
  }

  end_transaction(pp);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

static int __no_inline_not_in_flash_func(usb_out_transaction)(pio_port_t *pp,
                                                              endpoint_t *ep) {
  int res = 0;

  uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);

  park_receiver(pp);
  pio_usb_bus_send_token(pp, USB_PID_OUT, ep->dev_addr, ep->ep_num);

  pio_usb_bus_usb_transfer_arm_rx(pp, ep->buffer, ep->encoded_data_len);

  pio_usb_bus_wait_handshake(pp);
  end_transaction(pp);

  uint8_t const receive_token = pp->usb_rx_buffer[1];

  if (receive_token == USB_PID_ACK) {
    pio_usb_ll_transfer_continue(ep, xact_len);
  } else if (receive_token == USB_PID_NAK) {
    // NAK try again next frame
  } else if (receive_token == USB_PID_STALL) {
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_STALLED_BITS);
  } else {
    res = -1;
    if (++ep->failed_count >= TRANSACTION_MAX_RETRY) {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }
  }

  if (res == 0) {
    ep->failed_count = 0;// reset failed count if we got a sound response
  }

  end_transaction(pp);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

static int __no_inline_not_in_flash_func(usb_setup_transaction)(
    pio_port_t *pp,  endpoint_t *ep) {
  int res = 0;

  // Setup token
  park_receiver(pp);
  pio_usb_bus_send_token(pp, USB_PID_SETUP, ep->dev_addr, 0);

  // Data
  ep->data_id = 0; // set to DATA0
  pio_usb_bus_usb_transfer_arm_rx(pp, ep->buffer, ep->encoded_data_len);

  // Handshake
  const uint8_t handshake = pio_usb_bus_wait_handshake(pp);
  end_transaction(pp);

  if (handshake == USB_PID_ACK) {
    ep->actual_len = 8;
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
  } else {
    res = -1;
    ep->data_id = USB_PID_SETUP; // retry setup
    if (++ep->failed_count >= TRANSACTION_MAX_RETRY) {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }
  }

  if (res == 0) {
    ep->failed_count = 0;// reset failed count if we got a sound response
  }

  pp->usb_rx_buffer[1] = 0; // reset buffer

  return res;
}


static void __no_inline_not_in_flash_func(handle_endpoint_irq)(
    root_port_t *root, uint32_t flag, volatile uint32_t *ep_reg) {
  (void)root;
  const uint32_t ep_all = *ep_reg;

  for (uint8_t ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++) {
    if (ep_all & (1u << ep_idx)) {
      endpoint_t *ep = PIO_USB_ENDPOINT(ep_idx);
      usb_device_t *device = NULL;

      // find device this endpoint belongs to
      for (int idx = 0; idx < PIO_USB_DEVICE_CNT; idx++) {
        usb_device_t *dev = &pio_usb_device[idx];
        if (dev->connected && (ep->dev_addr == dev->address)) {
          device = dev;
          break;
        }
      }

      if (device) {
        // control endpoint is either 0x00 or 0x80
        if ((ep->ep_num & 0x7f) == 0) {
          control_pipe_t *pipe = &device->control_pipe;

          if (flag != PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
            pipe->stage = STAGE_SETUP;
            pipe->operation = CONTROL_ERROR;
          } else {
            ep->data_id = 1; // both data and status have DATA1
            if (pipe->stage == STAGE_SETUP) {
              if (pipe->operation == CONTROL_IN) {
                pipe->stage = STAGE_IN;
                ep->ep_num = 0x80;
                ep->is_tx = false;
                pio_usb_ll_transfer_start(ep,
                                          (uint8_t *)(uintptr_t)pipe->rx_buffer,
                                          pipe->request_length);
              } else if (pipe->operation == CONTROL_OUT) {
                if (pipe->out_data_packet.tx_address != NULL) {
                  pipe->stage = STAGE_OUT;
                  ep->ep_num = 0x00;
                  ep->is_tx = true;
                  pio_usb_ll_transfer_start(ep,
                                            pipe->out_data_packet.tx_address,
                                            pipe->out_data_packet.tx_length);
                } else {
                  pipe->stage = STAGE_STATUS;
                  ep->ep_num = 0x80;
                  ep->is_tx = false;
                  pio_usb_ll_transfer_start(ep, NULL, 0);
                }
              }
            } else if (pipe->stage == STAGE_IN) {
              pipe->stage = STAGE_STATUS;
              ep->ep_num = 0x00;
              ep->is_tx = true;
              pio_usb_ll_transfer_start(ep, NULL, 0);
            } else if (pipe->stage == STAGE_OUT) {
              pipe->stage = STAGE_STATUS;
              ep->ep_num = 0x80;
              ep->is_tx = false;
              pio_usb_ll_transfer_start(ep, NULL, 0);
            } else if (pipe->stage == STAGE_STATUS) {
              pipe->stage = STAGE_SETUP;
              pipe->operation = CONTROL_COMPLETE;
            }
          }
        } else if (device->device_class == CLASS_HUB && (ep->ep_num & EP_IN)) {
          // hub interrupt endpoint
          device->event = EVENT_HUB_PORT_CHANGE;
        }
      }
    }
  }

  // clear all
  (*ep_reg) &= ~ep_all;
}

// IRQ Handler
static void __no_inline_not_in_flash_func(__pio_usb_host_irq_handler)(uint8_t root_id) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_id);
  uint32_t const ints = root->ints;

  if (ints & PIO_USB_INTS_CONNECT_BITS) {
    root->event = EVENT_CONNECT;
  }

  if (ints & PIO_USB_INTS_DISCONNECT_BITS) {
    root->event = EVENT_DISCONNECT;
  }

  if (ints & PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    handle_endpoint_irq(root, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS,
                        &root->ep_complete);
  }

  if (ints & PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    handle_endpoint_irq(root, PIO_USB_INTS_ENDPOINT_STALLED_BITS,
                        &root->ep_stalled);
  }

  if (ints & PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    handle_endpoint_irq(root, PIO_USB_INTS_ENDPOINT_ERROR_BITS,
                        &root->ep_error);
  }

  // clear all
  root->ints &= ~ints;
}

// weak alias to __pio_usb_host_irq_handler
void pio_usb_host_irq_handler(uint8_t root_id) __attribute__ ((weak, alias("__pio_usb_host_irq_handler")));

#pragma GCC pop_options
