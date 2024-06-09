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

#include "pio_usb.h"
#include "pio_usb_ll.h"
#include "usb_crc.h"

#include "usb_rx.pio.h"
#include "usb_tx.pio.h"

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

static uint8_t new_devaddr = 0;
static uint8_t ep0_crc5_lut[16];
static __unused usb_descriptor_buffers_t descriptor_buffers;

static uint8_t nak_encoded[5];
static uint8_t stall_encoded[5];

static void __no_inline_not_in_flash_func(update_ep0_crc5_lut)(uint8_t addr) {
  uint16_t dat;
  uint8_t crc;

  for (int epnum = 0; epnum < 16; epnum++) {
    dat = (addr) | (epnum << 7);
    crc = calc_usb_crc5(dat);
    ep0_crc5_lut[epnum] = (crc << 3) | ((epnum >> 1) & 0x07);
  }
}

static __always_inline void restart_usb_receiver(pio_port_t *pp) {
  pio_sm_exec(pp->pio_usb_rx, pp->sm_rx, pp->rx_reset_instr2);
  pio_sm_restart(pp->pio_usb_rx, pp->sm_rx);
  pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
}

static __always_inline uint8_t device_receive_token(void) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  uint8_t idx = 0;
  uint8_t buffer[2];

  if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
    while ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      if (pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
        buffer[idx++] = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
        if (idx == 2) {
          return buffer[1];
        }
      }
    }
  } else {
    // host is probably timeout. Ignore this packets.
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
  }

  return 0;
}

static __always_inline int8_t device_receive_ep_address(uint8_t token,
                                                        uint8_t dev_addr) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  uint8_t idx = 0;
  uint8_t addr;
  uint8_t ep;
  uint8_t ep_num = 0;
  uint8_t buffer[3];
  bool match = false;

  static uint8_t eplut[2][8] = {{0, 2, 4, 6, 8, 10, 12, 14},
                                {1, 3, 5, 7, 9, 11, 13, 15}};
  uint8_t *current_lut;

  if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
    while ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      if (pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
        buffer[idx++] = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
        if ((idx == 1) && (token != USB_PID_SOF)) {
          addr = buffer[0] & 0x7f;
          current_lut = &eplut[buffer[0] >> 7][0];
          match = dev_addr == addr ? true : false;
        } else if (idx == 2) {
          ep_num = buffer[1];
          break;
        }
      }
    }
  } else {
    // host is probably timeout. Ignore this packets.
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
  }

  if (match) {
    ep = current_lut[ep_num & 0x07];
    if (ep0_crc5_lut[ep] == ep_num) {
      return ep;
    } else {
      return -1;
    }
  }

  return -1;
}

static __always_inline void wait_receive_complete(pio_port_t *pp) {
  while ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
    continue;
  }
  pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
}

static void __no_inline_not_in_flash_func(usb_device_packet_handler)(void) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t *rport = PIO_USB_ROOT_PORT(0);

  gpio_clr_mask(1<<3);
  //
  // time critical start
  //
  uint8_t addr = rport->dev_addr;
  uint8_t token = device_receive_token();

  if (token == USB_PID_IN) {
    int8_t ep_num = device_receive_ep_address(token, addr);
    if (ep_num < 0) {
      gpio_set_mask(1 << 3);
      return;
    }

    endpoint_t *ep = PIO_USB_ENDPOINT((ep_num << 1) | 0x01);

    pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_start_instr);
    volatile bool has_transfer = ep->has_transfer;

    if (has_transfer) {
      dma_channel_transfer_from_buffer_now(pp->tx_ch, ep->buffer, ep->encoded_data_len);
    } else if (ep->stalled) {
      dma_channel_transfer_from_buffer_now(pp->tx_ch, stall_encoded, sizeof(stall_encoded));
    } else {
      dma_channel_transfer_from_buffer_now(pp->tx_ch, nak_encoded, sizeof(nak_encoded));
    }

    pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK; // clear complete flag
    while ((pp->pio_usb_tx->irq & IRQ_TX_ALL_MASK) == 0) {
      continue;
    }

    if (has_transfer) {
      pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
      irq_clear(pp->device_rx_irq_num);
      pio_usb_bus_start_receive(pp);

      // wait for ack
      pio_usb_bus_wait_handshake(pp);

      pio_usb_bus_start_receive(pp);
      irq_clear(pp->device_rx_irq_num);

      //
      // time critical end
      //

      if (ep->ep_num == 0x80 && new_devaddr > 0) {
        rport->dev_addr = new_devaddr;
        new_devaddr = 0;
        update_ep0_crc5_lut(rport->dev_addr);
      }

      rport->ints |= PIO_USB_INTS_ENDPOINT_CONTINUE_BITS;
      rport->ep_continue |= (1 << ep_num);
    } else {
      pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
      irq_clear(pp->device_rx_irq_num);
      restart_usb_receiver(pp);
      pio_usb_bus_start_receive(pp);

      //
      // time critical end
      //
    }
  } else if (token == USB_PID_OUT) {
    int8_t ep_num = device_receive_ep_address(token, addr);
    wait_receive_complete(pp);
    restart_usb_receiver(pp);
    if (ep_num < 0) {
      return;
    }
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_num << 1);

  gpio_clr_mask(1<<4);
    uint8_t hanshake = ep->stalled
                           ? USB_PID_STALL
                           : (ep->has_transfer ? USB_PID_ACK : USB_PID_NAK);
    int res = pio_usb_bus_receive_packet_and_handshake(pp, hanshake);
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
    restart_usb_receiver(pp);
    pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
    irq_clear(pp->device_rx_irq_num);

    if (ep->has_transfer) {
      if (res >= 0) {
        memcpy(ep->app_buf, pp->usb_rx_buffer + 2, res);
        pio_usb_ll_transfer_continue(ep, res);
      }
    }
  } else if (token == USB_PID_SETUP) {
    int8_t ep_num = device_receive_ep_address(token, addr);
    restart_usb_receiver(pp);
    if (ep_num < 0) {
      return;
    }
    int res = pio_usb_bus_receive_packet_and_handshake(pp, USB_PID_ACK);
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
    restart_usb_receiver(pp);
    pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
    irq_clear(pp->device_rx_irq_num);

    if (res >= 0) {
      rport->setup_packet = pp->usb_rx_buffer + 2;
      rport->ints |= PIO_USB_INTS_SETUP_REQ_BITS;

      // DATA1 for both data and status stage
      PIO_USB_ENDPOINT(0)->has_transfer = PIO_USB_ENDPOINT(1)->has_transfer = false;
      PIO_USB_ENDPOINT(0)->data_id = PIO_USB_ENDPOINT(1)->data_id = 1;
      PIO_USB_ENDPOINT(0)->stalled = PIO_USB_ENDPOINT(1)->stalled = false;
    }
  } else if (token == USB_PID_SOF) {
    // SOF interrupt
    device_receive_ep_address(token, addr);
    wait_receive_complete(pp);
    restart_usb_receiver(pp);
  } else {
    device_receive_ep_address(token, addr);
    wait_receive_complete(pp);
    restart_usb_receiver(pp);
  }
}

usb_device_t *pio_usb_device_init(const pio_usb_configuration_t *c,
                                  const usb_descriptor_buffers_t *buffers) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t *rport = PIO_USB_ROOT_PORT(0);
  usb_device_t *dev = &pio_usb_device[0];

  pio_usb_bus_init(pp, c, rport);
  gpio_disable_pulls(rport->pin_dp); // needs external pull-up 
  rport->mode = PIO_USB_MODE_DEVICE;

  memset(dev, 0, sizeof(*dev));
  for (int i = 0; i < PIO_USB_DEV_EP_CNT; i++) {
    dev->endpoint_id[i] = 2 * (i + 1); // only index IN endpoint
  }

  update_ep0_crc5_lut(rport->dev_addr);

  float const cpu_freq = (float)clock_get_hz(clk_sys);

  pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, rport->pin_dp);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, rport->pin_dm);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, rport->pin_dp);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);

  descriptor_buffers = *buffers;

  pio_usb_bus_prepare_receive(pp);

  // configure PIOx_IRQ_0 to detect packet receive start
  pio_set_irqn_source_enabled(pp->pio_usb_rx, 0, pis_interrupt0 + IRQ_RX_START,
                              true);
  pp->device_rx_irq_num = (pp->pio_usb_rx == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
  irq_set_exclusive_handler(pp->device_rx_irq_num, usb_device_packet_handler);
  irq_set_enabled(pp->device_rx_irq_num, true);


  // pre-encode handshake packets
  uint8_t raw_packet[] = {USB_SYNC, USB_PID_NAK};
  pio_usb_ll_encode_tx_data(raw_packet, 2, nak_encoded);
  raw_packet[1] = USB_PID_STALL;
  pio_usb_ll_encode_tx_data(raw_packet, 2, stall_encoded);

  return dev;
}

//--------------------------------------------------------------------+
// Device Controller functions
//--------------------------------------------------------------------+

void pio_usb_device_set_address(uint8_t dev_addr) {
  new_devaddr = dev_addr;
}

bool __no_inline_not_in_flash_func(pio_usb_device_endpoint_open)(
    uint8_t const *desc_endpoint) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *)desc_endpoint;
  endpoint_t *ep = pio_usb_device_get_endpoint_by_address(d->epaddr);

  pio_usb_ll_configure_endpoint(ep, desc_endpoint);
  ep->root_idx = 0;
  ep->dev_addr = 0; // not used
  ep->need_pre = 0;
  ep->is_tx = (d->epaddr & 0x80) ? true : false; // device: endpoint in is tx

  return true;
}

bool pio_usb_device_transfer(uint8_t ep_address, uint8_t *buffer,
                             uint16_t buflen) {
  endpoint_t *ep = pio_usb_device_get_endpoint_by_address(ep_address);
  return pio_usb_ll_transfer_start(ep, buffer, buflen);
}

//--------------------------------------------------------------------+
// USB Device Stack
//--------------------------------------------------------------------+
static int8_t ep0_desc_request_type = -1;
static uint16_t ep0_desc_request_len;
static uint8_t ep0_desc_request_idx;

static void __no_inline_not_in_flash_func(prepare_ep0_data)(uint8_t *data,
                                                            uint8_t len) {
  // 0: control out (rx), 1 : control in (tx)
  endpoint_t *ep = &pio_usb_ep_pool[1];

  pio_usb_ll_transfer_start(ep, data, len);

  if (len) {
    // there is data, prepare for status as well
    pio_usb_ll_transfer_start(&pio_usb_ep_pool[0], NULL, 0);
  }
}

static void __no_inline_not_in_flash_func(prepare_ep0_rx)(uint8_t *data,
                                                          uint8_t len) {
  // 0: control out (rx), 1 : control in (tx)
  endpoint_t *ep = &pio_usb_ep_pool[0];

  pio_usb_ll_transfer_start(ep, data, len);

  if (len) {
    // there is data, prepare for status as well
    pio_usb_ll_transfer_start(&pio_usb_ep_pool[1], NULL, 0);
  }
}

void pio_usb_device_task(void) {
  root_port_t *rport = PIO_USB_ROOT_PORT(0);
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  if (rport->ints) {
    pio_usb_device_irq_handler(0);
  }

  switch (ep0_desc_request_type) {
    case DESC_TYPE_CONFIG: {
      uint16_t req_len = ep0_desc_request_len;
      uint16_t desc_len =
          descriptor_buffers.config[2] | (descriptor_buffers.config[3] << 8);
      req_len = req_len > desc_len ? desc_len : req_len;
      prepare_ep0_data((uint8_t *)descriptor_buffers.config, req_len);
      ep0_desc_request_type = -1;
    } break;
    case DESC_TYPE_STRING: {
      const uint16_t *str =
          (uint16_t *)&descriptor_buffers.string[ep0_desc_request_idx];
      prepare_ep0_data((uint8_t *)str, str[0] & 0xff);
      ep0_desc_request_type = -1;
    } break;
    case DESC_TYPE_HID_REPORT:{
      prepare_ep0_data(
          (uint8_t *)descriptor_buffers.hid_report[ep0_desc_request_idx],
          ep0_desc_request_len);
      ep0_desc_request_type = -1;
    }
    default:
      break;
  }

  uint32_t se0_time_us =0;
  bool reset = false;
  while (pio_usb_bus_get_line_state(rport) == PORT_PIN_SE0) {
    busy_wait_us_32(1);
    se0_time_us++;

    if (se0_time_us == 1000) {
      memset(pio_usb_ep_pool, 0, sizeof(pio_usb_ep_pool));
      rport->dev_addr = 0;
      update_ep0_crc5_lut(rport->dev_addr);

      // init endpoint control in/out
      PIO_USB_ENDPOINT(0)->size = 64;
      PIO_USB_ENDPOINT(0)->ep_num = 0;
      PIO_USB_ENDPOINT(0)->is_tx = false;

      PIO_USB_ENDPOINT(1)->size = 64;
      PIO_USB_ENDPOINT(1)->ep_num = 0x80;
      PIO_USB_ENDPOINT(1)->is_tx = true;

      // TODO should be reset end, this is reset start only
      rport->ep_complete = rport->ep_stalled = rport->ep_error = 0;
      rport->ints |= PIO_USB_INTS_RESET_END_BITS;
      reset = true;
    }
  }
  if (reset) {
    restart_usb_receiver(pp);
    pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
    pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
  }
}

static void __no_inline_not_in_flash_func(configure_all_endpoints)(uint8_t const *desc) {
  uint8_t const *desc_end = desc + (descriptor_buffers.config[2] |
                                    (descriptor_buffers.config[3] << 8));
  while (desc < desc_end) {
    if (desc[1] == DESC_TYPE_ENDPOINT) {
      pio_usb_device_endpoint_open(desc);
    }
    desc += desc[0];
  }
}

static int __no_inline_not_in_flash_func(process_device_setup_stage)(uint8_t *buffer) {
  int res = -1;
  const usb_setup_packet_t *packet = (usb_setup_packet_t *)buffer;

  if (packet->request_type == USB_REQ_DIR_IN) {
    if (packet->request == 0x06) {
      if (packet->value_msb == DESC_TYPE_DEVICE) {
        prepare_ep0_data((uint8_t *)descriptor_buffers.device, 18);
        res = 0;
      } else if (packet->value_msb == DESC_TYPE_CONFIG) {
        ep0_desc_request_len = (packet->length_lsb | (packet->length_msb << 8));
        ep0_desc_request_type = DESC_TYPE_CONFIG;
        res = 0;
      } else if (packet->value_msb == DESC_TYPE_STRING) {
        if (descriptor_buffers.string != NULL) {
          ep0_desc_request_idx = packet->value_lsb;
          ep0_desc_request_type = DESC_TYPE_STRING;
          res = 0;
        }
      }
    }
  } else if (packet->request_type == USB_REQ_DIR_OUT) {
    if (packet->request == 0x05) {
      // set address
      new_devaddr = packet->value_lsb;
      prepare_ep0_data(NULL, 0);
      res = 0;
    } else if (packet->request == 0x09) {
      // set configuration
      configure_all_endpoints(descriptor_buffers.config);
      prepare_ep0_data(NULL, 0);
      res = 0;
    }
  } else if (packet->request_type == (USB_REQ_DIR_IN | USB_REQ_REC_IFACE)) {
    if (packet->request == 0x06 && packet->value_msb == DESC_TYPE_HID_REPORT) {
      // get hid report desc
      ep0_desc_request_len = (packet->length_lsb | (packet->length_msb << 8));
      ep0_desc_request_idx = packet->index_lsb;
      ep0_desc_request_type = DESC_TYPE_HID_REPORT;
      res = 0;
    }
  } else if (packet->request_type == (USB_REQ_TYP_CLASS | USB_REQ_REC_IFACE)) {
    if (packet->request == 0x09) {
      // set hid report
      static __unused uint8_t received_hid_report[8]; // not used
      prepare_ep0_rx(received_hid_report, 8);
      res = 0;
    } else if (packet->request == 0x0A) {
      // set hid idle request
      prepare_ep0_data(NULL, 0);
      res = 0;
    } else if (packet->request == 0x0B) {
      // set hid protocol request
      prepare_ep0_data(NULL, 0);
      res = 0;
    }
  } else if (packet->request_type == (USB_REQ_REC_EP)) {
      prepare_ep0_data(NULL, 0);
      res = 0;
  }

  return res;
}

// IRQ Handler
static void __no_inline_not_in_flash_func(__pio_usb_device_irq_handler)(uint8_t root_idx) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
  usb_device_t *dev = &pio_usb_device[0];

  uint32_t const ints = root->ints;

  if (ints & PIO_USB_INTS_RESET_END_BITS) {
    memset(dev, 0, sizeof(*dev));
    for (int i = 0; i < PIO_USB_DEV_EP_CNT; i++) {
      dev->endpoint_id[i] = 2 * (i + 1); // only index IN endpoint
    }
  }

  if (ints & PIO_USB_INTS_SETUP_REQ_BITS) {
    process_device_setup_stage(root->setup_packet);
    dev->control_pipe.stage = STAGE_DATA;
  }

  if (ints & PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    const uint32_t ep_all = root->ep_complete;

    // control out
    if (ep_all & 0x01) {
      if (dev->control_pipe.stage == STAGE_STATUS) {
        dev->control_pipe.stage = STAGE_COMPLETE;
      } else if (dev->control_pipe.stage == STAGE_DATA) {
        dev->control_pipe.stage = STAGE_STATUS;
        prepare_ep0_data(NULL, 0);
      }
    }

    // control in
    if (ep_all & 0x02) {
      if (dev->control_pipe.stage == STAGE_STATUS) {
        dev->control_pipe.stage = STAGE_COMPLETE;
      }
    }

    // clear all
    root->ep_complete &= ~ep_all;
  }

  if (ints & PIO_USB_INTS_ENDPOINT_CONTINUE_BITS) {
    for (int b = 0; b < 16; b++) {
      if (root->ep_continue & (1 << b)) {
        endpoint_t *ep = PIO_USB_ENDPOINT((b << 1) | 0x01);
        uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);
        pio_usb_ll_transfer_continue(ep, xact_len);
        root->ep_continue &= ~(1 << b);
      }
    }
  }

  // clear all
  root->ints &= ~ints;
}

// weak alias to __pio_usb_device_irq_handler
void pio_usb_device_irq_handler(uint8_t root_id) __attribute__ ((weak, alias("__pio_usb_device_irq_handler")));

#pragma GCC pop_options
