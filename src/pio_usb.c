/**
 * Copyright (c) 2021 sekigon-gonnoc
 */

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdio.h>
#include <stdint.h>
#include <string.h> // memcpy

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/platform.h"

#include "pio_usb.h"
#include "usb_definitions.h"
#include "pio_usb_configuration.h"
#include "pio_usb_ll.h"
#include "usb_crc.h"
#if defined(USB_PACKET_CAPTURE_DEBUG) && USB_PACKET_CAPTURE_DEBUG
#include "usb_capture.h"
#define USB_CAPTURE_SEQUENCE_START(pp) usb_capture_sequence_start((pp)->low_speed)
#else
#define USB_CAPTURE_SEQUENCE_START(pp) ((void)0)
#endif

#define UNUSED_PARAMETER(x) (void)x

usb_device_t pio_usb_device[PIO_USB_DEVICE_CNT];
pio_port_t pio_port[1];
root_port_t pio_usb_root_port[PIO_USB_ROOT_PORT_CNT];
endpoint_t pio_usb_ep_pool[PIO_USB_EP_POOL_CNT];

static uint8_t ack_encoded[5];
static uint32_t ack_tail_words[sizeof(ack_encoded)];
static uint8_t nak_encoded[5];
static uint8_t stall_encoded[5];
static uint8_t pre_encoded[5];

//--------------------------------------------------------------------+
// Bus functions
//--------------------------------------------------------------------+

static void __no_inline_not_in_flash_func(send_pre)(pio_port_t *pp) {
  // send PRE token in full-speed
  pp->low_speed = false;
  uint16_t instr = pp->fs_tx_pre_program->instructions[0];
  pp->pio_usb_tx->instr_mem[pp->offset_tx] = instr;

  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);

  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_start_instr);
  pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK;       // clear complete flag
  dma_channel_transfer_from_buffer_now(pp->tx_ch, pre_encoded,
                                       sizeof(pre_encoded));

  while ((pp->pio_usb_tx->irq & IRQ_TX_EOP_MASK) == 0) {
    continue;
  }
  // Wait for complete transmission of the PRE packet. We don't want to
  // accidentally send trailing Ks in low speed mode due to an early start
  // instruction that re-enables the outputs.
  uint32_t stall_mask = 1 << (PIO_FDEBUG_TXSTALL_LSB + pp->sm_tx);
  pp->pio_usb_tx->fdebug = stall_mask; // clear sticky stall mask bit
  while (!(pp->pio_usb_tx->fdebug & stall_mask)) {
    continue;
  }

  // change bus speed to low-speed
  pp->low_speed = true;
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  instr = pp->fs_tx_program->instructions[0];
  pp->pio_usb_tx->instr_mem[pp->offset_tx] = instr;
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_ls_tx);
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);

  // pio_sm_clear_fifos(pp->pio_usb_tx, pp->sm_tx);
  // pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_start_instr);
  // SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_ls_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
}

static_assert(IRQ_TX_EOP == 0, "slot 0 patch needs IRQ_TX_EOP == 0");

static __always_inline void usb_transfer(pio_port_t *pp, uint8_t *data,
                                         uint16_t len, bool arm_rx) {
  if (pp->need_pre) {
    send_pre(pp);
  }

  // Bounds the time from the packet start to the slot 0 patch below
  uint32_t irq_status = 0;
  if (arm_rx) {
    irq_status = save_and_disable_interrupts();
  }

  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_start_instr);
  dma_channel_transfer_from_buffer_now(pp->tx_ch, data, len);
  pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK; // clear complete flag

  bool hw_arm = false;
  if (arm_rx) {
    __compiler_memory_barrier(); // keep the checks below after the start
    // IRQ flags are local to a PIO block
    hw_arm = pp->pio_usb_tx == pp->pio_usb_rx;
    if (hw_arm) {
      // Slot 0 runs only for the EOP, a whole packet later, so patching it
      // after the start keeps it out of the SETUP/OUT token-to-DATA gap.
      // irq IRQ_TX_EOP becomes irq clear IRQ_RX_EOP. The fs and ls programs
      // encode this slot identically, as send_pre() also relies on.
      pp->pio_usb_tx->instr_mem[pp->offset_tx] =
          pp->fs_tx_program->instructions[0] | pio_encode_irq_clear(false, IRQ_RX_EOP);
    }
    restore_interrupts(irq_status);
  }

  // After COMP the trailing K padding ends in an OUT stall at PC1, so a CPU
  // that misses the brief COMP window would poll the PC forever. Once DMA has
  // queued every byte, a fresh stall can only follow COMP, not a DMA gap.
  // Done before the EOP wait to keep it out of the COMP window.
  while (dma_channel_is_busy(pp->tx_ch)) {
    continue;
  }
  uint32_t const stall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + pp->sm_tx);
  pp->pio_usb_tx->fdebug = stall_mask;

  // LS also waits out the EOP's J for the 2-bit inter-packet delay
  uint32_t const done_pc = PIO_USB_TX_ENCODED_DATA_COMP + (pp->low_speed ? 1 : 0);
  io_ro_32 *pc = &pp->pio_usb_tx->sm[pp->sm_tx].addr;
  if (hw_arm) {
    // A fast reply may already have parked the detector again, but its START
    // still shows that our EOP was sent
    while ((pp->pio_usb_rx->irq & (IRQ_RX_COMP_MASK | IRQ_RX_START_MASK)) ==
           IRQ_RX_COMP_MASK) {
      continue;
    }
    pp->pio_usb_tx->instr_mem[pp->offset_tx] = pp->fs_tx_program->instructions[0];
  } else {
    // An interrupt between the start and the clear above that outlasts the
    // packet erases its EOP flag; the post-COMP stall still ends the wait
    while ((pp->pio_usb_tx->irq & IRQ_TX_ALL_MASK) == 0 &&
           !(pp->pio_usb_tx->fdebug & stall_mask)) {
      continue;
    }
    pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK; // clear complete flag
  }

  while (*pc < done_pc && !(pp->pio_usb_tx->fdebug & stall_mask)) {
    continue;
  }

  if (arm_rx && !hw_arm) {
    // The other RX flags were cleared while the detector was parked
    pp->pio_usb_rx->irq = IRQ_RX_COMP_MASK;
  }
}

void __not_in_flash_func(pio_usb_bus_usb_transfer)(pio_port_t *pp,
                                              uint8_t *data, uint16_t len) {
  usb_transfer(pp, data, len, false);
}

void __not_in_flash_func(pio_usb_bus_usb_transfer_arm_rx)(pio_port_t *pp,
                                                     uint8_t *data, uint16_t len) {
  usb_transfer(pp, data, len, true);
}

static __always_inline void wait_for_fs_ack_end(PIO pio, pio_sm_hw_t *tx_sm,
                                                uint32_t stall_mask) {
  while ((pio->irq & IRQ_TX_ALL_MASK) == 0) {
    continue;
  }
  pio->irq = IRQ_TX_ALL_MASK;
  while (tx_sm->addr < PIO_USB_TX_ENCODED_DATA_COMP &&
         !(pio->fdebug & stall_mask)) {
    continue;
  }
}

static bool __no_inline_not_in_flash_func(prepare_fs_ack)(pio_port_t *pp,
                                                          uint32_t stall_mask) {
  PIO pio = pp->pio_usb_tx;
  uint sm = pp->sm_tx;
  // Wait until the IN token's COMP and padding have drained. RX remains armed.
  if (!(pio->fdebug & stall_mask)) {
    const uint32_t start = get_time_us_32();
    do {
      if (get_time_us_32() - start > 1) return false;
    } while (!(pio->fdebug & stall_mask));
  }
  pio_sm_set_enabled(pio, sm, false);
  const uint16_t start_j = pp->fs_tx_program->instructions[4];
  const uint16_t start_k = start_j ^ pio_encode_sideset(2, 3);
  pio->instr_mem[pp->offset_tx + 4] = start_k | pio_encode_delay(3);
  pio_sm_exec(pio, sm, pp->tx_start_instr);
  for (uint i = 0; i < sizeof(ack_encoded); i++) {
    pio_sm_put(pio, sm, ack_tail_words[i]);
  }
  pio->irq = IRQ_TX_ALL_MASK;
  pio->fdebug = stall_mask;
  return true;
}

static void __no_inline_not_in_flash_func(restore_fs_tx)(pio_port_t *pp, bool sent) {
  PIO pio = pp->pio_usb_tx;
  pio->instr_mem[pp->offset_tx + 4] = pp->fs_tx_program->instructions[4];
  if (!sent) {
    pio_sm_clear_fifos(pio, pp->sm_tx);
    // Park on the J-side OUT with the pins still released.
    pio_sm_exec(pio, pp->sm_tx, pio_encode_jmp(pp->offset_tx + 1) |
                              (pp->tx_start_instr & pio_encode_sideset(2, 3)));
    pio_sm_set_enabled(pio, pp->sm_tx, true);
  }
}

static __always_inline bool wait_for_fs_ack_start(uint32_t bus_mask, uint32_t idle_j) {
  // The PIO flags EOP during SE0. GPIO inputs have the RX invert override.
  if ((gpio_get_all() & bus_mask) != idle_j) {
    const uint32_t start = get_time_us_32();
    do {
      if (get_time_us_32() - start > 1) return false;
    } while ((gpio_get_all() & bus_mask) != idle_j);
  }
  return true;
}

void __no_inline_not_in_flash_func(pio_usb_bus_send_token)(pio_port_t *pp,
                                                           uint8_t token,
                                                           uint8_t addr,
                                                           uint8_t ep_num) {

  uint8_t packet[4] = {USB_SYNC, token, 0, 0};
  uint16_t dat = ((uint16_t)(ep_num & 0xf) << 7) | (addr & 0x7f);
  uint8_t crc = calc_usb_crc5(dat);
  packet[2] = dat & 0xff;
  packet[3] = (crc << 3) | ((dat >> 8) & 0x1f);

  uint8_t packet_encoded[sizeof(packet) * 2 * 7 / 6 + 2];
  uint8_t encoded_len = pio_usb_ll_encode_tx_data(packet, sizeof(packet), packet_encoded);

  /* SOF uses pio_usb_bus_usb_transfer() directly. This path is only used for
   * IN/OUT/SETUP transactions, so an armed monitor starts on the first
   * non-SOF host communication sequence. */
  USB_CAPTURE_SEQUENCE_START(pp);

  // Only IN is answered by the device; SETUP/OUT is followed by our DATA, so
  // keep the arming checks out of that gap.
  if (token == USB_PID_IN) {
    usb_transfer(pp, packet_encoded, encoded_len, true);
  } else {
    usb_transfer(pp, packet_encoded, encoded_len, false);
  }
}

void __no_inline_not_in_flash_func(pio_usb_bus_prepare_receive)(const pio_port_t *pp) {
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_restart(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_exec(pp->pio_usb_rx, pp->sm_rx, pp->rx_reset_instr);
  pio_sm_exec(pp->pio_usb_rx, pp->sm_rx, pp->rx_reset_instr2);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);
}

static inline __force_inline bool pio_usb_bus_wait_for_rx_start(const pio_port_t* pp) {
  // USB 2.0 specs: 7.1.19.1: handshake timeout
  // Full-Speed (12 Mbps): 1 bit time = 1 / 12 MHz = 83.3 ns --> 16 bit times = 1.33 µs
  // Low-Speed (1.5 Mbps): 1 bit time = 1 / 1.5 MHz = 666.7 ns --> 16 bit times = 10.67 µs

  // We're starting the timing somewhere in the current microsecond so always assume the first one
  // is less than a full microsecond. For example, a wait of 2 could actually be 1.1 microseconds.
  // We will use 3 us (24 bit time) for Full speed and 12us (18 bit time) for Low speed.
  uint32_t start = get_time_us_32();
  uint32_t timeout = pp->low_speed ? 12 : 3;
  while (get_time_us_32() - start <= timeout) {
    if ((pp->pio_usb_rx->irq & IRQ_RX_START_MASK) != 0) {
      return true;
    }
  }
  return false;
};

uint8_t __no_inline_not_in_flash_func(pio_usb_bus_wait_handshake)(pio_port_t* pp) {
  if (!pio_usb_bus_wait_for_rx_start(pp)) {
    return 0;
  }

  int16_t idx = 0;
  // Timeout in seven microseconds. That is enough time to receive one byte at low speed.
  // This is to detect packets without an EOP because the device was unplugged.
  uint32_t start = get_time_us_32();
  while (get_time_us_32() - start <= 7) {
    if (idx < 2 && pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
      uint8_t data = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
      pp->usb_rx_buffer[idx++] = data;

      start = get_time_us_32(); // reset timeout when a byte is received
    } else if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) != 0) {
      break; // exit if we've gotten an EOP
    }
  }

  if (idx != 2 || pp->usb_rx_buffer[0] != USB_SYNC) {
    return 0; // invalid handshake
  }

  return pp->usb_rx_buffer[1];
}

static __always_inline int receive_packet_and_handshake(
    pio_port_t *pp, uint8_t handshake, bool direct_fs_ack,
    const root_port_t *fs_ack_root) {
  const uint16_t crc16_usb_residue = 0xb001;
  uint16_t crc = 0xffff;
  const uint16_t rx_buf_len = sizeof(pp->usb_rx_buffer) / sizeof(pp->usb_rx_buffer[0]);
  uint32_t idx = 0;

  // Per USB Specs 7.1.18 for turnaround: We must wait at least 2 bit times for inter-packet delay.
  // This is essential for working with LS device specially when we overlocked the mcu.
  // Pre-calculate number of cycle per bit time
  // - Lowspeed: 1 bit time = (cpufreq / 1.5 Mhz) = clk_div_ls_tx.div_int*6Mhz / 1.5 Mhz = 4 * clk_div_ls_tx.div_int
  // - Fullspeed 1 bit time = (cpufreq / 12 Mhz) = clk_div_fs_tx.div_int*48Mhz / 12 Mhz = 4 * clk_div_fs_tx.div_int
  // Since there is also overhead, we only wait 1.5 bit for LS and no wait for FS
  uint32_t turnaround_in_cycle = 0;
  if (!direct_fs_ack && pp->low_speed) {
    turnaround_in_cycle = 6 * pp->clk_div_ls_tx.div_int; // 1.5 bit time
  }

  PIO tx_pio = NULL;
  pio_sm_hw_t *tx_sm = NULL;
  uint32_t tx_stall_mask = 0;
  io_rw_32 *tx_ctrl_set = NULL;
  uint32_t tx_launch_mask = 0;
  uint32_t bus_mask = 0;
  uint32_t idle_j = 0;
  if (direct_fs_ack) {
    tx_pio = pp->pio_usb_tx;
    tx_sm = &tx_pio->sm[pp->sm_tx];
    tx_stall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + pp->sm_tx);
    uint32_t const tx_enable_mask = 1u << pp->sm_tx;
    tx_ctrl_set = hw_set_alias(&tx_pio->ctrl);
    tx_launch_mask = tx_enable_mask | (tx_enable_mask << PIO_CTRL_CLKDIV_RESTART_LSB);
    idle_j = 1u << fs_ack_root->pin_dm;
    bus_mask = (1u << fs_ack_root->pin_dp) | idle_j;
    __asm volatile("" : "+r"(tx_ctrl_set), "+r"(tx_launch_mask),
                        "+r"(bus_mask), "+r"(idle_j));
  }

  if (!pio_usb_bus_wait_for_rx_start(pp)) {
    return -1;
  }
  if (direct_fs_ack && !prepare_fs_ack(pp, tx_stall_mask)) {
    return -1;
  }

  // Timing Critical: use local variable to reduce de-reference
  PIO pio_usb_rx = pp->pio_usb_rx;
  uint sm_rx =  pp->sm_rx;
  uint32_t const rx_empty_mask = 1u << (PIO_FSTAT_RXEMPTY_LSB + sm_rx);
  uint8_t *usb_rx_buffer = pp->usb_rx_buffer;

  // Timeout in seven microseconds. That is enough time to receive one byte at low speed.
  // This is to detect packets without an EOP because the device was unplugged.
  uint32_t start = get_time_us_32();
  while (1) {
    if (!(pio_usb_rx->fstat & rx_empty_mask)) {
      uint8_t data = pio_sm_get(pio_usb_rx, sm_rx) >> 24;
      if (idx < rx_buf_len) {
        usb_rx_buffer[idx] = data;
      }
      start = get_time_us_32(); // reset timeout when a byte is received

      if (idx >= 2) {
        // Skip SYNC and PID. Include the two received CRC bytes.
        crc = update_usb_crc16(crc, data);
      }
      idx++;
    } else if ((pio_usb_rx->irq & IRQ_RX_COMP_MASK) != 0) {
      // Exit since we've gotten an EOP.
      // Timing critical: per USB specs, handshake must be sent within 2-7 bit-time strictly
      if (turnaround_in_cycle) {
        busy_wait_at_least_cycles(turnaround_in_cycle); // wait for turnaround for LS only
      }

      if (handshake == USB_PID_ACK) {
        // A valid DATA packet leaves this residue before the final XOR.
        if (idx >= 4 && crc == crc16_usb_residue) {
          if (direct_fs_ack) {
            if (!wait_for_fs_ack_start(bus_mask, idle_j)) break;
            *tx_ctrl_set = tx_launch_mask;
            wait_for_fs_ack_end(tx_pio, tx_sm, tx_stall_mask);
            restore_fs_tx(pp, true);
          } else {
            usb_transfer(pp, ack_encoded, 5, false);
          }
          return idx - 4;
        }
      } else if (handshake == USB_PID_NAK) {
        usb_transfer(pp, nak_encoded, 5, false);
      } else {
        usb_transfer(pp, stall_encoded, 5, false);
      }
      break;
    } else if (get_time_us_32() - start > 7) {
      break; // device is probably unplugged
    }
  }

  if (direct_fs_ack) {
    restore_fs_tx(pp, false);
  }
  return -1;
}

int __no_inline_not_in_flash_func(pio_usb_bus_receive_packet_and_handshake)(
    pio_port_t *pp, uint8_t handshake) {
  return receive_packet_and_handshake(pp, handshake, false, NULL);
}

int __no_inline_not_in_flash_func(pio_usb_bus_receive_fs_packet_and_ack)(
    pio_port_t *pp, const root_port_t *root) {
  return receive_packet_and_handshake(pp, USB_PID_ACK, true, root);
}

static __always_inline void add_pio_host_rx_program(PIO pio,
                                             const pio_program_t *program,
                                             const pio_program_t *debug_program,
                                             uint *offset, int debug_pin) {
  if (debug_pin < 0) {
    *offset = pio_add_program(pio, program);
  } else {
    *offset = pio_add_program(pio, debug_program);
  }
}

static void __no_inline_not_in_flash_func(initialize_host_programs)(
    pio_port_t *pp, const pio_usb_configuration_t *c, root_port_t *port) {
  // TX program should be placed at address 0
  pio_add_program_at_offset(pp->pio_usb_tx, pp->fs_tx_program, 0);
  pp->offset_tx = 0;
  usb_tx_fs_program_init(pp->pio_usb_tx, pp->sm_tx, pp->offset_tx, port->pin_dp,
                         port->pin_dm);
  uint32_t sideset_fj_lk;
  if (c->pinout == PIO_USB_PINOUT_DPDM) {
    sideset_fj_lk = pio_encode_sideset(2, usb_tx_dpdm_FJ_LK);
  } else {
    sideset_fj_lk = pio_encode_sideset(2, usb_tx_dmdp_FJ_LK);
  }

  pp->tx_start_instr = pio_encode_jmp(pp->offset_tx + 4) | sideset_fj_lk;
  pp->tx_reset_instr = pio_encode_jmp(pp->offset_tx + 2) | sideset_fj_lk;

  add_pio_host_rx_program(pp->pio_usb_rx, &usb_nrzi_decoder_program,
                          &usb_nrzi_decoder_debug_program, &pp->offset_rx,
                          c->debug_pin_rx);
  usb_rx_fs_program_init(pp->pio_usb_rx, pp->sm_rx, pp->offset_rx, port->pin_dp,
                         port->pin_dm, c->debug_pin_rx);
  pp->rx_reset_instr = pio_encode_jmp(pp->offset_rx);
  pp->rx_reset_instr2 = pio_encode_set(pio_x, 0);

  add_pio_host_rx_program(pp->pio_usb_rx, &usb_edge_detector_program,
                          &usb_edge_detector_debug_program, &pp->offset_eop,
                          c->debug_pin_eop);
  eop_detect_fs_program_init(pp->pio_usb_rx, c->sm_eop, pp->offset_eop,
                             port->pin_dp, port->pin_dm, true,
                             c->debug_pin_eop);

  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dp);
  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
}

static void configure_tx_channel(uint8_t ch, PIO pio, uint sm) {
  dma_channel_config conf = dma_channel_get_default_config(ch);

  channel_config_set_read_increment(&conf, true);
  channel_config_set_write_increment(&conf, false);
  channel_config_set_transfer_data_size(&conf, DMA_SIZE_8);
  channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, true));

  dma_channel_set_config(ch, &conf, false);
  dma_channel_set_write_addr(ch, &pio->txf[sm], false);
}

static void apply_config(pio_port_t *pp, const pio_usb_configuration_t *c,
                         root_port_t *port) {
  pp->pio_usb_tx = pio_get_instance(c->pio_tx_num);
  pp->sm_tx = c->sm_tx;
  pp->tx_ch = c->tx_ch;
  pp->pio_usb_rx = pio_get_instance(c->pio_rx_num);
  pp->sm_rx = c->sm_rx;
  pp->sm_eop = c->sm_eop;
  port->pin_dp = c->pin_dp;

  uint highest_pin;
  if (c->pinout == PIO_USB_PINOUT_DPDM) {
    port->pin_dm = c->pin_dp + 1;
    highest_pin = port->pin_dm;
    pp->fs_tx_program = &usb_tx_dpdm_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dpdm_program;
    pp->ls_tx_program = &usb_tx_dmdp_program;
  } else {
    port->pin_dm = c->pin_dp - 1;
    highest_pin = port->pin_dp;
    pp->fs_tx_program = &usb_tx_dmdp_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dmdp_program;
    pp->ls_tx_program = &usb_tx_dpdm_program;
  }

#if defined(PICO_PIO_USE_GPIO_BASE) && PICO_PIO_USE_GPIO_BASE+0
  if (highest_pin > 32) {
    pio_set_gpio_base(pp->pio_usb_tx, 16);
    pio_set_gpio_base(pp->pio_usb_rx, 16);
  }
#else
  (void)highest_pin;
#endif

  port->pinout = c->pinout;

  pp->debug_pin_rx = c->debug_pin_rx;
  pp->debug_pin_eop = c->debug_pin_eop;

  pio_sm_claim(pp->pio_usb_tx, pp->sm_tx);
  pio_sm_claim(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_claim(pp->pio_usb_rx, pp->sm_eop);
}

static void port_pin_drive_setting(const root_port_t *port) {
  gpio_set_slew_rate(port->pin_dp, GPIO_SLEW_RATE_FAST);
  gpio_set_slew_rate(port->pin_dm, GPIO_SLEW_RATE_FAST);
  gpio_set_drive_strength(port->pin_dp, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(port->pin_dm, GPIO_DRIVE_STRENGTH_12MA);
}

void pio_usb_bus_init(pio_port_t *pp, const pio_usb_configuration_t *c,
                      root_port_t *root) {
  memset(root, 0, sizeof(root_port_t));

  pp->pio_usb_tx = pio_get_instance(c->pio_tx_num);
  dma_claim_mask(1<<c->tx_ch);
  configure_tx_channel(c->tx_ch, pp->pio_usb_tx, c->sm_tx);

  apply_config(pp, c, root);
  initialize_host_programs(pp, c, root);
  port_pin_drive_setting(root);
  root->initialized = true;
  root->dev_addr = 0;

  // pre-encode handshake packets
  uint8_t raw_packet[] = {USB_SYNC, USB_PID_ACK};
  pio_usb_ll_encode_tx_data(raw_packet, 2, ack_encoded);
  // Slot 4 emits the first K and wraps to slot 1, which emits the next J
  // before consuming a FIFO symbol. The FIFO must start with the third bit.
  const uint32_t padding = (PIO_USB_TX_ENCODED_DATA_K << 2) |
                           PIO_USB_TX_ENCODED_DATA_K;
  for (uint i = 0; i < sizeof(ack_encoded); i++) {
    const uint32_t next = i + 1 < sizeof(ack_encoded)
                              ? ack_encoded[i + 1] >> 4
                              : padding;
    ack_tail_words[i] = ((uint32_t)ack_encoded[i] << 28) | (next << 24);
  }
  raw_packet[1] = USB_PID_NAK;
  pio_usb_ll_encode_tx_data(raw_packet, 2, nak_encoded);
  raw_packet[1] = USB_PID_STALL;
  pio_usb_ll_encode_tx_data(raw_packet, 2, stall_encoded);
  raw_packet[1] = USB_PID_PRE;
  pio_usb_ll_encode_tx_data(raw_packet, 2, pre_encoded);
}

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

endpoint_t *pio_usb_get_endpoint(usb_device_t *device, uint8_t idx) {
  uint8_t ep_id = device->endpoint_id[idx];
  if (ep_id == 0) {
    return NULL;
  } else if (ep_id >= 1) {
    return &pio_usb_ep_pool[ep_id - 1];
  }
  return NULL;
}

int __no_inline_not_in_flash_func(pio_usb_get_in_data)(endpoint_t *ep,
                                                       uint8_t *buffer,
                                                       uint8_t len) {
  if (ep->has_transfer || ep->is_tx) {
    return -1;
  }

  if (ep->new_data_flag) {
    len = len < ep->actual_len ? len : ep->actual_len;
    memcpy(buffer, (void *)ep->buffer, len);

    ep->new_data_flag = false;

    return pio_usb_ll_transfer_start(ep, ep->buffer, ep->size) ? len : -1;
  }

  return -1;
}

int __no_inline_not_in_flash_func(pio_usb_set_out_data)(endpoint_t *ep,
                                                        const uint8_t *buffer,
                                                        uint8_t len) {
  if (ep->has_transfer || !ep->is_tx) {
    return -1;
  }

  return pio_usb_ll_transfer_start(ep, (uint8_t *)buffer, len) ? 0 : -1;
}

//--------------------------------------------------------------------+
// Low Level Function
//--------------------------------------------------------------------+

void __no_inline_not_in_flash_func(pio_usb_ll_configure_endpoint)(
    endpoint_t *ep, uint8_t const *desc_endpoint) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *)desc_endpoint;
  ep->size = d->max_size[0] | (d->max_size[1] << 8);
  ep->ep_num = d->epaddr;
  ep->attr = d->attr;
  ep->interval = d->interval;
  ep->interval_counter = 0;
  ep->data_id = 0;
}

// Encode transfer data to 2bit sequence represents TX PIO instruction address
uint8_t __no_inline_not_in_flash_func(pio_usb_ll_encode_tx_data)(
    uint8_t const *buffer, uint8_t buffer_len, uint8_t *encoded_data) {
  uint16_t bit_idx = 0;
  int current_state = 1;
  int bit_stuffing = 6;
  for (int idx = 0; idx < buffer_len; idx++) {
    uint8_t data_byte = buffer[idx];
    for (int b = 0; b < 8; b++) {
      uint8_t byte_idx = bit_idx >> 2;
      encoded_data[byte_idx] <<= 2;
      if (data_byte & (1 << b)) {
        if (current_state) {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
        } else {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_J;
        }
        bit_stuffing--;
      } else {
        if (current_state) {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_J;
          current_state = 0;
        } else {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
          current_state = 1;
        }
        bit_stuffing = 6;
      }

      bit_idx++;

      if (bit_stuffing == 0) {
        byte_idx = bit_idx >> 2;
        encoded_data[byte_idx] <<= 2;

        if (current_state) {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_J;
          current_state = 0;
        } else {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
          current_state = 1;
        }
        bit_stuffing = 6;
        bit_idx++;
      }
    }
  }

  uint8_t byte_idx = bit_idx >> 2;
  encoded_data[byte_idx] <<= 2;
  encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_SE0;
  bit_idx++;

  byte_idx = bit_idx >> 2;
  encoded_data[byte_idx] <<= 2;
  encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_COMP;
  bit_idx++;

  // terminate buffers with K
  do {
    byte_idx = bit_idx >> 2;
    encoded_data[byte_idx] <<= 2;
    encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
    bit_idx++;
  } while (bit_idx & 0x03);

  byte_idx = bit_idx >> 2;
  return byte_idx;
}

static inline __force_inline void prepare_tx_data(endpoint_t *ep) {
  uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);
  uint8_t buffer[PIO_USB_EP_SIZE + 4];
  buffer[0] = USB_SYNC;
  buffer[1] = (ep->data_id == 1) ? USB_PID_DATA1
                                 : USB_PID_DATA0; // USB_PID_SETUP also DATA0
  memcpy(buffer + 2, ep->app_buf, xact_len);

  uint16_t const crc16 = calc_usb_crc16(ep->app_buf, xact_len);
  buffer[2 + xact_len] = crc16 & 0xff;
  buffer[2 + xact_len + 1] = crc16 >> 8;

  ep->encoded_data_len =
      pio_usb_ll_encode_tx_data(buffer, xact_len + 4, ep->buffer);
}

bool __no_inline_not_in_flash_func(pio_usb_ll_transfer_start)(endpoint_t *ep,
                                                              uint8_t *buffer,
                                                              uint16_t buflen) {
  if (ep->has_transfer) {
    return false;
  }

  ep->app_buf = buffer;
  ep->total_len = buflen;
  ep->actual_len = 0;
  ep->failed_count = 0;

  if (ep->is_tx) {
    prepare_tx_data(ep);
  } else {
    ep->new_data_flag = false;
  }

  ep->transfer_started = false;
  ep->transfer_aborted = false;
  ep->has_transfer = true;

  return true;
}

bool __no_inline_not_in_flash_func(pio_usb_ll_transfer_continue)(
    endpoint_t *ep, uint16_t xferred_bytes) {
  ep->app_buf += xferred_bytes;
  ep->actual_len += xferred_bytes;
  ep->data_id ^= 1;

  if ((xferred_bytes < ep->size) || (ep->actual_len >= ep->total_len)) {
    // complete if all bytes transferred or short packet
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
    return false;
  } else {
    if (ep->is_tx) {
      prepare_tx_data(ep);
    }

    return true;
  }
}

void __no_inline_not_in_flash_func(pio_usb_ll_transfer_complete)(
    endpoint_t *ep, uint32_t flag) {
  root_port_t *rport = PIO_USB_ROOT_PORT(ep->root_idx);
  uint32_t const ep_mask = (1u << (ep - pio_usb_ep_pool));

  rport->ints |= flag;

  if (flag == PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    rport->ep_complete |= ep_mask;
    if (!ep->is_tx) {
      ep->new_data_flag = true;
    }
  } else if (flag == PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    rport->ep_error |= ep_mask;
  } else if (flag == PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    rport->ep_stalled |= ep_mask;
  } else {
    // something wrong
  }

  ep->has_transfer = false;
}

int pio_usb_host_add_port(uint8_t pin_dp, PIO_USB_PINOUT pinout) {
  for (int idx = 0; idx < PIO_USB_ROOT_PORT_CNT; idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(idx);
    if (!root->initialized) {
      root->pin_dp = pin_dp;

      if (pinout == PIO_USB_PINOUT_DPDM) {
        root->pin_dm = pin_dp + 1;
      } else {
        root->pin_dm = pin_dp - 1;
      }
      root->pinout = pinout;

      gpio_pull_down(pin_dp);
      gpio_pull_down(root->pin_dm);
      pio_gpio_init(pio_port[0].pio_usb_tx, pin_dp);
      pio_gpio_init(pio_port[0].pio_usb_tx, root->pin_dm);
      gpio_set_inover(pin_dp, GPIO_OVERRIDE_INVERT);
      gpio_set_inover(root->pin_dm, GPIO_OVERRIDE_INVERT);
      pio_sm_set_pindirs_with_mask64(pio_port[0].pio_usb_tx, pio_port[0].sm_tx, 0,
                                   (1ull << pin_dp) | (1ull << root->pin_dm));
      port_pin_drive_setting(root);
      root->initialized = true;

      return 0;
    }
  }

  return -1;
}

#pragma GCC pop_options
