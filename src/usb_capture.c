/* Copyright 2026 sekigon-gonnoc
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "usb_capture.h"

#if USB_PACKET_CAPTURE_DEBUG

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/platform.h"

#define USB_CAPTURE_BLOCK_BITS 12u
#define USB_CAPTURE_BLOCK_BYTES (1u << USB_CAPTURE_BLOCK_BITS)
#define USB_CAPTURE_BLOCK_WORDS (USB_CAPTURE_BLOCK_BYTES / sizeof(uint32_t))
#define USB_CAPTURE_BLOCK_COUNT (USB_CAPTURE_CAPACITY / USB_CAPTURE_BLOCK_BYTES)

/* One sample per PIO clock. The capture SM only reads the pins, so the
 * host retains ownership of their mux and directions. */
static uint16_t capture_instructions[1];
static const struct pio_program capture_program = {
    .instructions = capture_instructions,
    .length = 1,
    .origin = -1,
};

/* 16 KiB SRAM ring, divided into four DMA blocks. A pair of chained DMA
 * channels continuously fills consecutive blocks without core servicing.
 */
static uint32_t capture_words[USB_CAPTURE_CAPACITY / sizeof(uint32_t)]
    __attribute__((aligned(USB_CAPTURE_BLOCK_BYTES)));

static int dma_channels[2] = {-1, -1};
static PIO capture_pio;
static uint capture_sm;
static uint32_t dma_mask;
static bool swap_pins;
static uint pio_program_offset;
static uint8_t channel_block[2];
static uint8_t next_slot;
static uint32_t completed_bytes;
static uint32_t valid_bytes;
static uint32_t oldest_block;
static bool tail_incomplete;
static bool freeze_seen;
static uint32_t sampling_rate;
static bool low_speed;
static volatile bool initialized;
static volatile bool armed;
static volatile bool frozen;
static volatile bool waiting_for_sequence;
static volatile bool arm_request;
static volatile bool freeze_request;
static volatile bool clear_request;
static volatile bool freeze_on_ack;

static inline void set_sample_clock(bool ls) {
    const uint32_t target_hz = ls ? 6000000u : 48000000u;
    const uint32_t sys_hz = clock_get_hz(clk_sys);
    const float divider = (float)sys_hz / (float)target_hz;
    pio_sm_set_clkdiv(capture_pio, capture_sm, divider);
    sampling_rate = (uint32_t)((float)sys_hz / divider + 0.5f);
    low_speed = ls;
}

static inline void start_dma_channel(uint8_t slot, uint8_t block) {
    const int channel = dma_channels[slot];
    channel_block[slot] = block;
    dma_channel_config cfg = dma_channel_get_default_config((uint)channel);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    /* A late IRQ can let chaining retrigger an unrearmed channel. Keep
     * that channel's writes inside its assigned block. */
    channel_config_set_ring(&cfg, true, USB_CAPTURE_BLOCK_BITS);
    channel_config_set_dreq(&cfg, pio_get_dreq(capture_pio, capture_sm, false));
    channel_config_set_chain_to(&cfg, (uint)dma_channels[slot ^ 1u]);
    dma_channel_configure(
        (uint)channel,
        &cfg,
        &capture_words[(uint32_t)block * USB_CAPTURE_BLOCK_WORDS],
        &capture_pio->rxf[capture_sm],
        USB_CAPTURE_BLOCK_WORDS,
        false);
}

static void __not_in_flash_func(stop_sampler)(void) {
    pio_sm_set_enabled(capture_pio, capture_sm, false);
    dma_set_irq1_channel_mask_enabled(dma_mask, false);
    for (uint slot = 0; slot < 2; ++slot) {
        /* Prevent either channel from restarting the other while aborting. */
        hw_clear_bits(&dma_channel_hw_addr((uint)dma_channels[slot])->al1_ctrl,
                      DMA_CH0_CTRL_TRIG_EN_BITS);
    }
    for (uint slot = 0; slot < 2; ++slot) {
        const uint channel = (uint)dma_channels[slot];
        dma_channel_abort(channel);
        dma_channel_acknowledge_irq1(channel);
    }
    pio_sm_clear_fifos(capture_pio, capture_sm);
}

static void __not_in_flash_func(capture_dma_complete)(uint8_t slot) {
    if (completed_bytes < USB_CAPTURE_CAPACITY) {
        completed_bytes += USB_CAPTURE_BLOCK_BYTES;
    }
    const uint8_t done_block = channel_block[slot];
    const uint8_t following_block = (uint8_t)((done_block + 2u) % USB_CAPTURE_BLOCK_COUNT);

    if (freeze_request && freeze_seen) {
        /* The first boundary after the request completes one more full
         * block. The chained channel is then in a new block; stop it and
         * retain only the words already committed to SRAM. */
        stop_sampler();

        const uint8_t active_slot = slot ^ 1u;
        const uint8_t active_block = channel_block[active_slot];
        const uint32_t remaining_words = dma_channel_hw_addr(
            (uint)dma_channels[active_slot])->transfer_count;
        const uint32_t active_bytes =
            (USB_CAPTURE_BLOCK_WORDS - remaining_words) * sizeof(uint32_t);
        if (completed_bytes >= USB_CAPTURE_CAPACITY && active_bytes != 0) {
            /* The active segment is replacing the oldest ring segment.
             * Its committed prefix is the newest tail; start at the next
             * segment and return that prefix last to preserve chronology.
             */
            valid_bytes = USB_CAPTURE_CAPACITY - USB_CAPTURE_BLOCK_BYTES + active_bytes;
            oldest_block = (uint32_t)(active_block + 1u) % USB_CAPTURE_BLOCK_COUNT;
        } else if (completed_bytes >= USB_CAPTURE_CAPACITY) {
            valid_bytes = USB_CAPTURE_CAPACITY;
            oldest_block = active_block;
        } else {
            valid_bytes = completed_bytes + active_bytes;
            oldest_block = 0;
        }
        tail_incomplete = true;
        freeze_request = false;
        freeze_seen = false;
        armed = false;
        __dmb();
        frozen = true;
        return;
    }

    /* The completed channel is rearmed two blocks ahead. The other
     * channel is filling the intervening block due to DMA chaining.
     */
    start_dma_channel(slot, following_block);
    if (freeze_request) freeze_seen = true;
}

static void __not_in_flash_func(capture_dma_irq)(void) {
    const uint slot = next_slot;
    const uint channel = (uint)dma_channels[slot];
    if (!dma_channel_get_irq1_status(channel)) return;

    next_slot ^= 1u;
    if (dma_channel_get_irq1_status((uint)dma_channels[next_slot])) {
        /* Both blocks completed before the first channel could be rearmed.
         * The stream can have a discontinuity, so publish an empty snapshot. */
        stop_sampler();
        valid_bytes = 0;
        tail_incomplete = true;
        freeze_request = false;
        armed = false;
        __dmb();
        frozen = true;
        return;
    }
    dma_channel_acknowledge_irq1(channel);
    capture_dma_complete(slot);
}

static void begin_capture(bool ls) {
    pio_sm_restart(capture_pio, capture_sm);
    pio_sm_exec(capture_pio, capture_sm, pio_encode_jmp(pio_program_offset));

    completed_bytes = 0;
    valid_bytes = 0;
    oldest_block = 0;
    tail_incomplete = false;
    freeze_seen = false;
    set_sample_clock(ls);

    next_slot = 0;
    start_dma_channel(0, 0);
    start_dma_channel(1, 1);
    dma_set_irq1_channel_mask_enabled(dma_mask, true);
    dma_channel_start((uint)dma_channels[0]);
    pio_sm_set_enabled(capture_pio, capture_sm, true);
    armed = true;
}

bool usb_capture_init(const pio_usb_configuration_t *host_config) {
    if (initialized) return true;

    capture_instructions[0] = pio_encode_in(pio_pins, 2);
    int sm = -1;
    for (uint i = 0; i < NUM_PIOS; ++i) {
        if (i == host_config->pio_tx_num || i == host_config->pio_rx_num) continue;

        PIO candidate = pio_get_instance(i);
        if (!pio_can_add_program(candidate, &capture_program)) continue;

        sm = pio_claim_unused_sm(candidate, false);
        if (sm >= 0) {
            capture_pio = candidate;
            break;
        }
    }
    if (sm < 0) return false;
    capture_sm = (uint)sm;

    dma_channels[0] = dma_claim_unused_channel(false);
    dma_channels[1] = dma_claim_unused_channel(false);
    if (dma_channels[0] < 0 || dma_channels[1] < 0) {
        for (uint slot = 0; slot < 2; ++slot) {
            if (dma_channels[slot] >= 0) dma_channel_unclaim((uint)dma_channels[slot]);
        }
        pio_sm_unclaim(capture_pio, capture_sm);
        return false;
    }
    dma_mask = (1u << dma_channels[0]) | (1u << dma_channels[1]);

    swap_pins = host_config->pinout == PIO_USB_PINOUT_DMDP;
    const uint pin_base = host_config->pin_dp - swap_pins;
    pio_program_offset = pio_add_program(capture_pio, &capture_program);
    pio_sm_config cfg = pio_get_default_sm_config();
    sm_config_set_in_pins(&cfg, pin_base);
    sm_config_set_in_shift(&cfg, true, true, 32);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_wrap(&cfg, pio_program_offset, pio_program_offset);
    pio_sm_init(capture_pio, capture_sm, pio_program_offset, &cfg);
    irq_add_shared_handler(DMA_IRQ_1, capture_dma_irq,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_1, true);

    sampling_rate = 48000000u;
    frozen = true;
    __dmb();
    initialized = true;
    return true;
}

void usb_capture_task(void) {
    if (!initialized || !(clear_request || arm_request ||
                          (freeze_request && waiting_for_sequence))) return;

    /* The host timer can start sampling as soon as it sees an armed
     * capture. Serialize the state transitions with that timer and DMA. */
    const uint32_t irq_state = save_and_disable_interrupts();
    if (clear_request && !armed) {
        completed_bytes = 0;
        valid_bytes = 0;
        oldest_block = 0;
        tail_incomplete = false;
        freeze_request = false;
        __dmb();
        clear_request = false;
    }

    if (arm_request && !armed) {
        frozen = false;
        completed_bytes = 0;
        valid_bytes = 0;
        oldest_block = 0;
        tail_incomplete = false;
        waiting_for_sequence = true;
        armed = true;
        arm_request = false;
    }

    if (freeze_request && waiting_for_sequence) {
        waiting_for_sequence = false;
        freeze_request = false;
        freeze_seen = false;
        armed = false;
        __dmb();
        frozen = true;
    }
    restore_interrupts(irq_state);
}

bool usb_capture_arm(void) {
    if (!initialized) return false;
    if (!armed) arm_request = true;
    return true;
}

void usb_capture_sequence_start(bool ls) {
    if (!initialized || !armed || !waiting_for_sequence) return;
    waiting_for_sequence = false;
    begin_capture(ls);
}

void __not_in_flash_func(usb_capture_freeze)(void) {
    if (armed || arm_request) freeze_request = true;
}

void usb_capture_clear(void) {
    arm_request = false;
    clear_request = true;
    if (armed) freeze_request = true;
}

void usb_capture_set_ack_trigger(bool enabled) {
    freeze_on_ack = enabled;
}

void usb_capture_ack_success(void) {
    if (freeze_on_ack && armed && !waiting_for_sequence) {
        freeze_request = true;
    }
}

void usb_capture_status(usb_capture_status_t *out) {
    if (!out) return;
    /* Report a core0 arm request immediately so STATUS cannot briefly expose
     * the previous frozen snapshot before core1 consumes the request. */
    out->armed = arm_request || armed;
    out->frozen = !out->armed && initialized && frozen;
    if (out->frozen) __dmb();
    out->low_speed = low_speed;
    out->tail_incomplete = tail_incomplete;
    out->freeze_on_ack = freeze_on_ack;
    out->sample_count = valid_bytes * 4u;
    out->sample_rate_hz = sampling_rate;
    out->data_size = valid_bytes;
    out->capacity = USB_CAPTURE_CAPACITY;
}

bool usb_capture_read(uint32_t offset, uint8_t *dst, uint16_t len) {
    if (!frozen || armed || arm_request || clear_request || !dst) return false;
    __dmb();
    if (offset > valid_bytes || len > valid_bytes - offset) return false;

    const uint32_t first_block_byte = oldest_block * USB_CAPTURE_BLOCK_BYTES;
    const uint8_t *capture_bytes = (const uint8_t *)capture_words;
    for (uint32_t i = 0; i < len; ++i) {
        const uint32_t physical = (first_block_byte + offset + i) % USB_CAPTURE_CAPACITY;
        /* GPIO input overrides apply to both PIO blocks. Undo the host RX
         * inversion and export D+ in bit 0 of each pair. */
        uint8_t levels = capture_bytes[physical] ^ 0xffu;
        if (swap_pins) levels = ((levels & 0x55u) << 1u) | ((levels & 0xaau) >> 1u);
        dst[i] = levels;
    }
    return true;
}

#endif
