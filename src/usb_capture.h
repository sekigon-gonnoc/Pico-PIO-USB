/* Copyright 2026 sekigon-gonnoc
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pio_usb_configuration.h"

#ifndef USB_PACKET_CAPTURE_DEBUG
#define USB_PACKET_CAPTURE_DEBUG 0
#endif

#define USB_CAPTURE_CAPACITY 16384u

/* Raw capture format: each 2-bit sample is one simultaneous D+/D- level.
 * D+ is bit 0 and D- is bit 1. In each byte, temporal
 * samples are packed into bit pairs from least to most significant: sample 0
 * is bits [1:0], sample 1 is [3:2], sample 2 is [5:4], sample 3 is [7:6].
 * Samples are uniformly spaced at sample_rate_hz (4x USB bit rate).
 */
typedef struct {
    bool     armed;
    bool     frozen;
    bool     low_speed;
    bool     tail_incomplete; /* sampler stopped on a DMA boundary, not a USB EOP */
    bool     freeze_on_ack;   /* stop after a valid IN DATA packet is ACKed */
    uint32_t sample_count;   /* raw sample instants, each containing D+ and D- */
    uint32_t sample_rate_hz; /* 48 MHz FS or 6 MHz LS */
    uint32_t data_size;      /* valid chronological bytes; 4 sample instants/byte */
    uint32_t capacity;       /* raw ring capacity in bytes */
} usb_capture_status_t;

/* Initialize after the host and service from the same core as the host.
 * Returns false if a spare PIO block/SM or two DMA channels are unavailable.
 * The monitor samples the configured root port without changing its GPIOs. */
bool usb_capture_init(const pio_usb_configuration_t *host_config);
void usb_capture_task(void);

/* Called by PIO-USB immediately before a non-SOF token is sent. An armed
 * capture starts sampling on the first such host sequence. */
void usb_capture_sequence_start(bool low_speed);

/* Control functions are request-only and safe to call from core0. */
bool usb_capture_arm(void);
void usb_capture_freeze(void);
void usb_capture_clear(void);
void usb_capture_set_ack_trigger(bool enabled);
/* Called by the host after a valid IN DATA packet has been ACKed. */
void usb_capture_ack_success(void);

void usb_capture_status(usb_capture_status_t *out);

/* Read logical chronological offsets. Data can be read only after freeze. */
bool usb_capture_read(uint32_t offset, uint8_t *dst, uint16_t len);
