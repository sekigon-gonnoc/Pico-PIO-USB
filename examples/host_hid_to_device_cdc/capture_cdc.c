/* SPDX-License-Identifier: MIT */
#include "capture_cdc.h"

#include "tusb.h"
#include "usb_capture.h"

enum {
    MAGIC_0 = 0xc7,
    MAGIC_1 = 0x51,
    CMD_ARM = 1,
    CMD_FREEZE,
    CMD_STATUS,
    CMD_READ,
    CMD_CLEAR,
    CMD_ACK_TRIGGER,
    HEADER_SIZE = 6,
    MAX_REQUEST_DATA = 6,
    MAX_READ_DATA = 48,
    RESULT_OK = 0,
    RESULT_BAD_REQUEST = 1,
    RESULT_UNAVAILABLE = 2,
};

static uint8_t request[HEADER_SIZE + MAX_REQUEST_DATA];
static uint8_t request_size;
static uint8_t response[HEADER_SIZE + 1 + MAX_READ_DATA];
static uint8_t response_size;
static uint8_t response_sent;

static uint16_t get_u16(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t get_u32(const uint8_t *data) {
    return (uint32_t)get_u16(data) | ((uint32_t)get_u16(data + 2) << 16);
}

static void put_u32(uint8_t *data, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
        data[i] = (uint8_t)(value >> (8u * i));
    }
}

static void make_response(uint16_t len) {
    const uint8_t command = request[2];
    const uint8_t *payload = request + HEADER_SIZE;
    uint8_t *out = response + HEADER_SIZE + 1;
    uint8_t result = RESULT_BAD_REQUEST;
    uint8_t out_len = 0;

    switch (command) {
        case CMD_ARM:
            if (len == 0) {
                result = usb_capture_arm() ? RESULT_OK : RESULT_UNAVAILABLE;
            }
            break;
        case CMD_FREEZE:
            if (len == 0) {
                usb_capture_freeze();
                result = RESULT_OK;
            }
            break;
        case CMD_STATUS:
            if (len == 0) {
                usb_capture_status_t status;
                usb_capture_status(&status);
                out[0] = status.armed ? 1 : status.frozen ? 2 : 0;
                out[1] = status.low_speed;
                out[2] = status.tail_incomplete;
                put_u32(out + 3, status.sample_count);
                put_u32(out + 7, status.sample_rate_hz);
                put_u32(out + 11, status.data_size);
                put_u32(out + 15, status.capacity);
                out[19] = status.freeze_on_ack;
                out_len = 20;
                result = RESULT_OK;
            }
            break;
        case CMD_READ:
            if (len == 6) {
                const uint16_t count = get_u16(payload + 4);
                if (count > MAX_READ_DATA) break;
                if (usb_capture_read(get_u32(payload), out, count)) {
                    out_len = (uint8_t)count;
                    result = RESULT_OK;
                } else {
                    result = RESULT_UNAVAILABLE;
                }
            }
            break;
        case CMD_CLEAR:
            if (len == 0) {
                usb_capture_clear();
                result = RESULT_OK;
            }
            break;
        case CMD_ACK_TRIGGER:
            if (len == 1 && payload[0] <= 1) {
                usb_capture_set_ack_trigger(payload[0] != 0);
                result = RESULT_OK;
            }
            break;
    }

    response[0] = MAGIC_0;
    response[1] = MAGIC_1;
    response[2] = command | 0x80u;
    response[3] = request[3];
    response[4] = out_len + 1;
    response[5] = 0;
    response[6] = result;
    response_size = HEADER_SIZE + 1 + out_len;
    response_sent = 0;
    request_size = 0;
}

void capture_cdc_task(void) {
    if (!tud_cdc_connected()) {
        request_size = 0;
        response_size = 0;
        response_sent = 0;
        return;
    }

    /* Wait for CDC TX space before consuming more commands. TinyUSB's RX
     * FIFO retains subsequent bytes while a response is partially queued. */
    if (response_sent < response_size) {
        response_sent += tud_cdc_write(response + response_sent,
                                       response_size - response_sent);
        if (response_sent < response_size) return;
    }

    uint8_t byte;
    while (tud_cdc_read(&byte, 1) == 1) {
        if (request_size < 2) {
            if (request_size == 1 && byte == MAGIC_1) {
                request[request_size++] = byte;
            } else {
                request_size = byte == MAGIC_0 ? 1 : 0;
                if (request_size) request[0] = byte;
            }
            continue;
        }
        request[request_size++] = byte;
        if (request_size < HEADER_SIZE) continue;

        const uint16_t len = get_u16(request + 4);
        if (len > MAX_REQUEST_DATA || request_size == HEADER_SIZE + len) {
            make_response(len);
            return;
        }
    }
}
