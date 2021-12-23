
#pragma once

#include <stdint.h>

#include "pio_usb_configuration.h"

typedef struct {
  volatile bool connected;
  volatile uint pin_dp;
  volatile uint pin_dm;
} root_port_t;

typedef enum {
  CONTROL_NONE,
  CONTROL_IN,
  CONTROL_OUT,
  CONTROL_COMPLETE,
  CONTROL_ERROR,
} control_transfer_operation_t;

typedef enum {
  EP_IN = 0x80,
  EP_OUT = 0x00,
} ep_type_t;

typedef enum {
  STAGE_SETUP,
  STAGE_IN,
  STAGE_OUT,
  STAGE_STATUS,
  STAGE_COMPLETE,
  STAGE_ERROR
} setup_transfer_stage_t;
typedef enum {
  STAGE_DEVICE,
  STAGE_CONFIG,
  STAGE_CONFIG2,
  STAGE_INTERFACE,
  STAGE_ENDPOINT
} usb_communication_stage_t;

typedef struct {
  uint8_t *tx_address;
  uint8_t tx_length;
} packet_info_t;

typedef struct {
  volatile uint8_t rx_buffer[512];
  volatile packet_info_t setup_packet;
  volatile packet_info_t out_data_packet;
  volatile int16_t request_length;
  volatile control_transfer_operation_t operation;
  volatile setup_transfer_stage_t stage;
} control_pipe_t;

typedef struct {
  volatile uint8_t ep_num;
  volatile uint8_t size;
  volatile uint8_t buffer[64 + 4];
  volatile uint8_t packet_len;
  volatile bool new_data_flag;
  volatile bool is_interrupt;
  volatile uint8_t interval;
  volatile uint8_t interval_counter;
  volatile uint8_t data_id;  // data0 or data1
} endpoint_t;

typedef enum {
  EVENT_NONE,
  EVENT_CONNECT,
  EVENT_DISCONNECT,
} usb_device_event_t;

typedef struct {
  volatile bool connected;
  volatile bool enumerated;
  volatile usb_device_event_t event;
  volatile uint8_t address;
  volatile uint16_t vid;
  volatile uint16_t pid;
  volatile bool is_fullspeed;
  control_pipe_t control_pipe;
  endpoint_t endpoint[PIO_USB_EP_CNT];
} usb_device_t;

enum {
  USB_SYNC = 0x80,
  USB_PID_OUT = 0xe1,
  USB_PID_IN = 0x69,
  USB_PID_SOF = 0xa5,
  USB_PID_SETUP = 0x2d,
  USB_PID_DATA0 = 0xc3,
  USB_PID_DATA1 = 0x4b,
  USB_PID_ACK = 0xd2,
  USB_PID_NAK = 0x5a,
  USB_PID_STALL = 0x1e,
  USB_CRC16_PLACE = 0,
};

enum {
    DESC_TYPE_STRING = 0x03,
    DESC_TYPE_INTERFACE = 0x04,
    DESC_TYPE_ENDPOINT = 0x05,
    DESC_TYPE_HID = 0x21,
};

enum {
    CLASS_HID = 0x03,
};

enum {
    EP_ATTR_INTERRUPT = 0x03,
};

typedef struct {
  uint8_t sync;
  uint8_t pid;
  uint8_t request_type;
  uint8_t request;
  uint8_t value_lsb;
  uint8_t value_msb;
  uint8_t index_lsb;
  uint8_t index_msb;
  uint8_t length_lsb;
  uint8_t length_msb;
  uint8_t crc16[2];
} usb_setup_packet_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t bcd_usb[2];
  uint8_t class;
  uint8_t subclass;
  uint8_t protocol;
  uint8_t max_packet_size;
  uint8_t vid[2];
  uint8_t pid[2];
  uint8_t bcd_device[2];
  uint8_t manufacture;
  uint8_t product;
  uint8_t serial;
  uint8_t num_configu;
} device_descriptor_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t string;
} string_descriptor_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t inum;
  uint8_t altsetting;
  uint8_t numep;
  uint8_t iclass;
  uint8_t isubclass;
  uint8_t iprotocol;
  uint8_t iface;
} interface_descriptor_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t epaddr;
  uint8_t attr;
  uint8_t max_size[2];
  uint8_t interval;
} endpoint_descriptor_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t bcd_hid[2];
  uint8_t contry_code;
  uint8_t num_desc;
  uint8_t desc_type;
  uint8_t desc_len[2];
} hid_descriptor_t;

typedef struct configuration_descriptor_tag {
  uint8_t length;
  uint8_t type;
  uint8_t total_length_lsb;
  uint8_t total_length_msb;
  uint8_t num_interfaces;
  uint8_t configuration_value;
  uint8_t configuration;
  uint8_t attributes;
  uint8_t max_power;
} configuration_descriptor_t;

#define GET_DEVICE_DESCRIPTOR_REQ_DEFAULT                          \
  {                                                                \
    USB_SYNC, USB_PID_DATA0, 0x80, 0x06, 0, 0x01, 0, 0, 0x12, 0, { \
      USB_CRC16_PLACE, USB_CRC16_PLACE                             \
    }                                                              \
  }
#define GET_CONFIGURATION_DESCRIPTOR_REQ_DEFAULT                   \
  {                                                                \
    USB_SYNC, USB_PID_DATA0, 0x80, 0x06, 0, 0x02, 0, 0, 0x09, 0, { \
      USB_CRC16_PLACE, USB_CRC16_PLACE                             \
    }                                                              \
  }
#define SET_CONFIGURATION_REQ_DEFAULT                        \
  {                                                          \
    USB_SYNC, USB_PID_DATA0, 0x00, 0x09, 0, 0, 0, 0, 0, 0, { \
      USB_CRC16_PLACE, USB_CRC16_PLACE                       \
    }                                                        \
  }

#define SET_HID_IDLE_REQ_DEFAULT                             \
  {                                                          \
    USB_SYNC, USB_PID_DATA0, 0x21, 0x0A, 0, 0, 0, 0, 0, 0, { \
      USB_CRC16_PLACE, USB_CRC16_PLACE                       \
    }                                                        \
  }
#define GET_HID_REPORT_DESCRIPTOR_DEFAULT                          \
  {                                                                \
    USB_SYNC, USB_PID_DATA0, 0x81, 0x06, 0, 0x22, 0, 0, 0xff, 0, { \
      USB_CRC16_PLACE, USB_CRC16_PLACE                             \
    }                                                              \
  }
#define SET_ADDRESS_REQ_DEFAULT                             \
  {                                                         \
    USB_SYNC, USB_PID_DATA0, 0, 0x5, 0x02, 0, 0, 0, 0, 0, { \
      USB_CRC16_PLACE, USB_CRC16_PLACE                      \
    }                                                       \
  }
