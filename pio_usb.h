
#pragma once

#include "pio_usb_configuration.h"
#include "usb_definitions.h"

usb_device_t *pio_usb_init(const pio_usb_configuration_t *c);
int pio_usb_add_port(uint8_t pin_dp);
void pio_usb_task(void);

endpoint_t *pio_usb_get_endpoint(usb_device_t *device, uint8_t idx);
int pio_usb_get_in_data(endpoint_t *ep, uint8_t *buffer, uint8_t len);
int pio_usb_set_out_data(endpoint_t *ep, const uint8_t *buffer, uint8_t len);

void pio_usb_stop(void);
void pio_usb_restart(void);