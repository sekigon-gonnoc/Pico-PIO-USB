
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include "pico/types.h"
#include "pio_usb.h"
#include "pio_usb_configuration.h"
#include "pio_usb_ll.h"
#include "usb_definitions.h"
#include "usb_rx.pio.h"

pio_usb_configuration_t pio_usb_config = PIO_USB_DEFAULT_CONFIG;

static bool do_test(pio_port_t *pp);

int main() {
  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);

  stdio_init_all();

  sleep_ms(1000);

  printf("start\n");

  pio_usb_config.alarm_pool = NULL;
  pio_usb_config.skip_alarm_pool = true;
  pio_usb_config.pin_dp = 0;
  pio_usb_host_init(&pio_usb_config);
  if (pio_usb_host_add_port(4, PIO_USB_PINOUT_DMDP) != 0) {
    printf("Failed to add root port\n");
  }
  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  while (1) {
    printf("\n[b]: Jump to bootloader, [others]: Execute tests\n");
    char c=0;
    scanf("%c", &c);

    if (c == 'b') {
      reset_usb_boot(1 << PICO_DEFAULT_LED_PIN, 0);
    }

    {
      printf("\nTest1: FS, 1\n");

      root_port_t *root = PIO_USB_ROOT_PORT(0);
      gpio_pull_up(root->pin_dp);
      gpio_pull_down(root->pin_dm);
      root->is_fullspeed = true;
      root->initialized = true;
      root->connected = true;
      root->suspended = false;

      uint32_t irq = save_and_disable_interrupts();
      pio_usb_host_frame();
      restore_interrupts(irq);

      printf("%s\n", do_test(pp) ? "[OK]" : "[NG]");

      root->connected = false;
    }

    {
      printf("\nTest2: FS, 2\n");

      root_port_t *root = PIO_USB_ROOT_PORT(1);
      gpio_pull_up(root->pin_dp);
      gpio_pull_down(root->pin_dm);
      root->is_fullspeed = true;
      root->initialized = true;
      root->connected = true;
      root->suspended = false;

      uint32_t irq = save_and_disable_interrupts();
      pio_usb_host_frame();
      restore_interrupts(irq);

      printf("%s\n", do_test(pp) ? "[OK]" : "[NG]");

      root->connected = false;
    }

    {
      printf("\nTest3: LS, 1\n");

      root_port_t *root = PIO_USB_ROOT_PORT(0);
      gpio_pull_down(root->pin_dp);
      gpio_pull_up(root->pin_dm);
      root->is_fullspeed = false;
      root->initialized = true;
      root->connected = true;
      root->suspended = false;

      uint32_t irq = save_and_disable_interrupts();
      pio_usb_host_frame();
      restore_interrupts(irq);

      printf("%s\n", do_test(pp) ? "[OK]" : "[NG]");

      root->connected = false;
    }

    {
      printf("\nTest4: LS, 2\n");

      root_port_t *root = PIO_USB_ROOT_PORT(1);
      gpio_pull_down(root->pin_dp);
      gpio_pull_up(root->pin_dm);
      root->is_fullspeed = false;
      root->initialized = true;
      root->connected = true;
      root->suspended = false;

      uint32_t irq = save_and_disable_interrupts();
      pio_usb_host_frame();
      restore_interrupts(irq);

      printf("%s\n", do_test(pp) ? "[OK]" : "[NG]");

      root->connected = false;
    }

    {
      printf("\nTest 5: Software Encode Speed\n");
      uint8_t buffer[64];
      uint8_t encoded_data[64 * 2 * 7 / 6 + 2];
      for (size_t i = 0; i < sizeof(buffer); i++) {
        buffer[i] = i;
      }

      absolute_time_t start = get_absolute_time();
      for (int i = 0; i < 1000; i++) {
        pio_usb_ll_encode_tx_data(buffer, sizeof(buffer), encoded_data);
      }
      absolute_time_t end = get_absolute_time();
      int64_t diff = absolute_time_diff_us(start, end);
      printf("%f us (64bytes packet)", diff / 1000.0f);
    }
  }
}

static bool do_test(pio_port_t *pp) {
  bool success = true;

  // Prepare transfer data
  uint8_t test_data[] = {0x80, 0xff, 0x00, 0x83};
  endpoint_t *ep = PIO_USB_ENDPOINT(0);
  ep->has_transfer = false;
  ep->is_tx = true;
  ep->size = 32;
  pio_usb_ll_transfer_start(ep, test_data, sizeof(test_data));

  // Start receiver
  pio_usb_bus_prepare_receive(pp);
  pio_usb_bus_start_receive(pp);

  uint32_t irq = save_and_disable_interrupts();
  // Start transmitter
  pio_usb_bus_usb_transfer(pp, ep->buffer, ep->encoded_data_len);
  restore_interrupts(irq);

  // Check received data
  uint8_t received[sizeof(test_data) + 4];
  memset(received, 0, sizeof(received));
  uint8_t received_cnt = 0;

  while (pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
    if (received_cnt >= sizeof(received)) {
      printf("\t[NG] Invalid size\n");
      success = false;
      break;
    }
    received[received_cnt++] = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
  }

  for (size_t i = 0; i < sizeof(received); i++) {
    printf("%02x ", received[i]);
  }
  printf("\n");

  for (size_t i = 0; i < sizeof(test_data); i++) {
    if (test_data[i] != received[i + 2]) {
      printf("\t[NG] Invalid data at %d. Expect: %02x, Received: %02x\n", i,
             test_data[i], received[i + 2]);
      success = false;
    }
  }

  ep->has_transfer = false;

  return success;
}