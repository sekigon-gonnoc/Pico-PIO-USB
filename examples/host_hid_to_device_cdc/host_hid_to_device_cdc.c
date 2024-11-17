/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *                    sekigon-gonnoc
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

// This example runs both host and device concurrently. The USB host receive
// reports from HID device and print it out over USB Device CDC interface.
// For TinyUSB roothub port0 is native usb controller, roothub port1 is
// pico-pio-usb.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

#include "pio_usb.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// uncomment if you are using colemak layout
// #define KEYBOARD_COLEMAK

#ifdef KEYBOARD_COLEMAK
const uint8_t colemak[128] = {
    0, 0, 0, 0, 0, 0, 0, 22,
    9, 23, 7, 0, 24, 17, 8, 12,
    0, 14, 28, 51, 0, 19, 21, 10,
    15, 0, 0, 0, 13, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 18, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0};
#endif

#define PIN_KEY_2 2
#define PIN_KEY_3 3
#define PIN_KEY_4 4
#define PIN_KEY_5 5
#define PIN_KEY_6 6
#define PIN_KEY_7 7
#define PIN_KEY_8 8
#define PIN_KEY_9 9

static uint8_t const keycode2ascii[128][2] = {HID_KEYCODE_TO_ASCII};

/*------------- MAIN -------------*/

// core1: handle host events
void core1_main()
{
  sleep_ms(10);

  // Use tuh_configure() to pass pio configuration to the host stack
  // Note: tuh_configure() must be called before
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

  // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub
  // port1) on core1
  tuh_init(1);

  while (true)
  {
    tuh_task(); // tinyusb host task
  }
}

// core0: handle device events
int main(void)
{
  gpio_init(PIN_KEY_2);
  gpio_init(PIN_KEY_3);
  gpio_init(PIN_KEY_4);
  gpio_init(PIN_KEY_5);
  gpio_init(PIN_KEY_6);
  gpio_init(PIN_KEY_7);
  gpio_init(PIN_KEY_8);
  gpio_init(PIN_KEY_9);

  gpio_set_dir(PIN_KEY_2, GPIO_OUT);
  gpio_set_dir(PIN_KEY_3, GPIO_OUT);
  gpio_set_dir(PIN_KEY_4, GPIO_OUT);
  gpio_set_dir(PIN_KEY_5, GPIO_OUT);
  gpio_set_dir(PIN_KEY_6, GPIO_OUT);
  gpio_set_dir(PIN_KEY_7, GPIO_OUT);
  gpio_set_dir(PIN_KEY_8, GPIO_OUT);
  gpio_set_dir(PIN_KEY_9, GPIO_OUT);

  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);

  sleep_ms(10);

  multicore_reset_core1();
  // all USB task run in core1
  multicore_launch_core1(core1_main);

  // init device stack on native usb (roothub port0)
  tud_init(0);

  while (true)
  {
    tud_task(); // tinyusb device task
    tud_cdc_write_flush();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device CDC
//--------------------------------------------------------------------+

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void)itf;

  char buf[64];
  uint32_t count = tud_cdc_read(buf, sizeof(buf));

  // TODO control LED on keyboard of host stack
  (void)count;
}

//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;

  // Interface protocol (hid_interface_protocol_enum_t)
  const char *protocol_str[] = {"None", "Keyboard", "Mouse"};
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  char tempbuf[256];
  int count = sprintf(tempbuf, "[%04x:%04x][%u] HID Interface%u, Protocol = %s\r\n", vid, pid, dev_addr, instance, protocol_str[itf_protocol]);

  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();

  // Receive report from boot keyboard & mouse only
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD || itf_protocol == HID_ITF_PROTOCOL_MOUSE)
  {
    if (!tuh_hid_receive_report(dev_addr, instance))
    {
      tud_cdc_write_str("Error: cannot request report\r\n");
    }
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  char tempbuf[256];
  int count = sprintf(tempbuf, "[%u] HID Interface%u is unmounted\r\n", dev_addr, instance);
  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();
}

// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for (uint8_t i = 0; i < 6; i++)
  {
    if (report->keycode[i] == keycode)
      return true;
  }

  return false;
}

// Функция для установки состояния пинов
void set_pin_state(uint8_t *key_states, uint8_t length)
{
    // Массив для хранения текущих состояний пинов
    bool pin_states[10] = {false}; // Инициализируем массив состояниями "выключено"

    // Сначала обрабатываем массив состояний клавиш
    for (uint8_t i = 0; i < length; i++)
    {
        char keycode = key_states[i];
        if (keycode) // Если клавиша активна
        {
            switch (keycode)
            {
                case '2':
                    pin_states[5] = true; // 2d 3u 4r 5l 6mh 7lf 8 mf 9lh
                    break;
                case '3':
                    pin_states[2] = true;
                    break;
                case '4':
                    pin_states[4] = true;
                    break;
                case '\n': // Код для клавиши Enter
                case '\r': // Код для возврата каретки (в зависимости от платформы)
                    pin_states[3] = true; // Включаем состояние пина Enter
                    break;
                case '.':
                case '6':
                    //pin_states[6] = true;
                    break;
                 case '0':
                    //pin_states[7] = true;
                    break;
                case '1':
                    //pin_states[8] = true;
                    break;
                case '5':
                    //pin_states[9] = true;
                    break;
                default:
                    break;
            }
        }
    }

    // Обновляем состояния пинов в соответствии с массивом состояний
    gpio_put(PIN_KEY_2, pin_states[2]); // down
    gpio_put(PIN_KEY_3, pin_states[3]); // up
    gpio_put(PIN_KEY_4, pin_states[4]); // right
    gpio_put(PIN_KEY_5, pin_states[5]); // left
    gpio_put(PIN_KEY_6, pin_states[6]); // M_H
    gpio_put(PIN_KEY_7, pin_states[7]); // L_F
    gpio_put(PIN_KEY_8, pin_states[8]); // M_F
    gpio_put(PIN_KEY_9, pin_states[9]); // L_H
}

// convert hid keycode to ascii and print via usb device CDC (ignore non-printable)
#define MAX_KEYS 6

static void process_kbd_report(uint8_t dev_addr, hid_keyboard_report_t const *report)
{
    (void)dev_addr;

    //static hid_keyboard_report_t prev_report = {0, 0, {0}};
    uint8_t key_states[MAX_KEYS] = {0}; // Массив для хранения текущих состояний клавиш
    uint8_t output_index = 0; // Индекс для выходного массива

    // Очистка предыдущего вывода
    //tud_cdc_write("\r       \r", 9);
    tud_cdc_write("\n", 1);

    // Первый цикл: Обработка текущего отчета и обновление состояния клавиш
    for (uint8_t i = 0; i < MAX_KEYS; i++)
    {
        uint8_t keycode = report->keycode[i]; // Используем report вместо hid_keyboard_report_t
        if (keycode)
        {
			//bool const is_shift = report->modifier & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT);
            //uint8_t ch = keycode2ascii[keycode][is_shift ? 1 : 0];
            key_states[i] = keycode2ascii[keycode][0];//keycode; // Обновляем состояние клавиши
            output_index++; // Увеличиваем индекс выходного массива
        }
    }

    // Отправляем текущее состояние только если есть символы в выходном массиве
    if (output_index > 0)
    {
        tud_cdc_write(key_states, output_index); // Отправляем только валидную часть выходного массива
    }
    set_pin_state(key_states, output_index);
    tud_cdc_write_flush(); // Сбрасываем данные в CDC
    //prev_report = *report; // Обновляем предыдущий отчет
}

// send mouse report to usb device CDC
static void process_mouse_report(uint8_t dev_addr, hid_mouse_report_t const *report)
{
  //------------- button state  -------------//
  // uint8_t button_changed_mask = report->buttons ^ prev_report.buttons;
  char l = report->buttons & MOUSE_BUTTON_LEFT ? 'L' : '-';
  char m = report->buttons & MOUSE_BUTTON_MIDDLE ? 'M' : '-';
  char r = report->buttons & MOUSE_BUTTON_RIGHT ? 'R' : '-';

  char tempbuf[32];
  int count = sprintf(tempbuf, "[%u] %c%c%c %d %d %d\r\n", dev_addr, l, m, r, report->x, report->y, report->wheel);

  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len)
{
  (void)len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  switch (itf_protocol)
  {
  case HID_ITF_PROTOCOL_KEYBOARD:
    process_kbd_report(dev_addr, (hid_keyboard_report_t const *)report);
    break;

  case HID_ITF_PROTOCOL_MOUSE:
    process_mouse_report(dev_addr, (hid_mouse_report_t const *)report);
    break;

  default:
    break;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance))
  {
    tud_cdc_write_str("Error: cannot request report\r\n");
  }
}
