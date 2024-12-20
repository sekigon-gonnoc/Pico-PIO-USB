#pragma once

#if PICO_SDK_VERSION_MAJOR < 2
static __always_inline void pio_sm_set_jmp_pin(PIO pio, uint sm, uint jmp_pin) {
  pio->sm[sm].execctrl =
      (pio->sm[sm].execctrl & ~PIO_SM0_EXECCTRL_JMP_PIN_BITS) |
      (jmp_pin << PIO_SM0_EXECCTRL_JMP_PIN_LSB);
}
#endif

#if PICO_SDK_VERSION_MAJOR < 2 || (PICO_SDK_VERSION_MAJOR == 2 && PICO_SDK_VERSION_MINOR < 1)
static void pio_sm_set_pins_with_mask64(PIO pio, uint sm, uint64_t pin_values, uint64_t pin_mask) {
    pio_sm_set_pins_with_mask(pio, sm, (uint32_t) pin_values, (uint32_t) pin_mask);
}

static void pio_sm_set_pindirs_with_mask64(PIO pio, uint sm, uint64_t pin_values, uint64_t pin_mask) {
    pio_sm_set_pindirs_with_mask(pio, sm, (uint32_t) pin_values, (uint32_t) pin_mask);
}
#endif

