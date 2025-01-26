#pragma once

#if PICO_SDK_VERSION_MAJOR < 2
static __always_inline void pio_sm_set_jmp_pin(PIO pio, uint sm, uint jmp_pin) {
  pio->sm[sm].execctrl =
      (pio->sm[sm].execctrl & ~PIO_SM0_EXECCTRL_JMP_PIN_BITS) |
      (jmp_pin << PIO_SM0_EXECCTRL_JMP_PIN_LSB);
}

static_assert(PIO1_BASE - PIO0_BASE == (1u << 20), "hardware layout mismatch");
#define PIO_INSTANCE(instance) ((pio_hw_t *)(PIO0_BASE + (instance) * (1u << 20)))
static __always_inline PIO pio_get_instance(uint instance) {
    return PIO_INSTANCE(instance);
}

#define PIO_NUM(pio) (((uintptr_t)(pio) - PIO0_BASE) >> 20)
#define NUM_PIO_IRQS (2u)
#define PIO_IRQ_NUM(pio, irqn) (PIO0_IRQ_0 + NUM_PIO_IRQS * PIO_NUM(pio) + (irqn))

#endif

#if PICO_SDK_VERSION_MAJOR < 2 || (PICO_SDK_VERSION_MAJOR == 2 && PICO_SDK_VERSION_MINOR < 1)
static void pio_sm_set_pins_with_mask64(PIO pio, uint sm, uint64_t pin_values, uint64_t pin_mask) {
    pio_sm_set_pins_with_mask(pio, sm, (uint32_t) pin_values, (uint32_t) pin_mask);
}

static void pio_sm_set_pindirs_with_mask64(PIO pio, uint sm, uint64_t pin_values, uint64_t pin_mask) {
    pio_sm_set_pindirs_with_mask(pio, sm, (uint32_t) pin_values, (uint32_t) pin_mask);
}
#endif

