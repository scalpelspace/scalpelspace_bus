/*******************************************************************************
 * @file avr_llround.c
 * @brief llround() shim for AVR targets.
 *******************************************************************************
 * avr-libc provides lround() but not llround(), which the vendored
 * can_driver.c uses to round clamped physical values (can_driver.c:39).
 * lround() cannot substitute directly: 32-bit unsigned signals produce raw
 * values above LONG_MAX. On AVR double is 32-bit float, so a plain cast
 * after half-away-from-zero rounding covers the full signal range.
 *
 * Declared weak so a future avr-libc definition wins without a conflict.
 *******************************************************************************
 */

#if defined(__AVR__)

__attribute__((weak)) long long llround(double x) {
  return (long long)(x >= 0 ? (x + 0.5) : (x - 0.5));
}

#endif
