# Invensense Embedded Motion Driver 6.12 for Cortex-M

This is an minimal copy of the core code of the motion driver download
for the sole purpose of integrating with the modm embedded library.

The specific folders copied here are:

- `arm/STM32F4_MD6/Projects/eMD6/core/` to `core/`.
- `mpl libraries/arm/gcc4.9.3` to `lib/arm/`

Minimal modifications were made to these files:

- `core/driver/eMPL/inv_mpu.c` for porting.
- `core/driver/eMPL/inv_mpu_dmp_motion_driver.c` for porting.
- `core/driver/stm32L/log_stm32.c` for porting.
- `core/mllite/storage_manager.c` for unsigned/signed comparison.

The changes for porting only consist out of extern declaring the following
functions, which must be ported for the specific platform.

```c
// Write to {slave_addr}: one byte {reg_addr}, followed by write of {length} bytes from {data}
extern int inv_i2c_write(uint8_t slave_addr, uint8_t reg_addr,
                         uint8_t length, uint8_t const *data);
// Write+Read to {slave_addr}: one byte {reg_addr}, followed by read of {length} bytes to {data}
extern int inv_i2c_read(uint8_t slave_addr, uint8_t reg_addr,
                        uint8_t length, uint8_t *data);

// Blocks for {num_ms} milliseconds
extern void inv_delay_ms(uint32_t num_ms);
// Writes the current value of the millisecond counter to {count}
extern void inv_get_ms(uint32_t *count);

// vsnprintf replacement: for logging output
extern int inv_vsnprintf(char *s, size_t n, const char *fmt, va_list args);
// fputc replacement: for logging output
extern int inv_fputc(int s);
```

Please note that the license from Invensense is non-standard, proprietary license,
which, however, permits distribution for the purpose of integration into non-GPL
open-source software.
