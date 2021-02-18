# Invensense Embedded Motion Driver 6.12 for Cortex-M

This is a minimal copy of the core code of the motion driver download
for the sole purpose of integrating with the modm embedded library.

The specific files copied here are:

- `arm/STM32F4_MD6/Projects/eMD6/core/*` to `eMD/*`.
- `arm/STM32F4_MD6/Projects/eMD6/User/src/main.c` to `example/vendor/main.cpp`.
- `mpl libraries/arm/gcc4.9.3/*` to `eMD/mpl/lib/*`.
- `eMPL-pythonclient/*` to `eMD/pythonclient/*`.


## Porting

Modifications were made to these files:

- `eMD/driver/eMPL/inv_mpu.c` for porting.
- `eMD/driver/eMPL/inv_mpu_dmp_motion_driver.c` for porting.
- `eMD/driver/stm32L/log_stm32.c` for porting.
- `eMD/mllite/storage_manager.c` for unsigned/signed comparison.
- `eMD/pythonclient/*` for porting to Python 3.6.
- `example/vendor/main.cpp` for porting to modm and C++.

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

A modm port of these functions is provided in `modm/port/`.


## Examples

A few examples are provided in the `example` folder:

```
# Clone the modm repository into examples/.lbuild_cache
cd examples
lbuild init

# Compile with SCons or CMake
cd mpu_class
lbuild build
scons
make

# Compile with SCons or Makefile
cd ../vendor
lbuild build
scons
make
```


These examples are compiled against modm in CI for every pull request.
[![CircleCI](https://circleci.com/gh/modm-io/Invensense-eMD.svg?style=svg)](https://circleci.com/gh/modm-io/Invensense-eMD)


## License

Please note that the license from Invensense is non-standard, proprietary license,
which, however, permits distribution for the purpose of integration into non-GPL
open-source software.
