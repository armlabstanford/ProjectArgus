#pragma once

#include <stdint.h>  // use C-compatible header to avoid C++ mangling issues

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BMI270 via Bosch Sensor API using an already-open I2C fd and 7-bit address.
// Returns true (1) on success, false (0) on failure.
bool bmi270_api_init_with_fd(int fd, uint8_t i2c_addr);

// Configure accel+gyro for 200 Hz ODR and reasonable ranges/filters.
// Returns true (1) on success, false (0) on failure.
bool bmi270_api_set_acc_gyr_200hz(int fd, uint8_t i2c_addr);

#ifdef __cplusplus
} // extern "C"
#endif
