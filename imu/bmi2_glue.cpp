#include <vector>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

// Bosch BMI2/BMI270 C API (C headers)
extern "C" {
  #include "../external/BMI270_SensorAPI/bmi2.h"
  #include "../external/BMI270_SensorAPI/bmi270.h"
}

// Our glue header exports C linkage for the two functions we provide
#include "bmi2_glue.h"

// Small holder for our I2C context passed into Bosch driver callbacks
struct I2CIntf {
  int fd;
  uint8_t addr;
};

static int8_t user_i2c_reg_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
  I2CIntf* ctx = static_cast<I2CIntf*>(intf_ptr);

  i2c_rdwr_ioctl_data rdwr{};
  i2c_msg msgs[2]{};

  msgs[0].addr  = ctx->addr;
  msgs[0].flags = 0;               // write
  msgs[0].len   = 1;
  msgs[0].buf   = &reg_addr;

  msgs[1].addr  = ctx->addr;
  msgs[1].flags = I2C_M_RD;        // read
  msgs[1].len   = static_cast<__u16>(len);
  msgs[1].buf   = data;

  rdwr.msgs  = msgs;
  rdwr.nmsgs = 2;

  if (ioctl(ctx->fd, I2C_RDWR, &rdwr) < 0)
    return BMI2_E_COM_FAIL;

  return BMI2_OK;
}

static int8_t user_i2c_reg_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
  I2CIntf* ctx = static_cast<I2CIntf*>(intf_ptr);

  std::vector<uint8_t> buf;
  buf.reserve(len + 1);
  buf.push_back(reg_addr);
  buf.insert(buf.end(), data, data + len);

  i2c_rdwr_ioctl_data rdwr{};
  i2c_msg msg{};

  msg.addr  = ctx->addr;
  msg.flags = 0;                   // write
  msg.len   = static_cast<__u16>(buf.size());
  msg.buf   = buf.data();

  rdwr.msgs  = &msg;
  rdwr.nmsgs = 1;

  if (ioctl(ctx->fd, I2C_RDWR, &rdwr) < 0)
    return BMI2_E_COM_FAIL;

  return BMI2_OK;
}

static void user_delay_us(uint32_t period, void * /*intf_ptr*/) {
  usleep(period);
}

// The two glue functions are exported with C linkage per header.
// Implement them in C++ here; the header takes care of the extern "C".
bool bmi270_api_init_with_fd(int fd, uint8_t i2c_addr) {
  // Ensure fd is set to the target 7-bit address for subsequent transfers
  if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0)
    return false;

  I2CIntf intf{fd, i2c_addr};

  bmi2_dev dev{};
  dev.intf           = BMI2_I2C_INTF;
  dev.read           = user_i2c_reg_read;
  dev.write          = user_i2c_reg_write;
  dev.delay_us       = user_delay_us;
  dev.intf_ptr       = &intf;
  dev.read_write_len = 32;        // safe default

  // Loads embedded config and returns BMI2_OK on success
  const int8_t rslt = bmi270_init(&dev);
  return (rslt == BMI2_OK);
}

bool bmi270_api_set_acc_gyr_200hz(int fd, uint8_t i2c_addr) {
  if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0)
    return false;

  I2CIntf intf{fd, i2c_addr};

  bmi2_dev dev{};
  dev.intf           = BMI2_I2C_INTF;
  dev.read           = user_i2c_reg_read;
  dev.write          = user_i2c_reg_write;
  dev.delay_us       = user_delay_us;
  dev.intf_ptr       = &intf;
  dev.read_write_len = 32;

  // Enable accel + gyro
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  if (bmi2_sensor_enable(sens_list, 2, &dev) != BMI2_OK)
    return false;

  // Configure both sensors @200Hz
  bmi2_sens_config cfg[2]{};

  cfg[0].type            = BMI2_ACCEL;
  cfg[0].cfg.acc.odr     = BMI2_ACC_ODR_200HZ;
  cfg[0].cfg.acc.range   = BMI2_ACC_RANGE_2G;
  cfg[0].cfg.acc.bwp     = BMI2_ACC_NORMAL_AVG4;
  cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

  cfg[1].type            = BMI2_GYRO;
  cfg[1].cfg.gyr.odr     = BMI2_GYR_ODR_200HZ;
  cfg[1].cfg.gyr.range   = BMI2_GYR_RANGE_1000;
  cfg[1].cfg.gyr.bwp     = BMI2_GYR_NORMAL_MODE;
  cfg[1].cfg.gyr.noise_perf  = BMI2_PERF_OPT_MODE;
  cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  if (bmi2_set_sensor_config(cfg, 2, &dev) != BMI2_OK)
    return false;

  return true;
}
