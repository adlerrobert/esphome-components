#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace mpu9255 {

constexpr uint8_t MPU9255_REGISTER_MAG_address       = 0x0C; //magnetometer
constexpr uint8_t MPU9255_REGISTER_MPU_address       = 0x68; //main chip

enum Modules
{
  MM_ACC_X,//accelerometer X axis
  MM_ACC_Y,//accelerometer Y axis
  MM_ACC_Z,//accelerometer Z axis
  MM_GYRO_X,//gyroscope X axis
  MM_GYRO_Y,//gyroscope Y axis
  MM_GYRO_Z,//gyroscope Z axis
  MM_MAG,//magnetometer
  MM_ACC,//accelerometer
  MM_GYRO,//gyroscope
  MM_THERMO,//thermometer
  MM_SIGNAL_PATHS,//all signal paths
};

enum MS_ACC
{
  MS_ACC_2G,//+-2g
  MS_ACC_4G,//+-4g
  MS_ACC_8G,//+-8g
  MS_ACC_16G,//+-16g
};

enum MS_GYRO
{
  MS_GYRO_250DPS,//+-250 degrees per second
  MS_GYRO_500DPS,//+- 500 degrees per second
  MS_GYRO_1000DPS,//+- 1000 degrees per second
  MS_GYRO_2000DPS,//+- 2000 degrees per second
};

enum MB_ACC
{
  MB_ACC_1KHZ,
  MB_ACC_460HZ,
  MB_ACC_184HZ,
  MB_ACC_92HZ,
  MB_ACC_41HZ,
  MB_ACC_20HZ,
  MB_ACC_10HZ,
  MB_ACC_5HZ,
};

enum MB_GYRO
{
  MB_GYRO_8KHZ,
  MB_GYRO_3KHZ,
  MB_GYRO_250HZ,
  MB_GYRO_184HZ,
  MB_GYRO_92HZ,
  MB_GYRO_41HZ,
  MB_GYRO_20HZ,
  MB_GYRO_10HZ,
  MB_GYRO_5HZ,
};

enum MPU9255InterruptSignalMode {
  MPU9255InterruptSignalMode_Pulse,
  MPU9255InterruptSignalMode_Latched,
};

enum MPU9255InterruptActiveState {
  MPU9255InterruptActiveState_ActiveLow,
  MPU9255InterruptActiveState_ActiveHigh,
};

enum MPU9255InterruptPinMode {
  MPU9255InterruptPinMode_OpenDrain,
  MPU9255InterruptPinMode_PushPull,
};

enum MPU9255InterruptOutputMode {
  MPU9255InterruptOutputMode_MOTION_DETECTION,
  MPU9255InterruptOutputMode_FIFO_OVERFLOW,
  MPU9255InterruptOutputMode_FSYNC,
  MPU9255InterruptOutputMode_DATA_READY,
};

class MPU9255Component : public PollingComponent, public i2c::I2CDevice {
 public:
  // general functions
  void setup() override;
  void dump_config() override;

  void update() override;
  float get_setup_priority() const override;

  // parameter setters
  void set_accel_scale(MS_ACC scale) { accel_scale_ = scale; }
  void set_gyro_scale(MS_GYRO scale) { gyro_scale_ = scale; }
  void set_accel_bandwidth(MB_ACC bandwidth) { accel_bandwidth_ = bandwidth; }
  void set_gyro_bandwidth(MB_GYRO bandwidth) { gyro_bandwidth_ = bandwidth; }

  void set_accel_offset_x(float offset) { accel_offset_x_ = offset; }
  void set_accel_offset_y(float offset) { accel_offset_y_ = offset; } 
  void set_accel_offset_z(float offset) { accel_offset_z_ = offset; }
  void set_gyro_offset_x(float offset) { gyro_offset_x_ = offset; }
  void set_gyro_offset_y(float offset) { gyro_offset_y_ = offset; }
  void set_gyro_offset_z(float offset) { gyro_offset_z_ = offset; }
  void set_mag_offset_x(float offset) { mag_offset_x_ = offset; }
  void set_mag_offset_y(float offset) { mag_offset_y_ = offset; }
  void set_mag_offset_z(float offset) { mag_offset_z_ = offset; }
  void set_mag_calibration_matrix_x(float x, float y, float z) {
    mag_calibration_matrix_x_[0] = x;
    mag_calibration_matrix_x_[1] = y;
    mag_calibration_matrix_x_[2] = z;
  }
  void set_mag_calibration_matrix_y(float x, float y, float z) {
    mag_calibration_matrix_y_[0] = x;
    mag_calibration_matrix_y_[1] = y;
    mag_calibration_matrix_y_[2] = z;
  }
  void set_mag_calibration_matrix_z(float x, float y, float z) {
    mag_calibration_matrix_z_[0] = x;
    mag_calibration_matrix_z_[1] = y;
    mag_calibration_matrix_z_[2] = z;
  }

  void set_mag_i2c_address(int address) {
    mag_i2c_address_ = address;
  }

  // sensor setterss
  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; } 
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  void set_mag_x_sensor(sensor::Sensor *mag_x_sensor) { mag_x_sensor_ = mag_x_sensor; }
  void set_mag_y_sensor(sensor::Sensor *mag_y_sensor) { mag_y_sensor_ = mag_y_sensor; }
  void set_mag_z_sensor(sensor::Sensor *mag_z_sensor) { mag_z_sensor_ = mag_z_sensor; }

protected:
  enum I2CSensor {
    I2CSensor_MPU,
    I2CSensor_Magnetometer,
  };

  // esphome sensors
  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *mag_x_sensor_{nullptr};
  sensor::Sensor *mag_y_sensor_{nullptr};
  sensor::Sensor *mag_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  // scale and bandwidth settings
  MS_ACC accel_scale_{MS_ACC_4G};
  MS_GYRO gyro_scale_{MS_GYRO_250DPS};
  MB_ACC accel_bandwidth_{MB_ACC_41HZ};
  MB_GYRO gyro_bandwidth_{MB_GYRO_41HZ};

  // offsets for accelerometer
  float accel_offset_x_{0.0f};
  float accel_offset_y_{0.0f};
  float accel_offset_z_{0.0f};

  // offsets for gyroscope
  float gyro_offset_x_{0.0f};
  float gyro_offset_y_{0.0f};
  float gyro_offset_z_{0.0f};

  // offsets for magnetometer
  float mag_offset_x_{0.0f};
  float mag_offset_y_{0.0f};
  float mag_offset_z_{0.0f};

  // calibration matrices for magnetometer
  float mag_calibration_matrix_x_[3] = {1.0f, 0.0f, 0.0f};
  float mag_calibration_matrix_y_[3] = {0.0f, 1.0f, 0.0f};
  float mag_calibration_matrix_z_[3] = {0.0f, 0.0f, 1.0f};

  // data values
  int16_t accel_x_data_{0};
  int16_t accel_y_data_{0};
  int16_t accel_z_data_{0};
  int16_t gyro_x_data_{0};
  int16_t gyro_y_data_{0};
  int16_t gyro_z_data_{0};
  int16_t mag_x_data_{0};
  int16_t mag_y_data_{0};
  int16_t mag_z_data_{0};
  int16_t accel_x_data_raw_{0};
  int16_t accel_y_data_raw_{0};
  int16_t accel_z_data_raw_{0};
  int16_t gyro_x_data_raw_{0};
  int16_t gyro_y_data_raw_{0};
  int16_t gyro_z_data_raw_{0};
  int16_t mag_x_data_raw_{0};
  int16_t mag_y_data_raw_{0};
  int16_t mag_z_data_raw_{0};
  int16_t temperature_data_{0};

  // magnetometer I2C address
  int mag_i2c_address_{0x0C};

  // I2C interface for magnetometer
  i2c::I2CDevice mag_i2c_;

  // sensitivity values for magnetometer
  double mag_x_sensitivity_{1.0f};
  double mag_y_sensitivity_{1.0f};
  double mag_z_sensitivity_{1.0f};

  // offsets for accelerometer and gyroscope
  int16_t accel_x_offset_internal_{0};
  int16_t accel_y_offset_internal_{0};
  int16_t accel_z_offset_internal_{0};
  int16_t gyro_x_offset_internal_{0};
  int16_t gyro_y_offset_internal_{0};
  int16_t gyro_z_offset_internal_{0};

  // I2C read/write functions
  bool read_byte_(I2CSensor sensor, uint8_t a_register, uint8_t *data);
  bool write_byte_(I2CSensor sensor, uint8_t a_register, uint8_t data);
  bool read_bytes_(I2CSensor sensor, uint8_t a_register, uint8_t *data, size_t len);
  bool write_byte_OR_(I2CSensor sensor, uint8_t a_register, uint8_t data);
  bool write_byte_AND_(I2CSensor sensor, uint8_t a_register, uint8_t data);

  // power management functions
  bool hardware_reset_();
  bool sleep_enable_(bool enable);
  bool reset_module_(Modules module);
  bool enable_module_(Modules module, bool enable);

  // interrupt functions
  bool set_interupt_signal_mode_(MPU9255InterruptSignalMode mode);
  bool set_interrupt_active_state_(MPU9255InterruptActiveState state);
  bool set_interrupt_pin_mode_(MPU9255InterruptPinMode mode);
  bool enable_interrupt_output_mode_(MPU9255InterruptOutputMode mode, bool enable);
  bool set_interrupt_motion_detection_threshold_(uint8_t threshold);
  bool enable_interrupt_motion_detection_(bool enable);
  bool clear_interrupts_();
  
  // data update functions
  bool update_accel_data_();
  bool update_gyro_data_();
  bool update_mag_data_();
  bool update_temperature_data_();

  // sensor initialization
  bool sensor_init_();
  bool testIMU_();
  bool testMag_();

  // scale and bandwidth setter functions
  uint8_t convert_accel_scale_to_register_(uint8_t current_state, MS_ACC scale);
  uint8_t convert_gyro_scale_to_register_(uint8_t current_state, MS_GYRO scale);
  bool set_accel_scale_(MS_ACC scale);
  bool set_gyro_scale_(MS_GYRO scale);
  void set_accel_bandwidth_(MB_ACC bandwidth);
  void set_gyro_bandwidth_(MB_GYRO bandwidth);
};

}  // namespace mpu9255
}  // namespace esphome