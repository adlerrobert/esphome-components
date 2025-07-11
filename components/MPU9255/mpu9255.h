#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace mpu9255 {

constexpr uint8_t MPU9255_REGISTER_MAG_address       = 0x0C; //magnetometer
constexpr uint8_t MPU9255_REGISTER_MPU_address       = 0x68; //main chip

enum MPU9255Modules
{
  MPU9255Modules_Acc_X,//accelerometer X axis
  MPU9255Modules_Acc_Y,//accelerometer Y axis
  MPU9255Modules_Acc_Z,//accelerometer Z axis
  MPU9255Modules_Gyro_X,//gyroscope X axis
  MPU9255Modules_Gyro_Y,//gyroscope Y axis
  MPU9255Modules_Gyro_Z,//gyroscope Z axis
  MPU9255Modules_Magnetometer,//magnetometer
  MPU9255Modules_Accelerometer,//accelerometer
  MPU9255Modules_Gyroscope,//gyroscope
  MPU9255Modules_Thermometer,//thermometer
  MPU9255Modules_SignalPaths,//all signal paths
};

enum MPU9255AccelerometerScales
{
  MPU9255AccelerometerScales_2g,//+-2g
  MPU9255AccelerometerScales_4g,//+-4g
  MPU9255AccelerometerScales_8g,//+-8g
  MPU9255AccelerometerScales_16g,//+-16g
};

enum MPU9255GyroscopeScales
{
  MPU9255GyroscopeScales_250dps,//+-250 degrees per second
  MPU9255GyroscopeScales_500dps,//+- 500 degrees per second
  MPU9255GyroscopeScales_1000dps,//+- 1000 degrees per second
  MPU9255GyroscopeScales_2000dps,//+- 2000 degrees per second
};

enum MPU9255AccelerometerBandwidth
{
  MPU9255AccelerometerBandwidth_1113Hz,
  MPU9255AccelerometerBandwidth_460Hz,
  MPU9255AccelerometerBandwidth_184Hz,
  MPU9255AccelerometerBandwidth_92Hz,
  MPU9255AccelerometerBandwidth_41Hz,
  MPU9255AccelerometerBandwidth_20Hz,
  MPU9255AccelerometerBandwidth_10Hz,
  MPU9255AccelerometerBandwidth_5Hz,
};

enum MPU9255GyroscopeBandwidth
{
  MPU9255GyroscopeBandwidth_8800Hz,
  MPU9255GyroscopeBandwidth_3600Hz,
  MPU9255GyroscopeBandwidth_250Hz,
  MPU9255GyroscopeBandwidth_184Hz,
  MPU9255GyroscopeBandwidth_92Hz,
  MPU9255GyroscopeBandwidth_41Hz,
  MPU9255GyroscopeBandwidth_20Hz,
  MPU9255GyroscopeBandwidth_10Hz,
  MPU9255GyroscopeBandwidth_5Hz,
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
  void set_accel_scale(MPU9255AccelerometerScales scale) { accel_scale_ = scale; }
  void set_gyro_scale(MPU9255GyroscopeScales scale) { gyro_scale_ = scale; }
  void set_accel_bandwidth(MPU9255AccelerometerBandwidth bandwidth) { accel_bandwidth_ = bandwidth; }
  void set_gyro_bandwidth(MPU9255GyroscopeBandwidth bandwidth) { gyro_bandwidth_ = bandwidth; }

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
  MPU9255AccelerometerScales accel_scale_{MPU9255AccelerometerScales_4g};
  MPU9255GyroscopeScales gyro_scale_{MPU9255GyroscopeScales_250dps};
  MPU9255AccelerometerBandwidth accel_bandwidth_{MPU9255AccelerometerBandwidth_41Hz};
  MPU9255GyroscopeBandwidth gyro_bandwidth_{MPU9255GyroscopeBandwidth_41Hz};

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
  bool reset_module_(MPU9255Modules module);
  bool enable_module_(MPU9255Modules module, bool enable);

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
  uint8_t convert_accel_scale_to_register_(uint8_t current_state, MPU9255AccelerometerScales scale);
  uint8_t convert_gyro_scale_to_register_(uint8_t current_state, MPU9255GyroscopeScales scale);
  bool set_accel_scale_(MPU9255AccelerometerScales scale);
  bool set_gyro_scale_(MPU9255GyroscopeScales scale);
  void set_accel_bandwidth_(MPU9255AccelerometerBandwidth bandwidth);
  void set_gyro_bandwidth_(MPU9255GyroscopeBandwidth bandwidth);
};

}  // namespace mpu9255
}  // namespace esphome