#include "mpu9255.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mpu9255 {

static const char *const TAG = "mpu9255.sensor";
constexpr float GRAVITY = 9.80665f;
constexpr float MAGNETOMETER_CAL = 0.06;

//main chip
constexpr uint8_t MPU9255_REGISTER_USER_CTRL         = 0x6A;
constexpr uint8_t MPU9255_REGISTER_PWR_MGMT_1        = 0x6B;
constexpr uint8_t MPU9255_REGISTER_PWR_MGMT_2        = 0x6C;
constexpr uint8_t MPU9255_REGISTER_SIGNAL_PATH_RESET = 0x68;
constexpr uint8_t MPU9255_REGISTER_INT_PIN_CFG       = 0x37;
constexpr uint8_t MPU9255_REGISTER_ST1               = 0x02;
constexpr uint8_t MPU9255_REGISTER_ACCEL_CONFIG      = 0x1C;
constexpr uint8_t MPU9255_REGISTER_ACCEL_CONFIG_2    = 0x1D;
constexpr uint8_t MPU9255_REGISTER_MOT_DETECT_CTRL   = 0x69;
constexpr uint8_t MPU9255_REGISTER_WOM_THR           = 0x1F;
constexpr uint8_t MPU9255_REGISTER_GYRO_CONFIG       = 0x1B;
constexpr uint8_t MPU9255_REGISTER_CONFIG            = 0x1A;
constexpr uint8_t MPU9255_REGISTER_SMPLRT_DIV        = 0x19;
constexpr uint8_t MPU9255_REGISTER_INT_ENABLE        = 0x38;
constexpr uint8_t MPU9255_REGISTER_INT_STATUS        = 0x3A;
constexpr uint8_t MPU9255_REGISTER_WHO_AM_I          = 0x75;

//gyroscope offset
constexpr uint8_t MPU9255_REGISTER_XG_OFFSET_H       = 0x13;
constexpr uint8_t MPU9255_REGISTER_XG_OFFSET_L       = 0x14;
constexpr uint8_t MPU9255_REGISTER_YG_OFFSET_H       = 0x15;
constexpr uint8_t MPU9255_REGISTER_YG_OFFSET_L       = 0x16;
constexpr uint8_t MPU9255_REGISTER_ZG_OFFSET_H       = 0x17;
constexpr uint8_t MPU9255_REGISTER_ZG_OFFSET_L       = 0x18;

//accelerometer offset
constexpr uint8_t MPU9255_REGISTER_XA_OFFSET_H       = 0x77;
constexpr uint8_t MPU9255_REGISTER_XA_OFFSET_L       = 0x78;
constexpr uint8_t MPU9255_REGISTER_YA_OFFSET_H       = 0x7A;
constexpr uint8_t MPU9255_REGISTER_YA_OFFSET_L       = 0x7B;
constexpr uint8_t MPU9255_REGISTER_ZA_OFFSET_H       = 0x7D;
constexpr uint8_t MPU9255_REGISTER_ZA_OFFSET_L       = 0x7E;

//magnetometer
constexpr uint8_t MPU9255_REGISTER_MAG_ID            = 0x00;
constexpr uint8_t MPU9255_REGISTER_CNTL              = 0x0A;
constexpr uint8_t MPU9255_REGISTER_CNTL2             = 0x0B;
constexpr uint8_t MPU9255_REGISTER_ASAX              = 0x10;
constexpr uint8_t MPU9255_REGISTER_ASAY              = 0x11;
constexpr uint8_t MPU9255_REGISTER_ASAZ              = 0x12;

/// data registers
constexpr uint8_t MPU9255_REGISTER_MAG_XOUT_L        = 0x03;//magnetometer
constexpr uint8_t MPU9255_REGISTER_GYRO_XOUT_H       = 0x43;//gyro
constexpr uint8_t MPU9255_REGISTER_ACCEL_XOUT_H      = 0x3B;//accelerometer
constexpr uint8_t MPU9255_REGISTER_TEMP_OUT_H        = 0x41;//thermometer

inline uint16_t combine_bytes(uint8_t msb, uint8_t lsb) { return ((msb & 0xFF) << 8) | (lsb & 0xFF); }

void MPU9255Component::setup() {
  ESP_LOGCONFIG(TAG, "Running setup");
  // init the MPU9255 sensor
  if(!sensor_init_()) {
    ESP_LOGE(TAG, "Failed to initialize MPU9255 sensor");
    this->mark_failed();
    return;
  }

  // scale accelerometer
  set_accel_scale(accel_scale_);
  // scale gyroscope
  set_gyro_scale(gyro_scale_);
  // set accelerometer bandwidth
  set_accel_bandwidth(accel_bandwidth_);
  // set gyroscope bandwidth
  set_gyro_bandwidth(gyro_bandwidth_);

  // configure all sensor objects
  enable_module_(MM_ACC_X, this->accel_x_sensor_ != nullptr);
  enable_module_(MM_ACC_Y, this->accel_y_sensor_ != nullptr);
  enable_module_(MM_ACC_Z, this->accel_z_sensor_ != nullptr);
  enable_module_(MM_GYRO_X, this->gyro_x_sensor_ != nullptr);
  enable_module_(MM_GYRO_Y, this->gyro_y_sensor_ != nullptr);
  enable_module_(MM_GYRO_Z, this->gyro_z_sensor_ != nullptr);
  enable_module_(MM_THERMO, this->temperature_sensor_ != nullptr);
  enable_module_(MM_MAG, this->mag_x_sensor_ != nullptr ||
                                                this->mag_y_sensor_ != nullptr ||
                                                this->mag_z_sensor_ != nullptr);
}

void MPU9255Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MPU9255:");
  ESP_LOGCONFIG(TAG, "  Address MPU: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Address MAG: 0x%02X", this->mag_i2c_.get_i2c_address());
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void MPU9255Component::update() {
  if (!update_accel_data_()) {
    this->status_set_warning();
    return;
  }

  if (!update_gyro_data_()) {
    this->status_set_warning();
    return;
  }

  if (!update_mag_data_()) {
    this->status_set_warning();
    return;
  }

  if (!update_temperature_data_()) {
    this->status_set_warning();
    return;
  }

  this->status_clear_warning();
}

float MPU9255Component::get_setup_priority() const { return setup_priority::DATA; }

bool MPU9255Component::sensor_init_() {
  hardware_reset_();//reset the chip

  write_byte_(I2CSensor_MPU, MPU9255_REGISTER_CONFIG, 0x03);//set DLPF_CFG to 0b11, so 41 Hz bandwidth
  write_byte_(I2CSensor_MPU, MPU9255_REGISTER_SMPLRT_DIV, 0x00);//set prescaler sample rate to 0, so 1 kHz sample rate
  write_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_PIN_CFG, 0x02);//enable bypass in order to access the magnetometer via I2C

  // setup magnetometer
  this->mag_i2c_.set_i2c_bus(this->bus_);
  this->mag_i2c_.set_i2c_address(this->mag_i2c_address_);

  mag_i2c_.write_byte(MPU9255_REGISTER_CNTL, 0x16);//set magnetometer to read in mode 2 and enable 16 bit measurements

  //read magnetometer sensitivity
  uint8_t temp_asa;
  if (!mag_i2c_.read_byte(MPU9255_REGISTER_ASAX, &temp_asa)) {
    ESP_LOGE(TAG, "Failed to read magnetometer x sensitivity");
    return false;
  }
  mag_x_sensitivity_ = (((temp_asa-128)*0.5)/128)+1;

  if (!mag_i2c_.read_byte(MPU9255_REGISTER_ASAY, &temp_asa)) {
    ESP_LOGE(TAG, "Failed to read magnetometer y sensitivity");
    return false;
  }
  mag_y_sensitivity_ = (((temp_asa-128)*0.5)/128)+1;

  if (!mag_i2c_.read_byte(MPU9255_REGISTER_ASAZ, &temp_asa)) {
    ESP_LOGE(TAG, "Failed to read magnetometer z sensitivity");
    return false;
  }
  mag_z_sensitivity_ = (((temp_asa-128)*0.5)/128)+1;

  //read factory gyroscope offset
  uint8_t g_offset_h, g_offset_l;
  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_XG_OFFSET_H, &g_offset_h) ||
      !read_byte_(I2CSensor_MPU, MPU9255_REGISTER_XG_OFFSET_L, &g_offset_l)) {
    ESP_LOGE(TAG, "Failed to read gyroscope X offset");
    return false;
  }
  gyro_x_offset_internal_ = combine_bytes(g_offset_h, g_offset_l);

  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_YG_OFFSET_H, &g_offset_h) ||
      !read_byte_(I2CSensor_MPU, MPU9255_REGISTER_YG_OFFSET_L, &g_offset_l)) {
    ESP_LOGE(TAG, "Failed to read gyroscope Y offset");
    return false;
  }
  gyro_y_offset_internal_ = combine_bytes(g_offset_h, g_offset_l);

  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_ZG_OFFSET_H, &g_offset_h) ||
      !read_byte_(I2CSensor_MPU, MPU9255_REGISTER_ZG_OFFSET_L, &g_offset_l)) {
    ESP_LOGE(TAG, "Failed to read gyroscope Z offset");
    return false;
  }
  gyro_z_offset_internal_ = combine_bytes(g_offset_h, g_offset_l);

  //read the register values and save them as a 16 bit value
  uint8_t a_offset_h, a_offset_l;
  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_XA_OFFSET_H, &a_offset_h) ||
      !read_byte_(I2CSensor_MPU, MPU9255_REGISTER_XA_OFFSET_L, &a_offset_l)) {
    ESP_LOGE(TAG, "Failed to read accelerometer X offset");
    return false;
  }
  accel_x_offset_internal_ = combine_bytes(a_offset_h, a_offset_l);

  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_YA_OFFSET_H, &a_offset_h) ||
      !read_byte_(I2CSensor_MPU, MPU9255_REGISTER_YA_OFFSET_L, &a_offset_l)) {
    ESP_LOGE(TAG, "Failed to read accelerometer Y offset");
    return false;
  }
  accel_y_offset_internal_ = combine_bytes(a_offset_h, a_offset_l);

  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_ZA_OFFSET_H, &a_offset_h) ||
      !read_byte_(I2CSensor_MPU, MPU9255_REGISTER_ZA_OFFSET_L, &a_offset_l)) {
    ESP_LOGE(TAG, "Failed to read accelerometer Z offset");
    return false;
  }
  accel_z_offset_internal_ = combine_bytes(a_offset_h, a_offset_l);
  
  //shift offset values to the right to remove the LSB
  accel_x_offset_internal_ >>= 1;
  accel_y_offset_internal_ >>= 1;
  accel_z_offset_internal_ >>= 1;

  return (testIMU_() || testMag_());
}

// helper functions ////////////////////////////////////////////////////////
bool MPU9255Component::testIMU_() {
  uint8_t who_am_i;
  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_WHO_AM_I, &who_am_i) || (who_am_i != 0x68 && who_am_i != 0x70 && who_am_i != 0x98)) {
    ESP_LOGE(TAG, "Failed to read WHO_AM_I register or invalid value: %02X", who_am_i);
    return false;
  }
  return true;
} 

bool MPU9255Component::testMag_() {
  uint8_t who_am_i;
  if (!mag_i2c_.read_byte(MPU9255_REGISTER_MAG_ID, &who_am_i) || (who_am_i == 0xFF)) {
    ESP_LOGE(TAG, "Failed to read magnetometer ID at address %02X", mag_i2c_address_);
    return false;
  }
  
  if (who_am_i != 0x48) { // AK8963 magnetometer ID
    ESP_LOGE(TAG, "Invalid magnetometer ID: %02X", who_am_i);
    return false;
  }
  
  return true;
} 

// Internal functions /////////////////////////////////////////////////////////
bool MPU9255Component::hardware_reset_() {
  return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_1, 0x80);
}

bool MPU9255Component::sleep_enable_(bool enable) {
  if (enable) {
    return write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_1, 1 << 6);
  } else {
    return write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_1, ~(1 << 6));
  }
}

bool MPU9255Component::reset_module_(Modules module) {
  switch (module) {
    case Modules::MM_ACC:
      return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_SIGNAL_PATH_RESET, 1 << 1); // Reset accelerometer
    case Modules::MM_GYRO:
      return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_SIGNAL_PATH_RESET, 1 << 2); // Reset gyroscope
    case Modules::MM_THERMO:
      return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_SIGNAL_PATH_RESET, 1 << 0); // Reset thermometer
    case Modules::MM_SIGNAL_PATHS:
      return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_USER_CTRL, 1 << 0); // Reset all signal paths
    case Modules::MM_MAG:
      return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_CNTL2, 1 << 0); // Reset magnetometer
    default:
      return false; // Unsupported module
  }
}

bool MPU9255Component::enable_module_(Modules module, bool enable) {
  switch (module)
  {
    case MM_ACC_X:
      return enable ? write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, ~(1<<5)) : write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, 1<<5);
      break;

    case MM_ACC_Y:
      return enable ? write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, ~(1<<4)) : write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, 1<<4);
      break;

    case MM_ACC_Z:
      return enable ? write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, ~(1<<3)) : write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, 1<<3);
      break;

    case MM_GYRO_X:
      return enable ? write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, ~(1<<2)) : write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, 1<<2);
      break;

    case MM_GYRO_Y:
      return enable ? write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, ~(1<<1)) : write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, 1<<1);
      break;

    case MM_GYRO_Z:
      return enable ? write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, ~(1<<0)) : write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_PWR_MGMT_2, 1<<0);
      break;

    case MM_MAG:
      return enable ? write_byte_(I2CSensor_MPU, MPU9255_REGISTER_CNTL2, 0x16) : write_byte_(I2CSensor_MPU, MPU9255_REGISTER_CNTL2, 0x00);
      break;
  }

  return false; // Unsupported module
}

// Interrupt functions ////////////////////////////////////////////////////////
bool MPU9255Component::set_interupt_signal_mode_(MPU9255InterruptSignalMode mode)  {
  return mode == MPU9255InterruptSignalMode_Pulse ? 
    write_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_PIN_CFG, ~(1<<5)) : // Pulse mode
    write_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_PIN_CFG, 1<<5); // Latched mode
}

bool MPU9255Component::set_interrupt_active_state_(MPU9255InterruptActiveState state) {
  return state == MPU9255InterruptActiveState_ActiveLow ? 
    write_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_PIN_CFG, 1<<7) : // Active low
    write_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_PIN_CFG, 0x7F); // Active high ~(1<<7)
}

bool MPU9255Component::set_interrupt_pin_mode_(MPU9255InterruptPinMode mode) {
  return mode == MPU9255InterruptPinMode_OpenDrain ? 
    write_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_PIN_CFG, 1<<6) : // Open drain
    write_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_PIN_CFG, ~(1<<6)); // Push-pull
}

bool MPU9255Component::enable_interrupt_output_mode_(MPU9255InterruptOutputMode mode, bool enable) {
  uint8_t reg;
  switch (mode) {
    case MPU9255InterruptOutputMode_MOTION_DETECTION:
      reg = 1 << 6; // Bit 7 for motion detection
      break;
    case MPU9255InterruptOutputMode_FIFO_OVERFLOW:
      reg = 1 << 4; // Bit 4 for FIFO overflow
      break;
    case MPU9255InterruptOutputMode_FSYNC:
      reg = 1 << 3; // Bit 3 for FSYNC
      break;
    case MPU9255InterruptOutputMode_DATA_READY:
      reg = 1 << 0; // Bit 0 for data ready
      break;
    default:
      return false; // Unsupported mode
  }
  
  return enable ? write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_INT_ENABLE, reg) : write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_INT_ENABLE, ~reg);
}

bool MPU9255Component::set_interrupt_motion_detection_threshold_(uint8_t threshold) {
  return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_WOM_THR, threshold & 0x7F);
}

bool MPU9255Component::enable_interrupt_motion_detection_(bool enable) {
  return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_MOT_DETECT_CTRL, enable ? (1<<7) : ~(1<<7));
}

bool MPU9255Component::clear_interrupts_() {
  return read_byte_(I2CSensor_MPU, MPU9255_REGISTER_INT_STATUS, nullptr); // Reading the interrupt status register clears the interrupts
}

// data update functions ////////////////////////////////////////////////////
bool MPU9255Component::update_accel_data_() {
  if (this->accel_x_sensor_ == nullptr && this->accel_y_sensor_ == nullptr && this->accel_z_sensor_ == nullptr) {
    return true; // No accelerometer sensors to update
  }

  uint8_t raw_data[6];
  if (!this->read_bytes_(I2CSensor_MPU, MPU9255_REGISTER_ACCEL_XOUT_H, raw_data, 6)) {
    this->status_set_warning();
    return false;
  }

  // Extract raw 16-bit values
  this->accel_x_data_raw_ = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  this->accel_y_data_raw_ = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  this->accel_z_data_raw_ = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  this->accel_x_data_raw_ -= this->accel_offset_x_;
  this->accel_y_data_raw_ -= this->accel_offset_y_;
  this->accel_z_data_raw_ -= this->accel_offset_z_;

  // Select scale factor based on configured range
  float lsb_per_g = 16384.0f; // default for ±2g
  switch (this->accel_scale_) {
    case MS_ACC_2G:
      lsb_per_g = 16384.0f;
      break;
    case MS_ACC_4G:
      lsb_per_g = 8192.0f;
      break;
    case MS_ACC_8G:
      lsb_per_g = 4096.0f;
      break;
    case MS_ACC_16G:
      lsb_per_g = 2048.0f;
      break;
  }

  // Convert to g and then to m/s²
  this->accel_x_data_ = (this->accel_x_data_raw_ / lsb_per_g) * GRAVITY;
  this->accel_y_data_ = (this->accel_y_data_raw_ / lsb_per_g) * GRAVITY;
  this->accel_z_data_ = (this->accel_z_data_raw_ / lsb_per_g) * GRAVITY;

  // Publish converted values
  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(this->accel_x_data_);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(this->accel_y_data_);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(this->accel_z_data_);

  return true;
}

bool MPU9255Component::update_gyro_data_() {
  if (this->gyro_x_sensor_ == nullptr && this->gyro_y_sensor_ == nullptr && this->gyro_z_sensor_ == nullptr) {
    return true; // No gyroscope sensors to update
  }

  uint8_t raw_data[6];
  if (!this->read_bytes_(I2CSensor_MPU, MPU9255_REGISTER_GYRO_XOUT_H, raw_data, 6)) {
    this->status_set_warning();
    return false;
  }

  // Extract raw 16-bit values
  this->gyro_x_data_raw_ = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  this->gyro_y_data_raw_ = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  this->gyro_z_data_raw_ = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  this->gyro_x_data_raw_ -= this->gyro_offset_x_;
  this->gyro_y_data_raw_ -= this->gyro_offset_y_;
  this->gyro_z_data_raw_ -= this->gyro_offset_z_;

  // Select scale factor based on configured range
  float dps_per_digit = 131.0f; // default for ±250dps
  switch (this->gyro_scale_) {
    case MS_GYRO_250DPS:
      dps_per_digit = 131.0f;
      break;
    case MS_GYRO_500DPS:
      dps_per_digit = 65.5f;
      break;
    case MS_GYRO_1000DPS:
      dps_per_digit = 32.8f;
      break;
    case MS_GYRO_2000DPS:
      dps_per_digit = 16.4f;
      break;
  }

  // Convert to degrees per second
  this->gyro_x_data_ = this->gyro_x_data_raw_ / dps_per_digit;
  this->gyro_y_data_ = this->gyro_y_data_raw_ / dps_per_digit;
  this->gyro_z_data_ = this->gyro_z_data_raw_ / dps_per_digit;

  // Publish converted values
  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(this->gyro_x_data_);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(this->gyro_y_data_);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(this->gyro_z_data_);

  return true;
}

bool MPU9255Component::update_mag_data_() {
  if (this->mag_x_sensor_ == nullptr && this->mag_y_sensor_ == nullptr && this->mag_z_sensor_ == nullptr) {
    return true; // No magnetometer sensors to update
  }

  uint8_t raw_data[6];
  if (!this->read_bytes_(I2CSensor_MPU, MPU9255_REGISTER_MAG_XOUT_L, raw_data, 8)) {
    this->status_set_warning();
    return false;
  }

  // Extract raw 16-bit values
  this->mag_x_data_raw_ = (int16_t)((raw_data[1] << 8) | raw_data[0]);
  this->mag_y_data_raw_ = (int16_t)((raw_data[3] << 8) | raw_data[2]);
  this->mag_z_data_raw_ = (int16_t)((raw_data[5] << 8) | raw_data[4]);

  this->mag_x_data_raw_ -= mag_offset_x_;
  this->mag_y_data_raw_ -= mag_offset_y_;
  this->mag_z_data_raw_ -= mag_offset_z_;

  float calibrated_mx = mag_calibration_matrix_x_[0] * mag_x_data_raw_ +
                        mag_calibration_matrix_x_[1] * mag_y_data_raw_ +
                        mag_calibration_matrix_x_[2] * mag_z_data_raw_;

  float calibrated_my = mag_calibration_matrix_y_[0] * this->mag_x_data_raw_ +
                        mag_calibration_matrix_y_[1] * mag_y_data_raw_ +
                        mag_calibration_matrix_y_[2] * mag_z_data_raw_;

  float calibrated_mz = mag_calibration_matrix_z_[0] * this->mag_x_data_raw_ +
                        mag_calibration_matrix_z_[1] * mag_y_data_raw_ +
                        mag_calibration_matrix_z_[2] * mag_z_data_raw_;

  // Apply sensitivity scaling
  this->mag_x_data_ = (calibrated_mx * this->mag_x_sensitivity_)/0.6;
  this->mag_y_data_ = (calibrated_my * this->mag_y_sensitivity_)/0.6;
  this->mag_z_data_ = (calibrated_mz * this->mag_z_sensitivity_)/0.6;

  // Publish converted values
  if (this->mag_x_sensor_ != nullptr)
    this->mag_x_sensor_->publish_state(this->mag_x_data_);
  if (this->mag_y_sensor_ != nullptr)
    this->mag_y_sensor_->publish_state(this->mag_y_data_);
  if (this->mag_z_sensor_ != nullptr)
    this->mag_z_sensor_->publish_state(this->mag_z_data_);

  return true;
}

bool MPU9255Component::update_temperature_data_() {
  if (this->temperature_sensor_ == nullptr) {
    return true; // No temperature sensor to update
  }

  uint8_t raw_data[2];
  if (!this->read_bytes_(I2CSensor_MPU, MPU9255_REGISTER_TEMP_OUT_H, raw_data, 2)) {
    this->status_set_warning();
    return false;
  }

  // Extract raw 16-bit value
  int16_t temp_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);

  // Convert to Celsius
  this->temperature_data_ = temp_raw; // (temp_raw / 340.0f) + 36.53f;

  // Publish converted value
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(this->temperature_data_);

  return true;
}

// Scale conversion functions ////////////////////////////////////////////////
uint8_t MPU9255Component::convert_accel_scale_to_register_(uint8_t current_state, MS_ACC scale) {
  const uint8_t accel_masks[] = {0x00, 0x01, 0x02, 0x03}; // << 3 later
  return (current_state & ~(0x03 << 3)) | (accel_masks[scale] << 3);
}

uint8_t MPU9255Component::convert_gyro_scale_to_register_(uint8_t current_state, MS_GYRO scale) {
  const uint8_t gyro_masks[] = {0x00, 0x01, 0x02, 0x03}; // << 3 later
  return (current_state & ~(0x03 << 3)) | (gyro_masks[scale] << 3);
}

bool MPU9255Component::set_accel_scale_(MS_ACC scale) {
  uint8_t current_state;
  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_ACCEL_CONFIG, &current_state)) {
    ESP_LOGE("MPU9255", "Failed to read accelerometer configuration register");
    return false;
  }
  current_state = convert_accel_scale_to_register_(current_state, scale);
  return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_ACCEL_CONFIG, current_state);
}

bool MPU9255Component::set_gyro_scale_(MS_GYRO scale) {
  uint8_t current_state;
  if (!read_byte_(I2CSensor_MPU, MPU9255_REGISTER_GYRO_CONFIG, &current_state)) {
    ESP_LOGE("MPU9255", "Failed to read gyroscope configuration register");
    return false;
  }
  current_state = convert_gyro_scale_to_register_(current_state, scale);
  return write_byte_(I2CSensor_MPU, MPU9255_REGISTER_GYRO_CONFIG, current_state);
}

void MPU9255Component::set_accel_bandwidth_(MB_ACC bandwidth) {
  const uint8_t accel_bandwidth_masks[] = { 0x00,  0x01,  0x02,  0x03,  0x04,  0x05,  0x06,  0x07};
  // Clear lower 4 bits (fchoice_b and DLPF_CFG)
  write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_ACCEL_CONFIG_2, ~0x0F);

  if (bandwidth == MB_ACC_1KHZ) {
    // Set accel_fchoice_b = 1 (bit 3)
    write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_ACCEL_CONFIG_2, (1 << 3));
  } else {
    // Set A_DLPF_CFG using mask from lookup table (bits 0–2)
    write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_ACCEL_CONFIG_2, accel_bandwidth_masks[bandwidth-1]);
  }
}

void MPU9255Component::set_gyro_bandwidth_(MB_GYRO bandwidth) {
  const uint8_t gyro_bandwidth_masks[] = { 0x00,  0x01,  0x02,  0x03,  0x04,  0x05,  0x06,  0x07};
  // Clear lower 3 bits (DLPF_CFG)
  write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_GYRO_CONFIG, ~0x03);

  if (bandwidth == MB_GYRO_8KHZ) {
    // Fchoice_b = 01 → bit 0 = 1
    write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_GYRO_CONFIG, (1 << 0));
  }
  else if (bandwidth == MB_GYRO_3KHZ) {
    // Fchoice_b = 10 → bit 1 = 1
    write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_GYRO_CONFIG, (1 << 1));
  }
  else {
    // Fchoice_b = 00 (already cleared)
    const uint8_t index = bandwidth - 2;

    // Clear DLPF_CFG bits (0–2) in CONFIG
    write_byte_AND_(I2CSensor_MPU, MPU9255_REGISTER_CONFIG, ~0x07);

    // Set DLPF_CFG bits (0–2) using lookup
    write_byte_OR_(I2CSensor_MPU, MPU9255_REGISTER_CONFIG, gyro_bandwidth_masks[index]);
  }
}

// I2C read/write functions //////////////////////////////////////////////////
bool MPU9255Component::read_byte_(I2CSensor sensor, uint8_t a_register, uint8_t *data) {
  if (sensor == I2CSensor_Magnetometer) {
    return this->mag_i2c_.read_byte(a_register, data);
  }
  return I2CDevice::read_byte(a_register, data);
};

bool MPU9255Component::write_byte_(I2CSensor sensor, uint8_t a_register, uint8_t data) {
  if (sensor == I2CSensor_Magnetometer) {
    return this->mag_i2c_.write_byte(a_register, data);
  }
  return I2CDevice::write_byte(a_register, data);
};

bool MPU9255Component::read_bytes_(I2CSensor sensor, uint8_t a_register, uint8_t *data, size_t len) {
  if (sensor == I2CSensor_Magnetometer) {
    return this->mag_i2c_.read_bytes(a_register, data, len);
  }
  return I2CDevice::read_bytes(a_register, data, len);
};

bool MPU9255Component::write_byte_OR_(I2CSensor sensor, uint8_t a_register, uint8_t data) {
  uint8_t current_value;
  if (!read_byte_(sensor, a_register, &current_value))
    return false;
  return write_byte_(sensor, a_register, current_value|data);
};

bool MPU9255Component::write_byte_AND_(I2CSensor sensor, uint8_t a_register, uint8_t data) {
  uint8_t current_value;
  if (!read_byte_(sensor, a_register, &current_value))
    return false;
  return write_byte_(sensor, a_register, current_value&data);
};

}  // namespace mpu9255
}  // namespace esphome