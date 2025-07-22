#include "icm20948.h"
#include "esp_timer.h"

#include "AK09916_ENUMERATIONS.h"
#include "AK09916_REGISTERS.h"
#include "ICM_20948_REGISTERS.h"
#include "ICM_20948_ENUMERATIONS.h"
#include "ICM_20948_IMG.DMP3A.h"
#include "ICM_20948_DMP.h"

#include <format>

#define MAX_MAGNETOMETER_STARTS 10

#define BIAS_STORE_PREF_KEY "iKYM4Rqc2EhHLxEV2gZLz86hpYYQaIxlmIfR0WW22Kap5MXE2lg4T0DSFYVaqpO"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace esphome {
namespace icm20948 {

// ICM-20948 data is big-endian. We need to make it little-endian when writing into icm_20948_DMP_data_t
const int DMP_Quat9_Byte_Ordering[icm_20948_DMP_Quat9_Bytes] =
    {
        3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 13, 12 // Also used for Geomag
};
const int DMP_Quat6_Byte_Ordering[icm_20948_DMP_Quat6_Bytes] =
    {
        3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8 // Also used for Gyro_Calibr, Compass_Calibr
};
const int DMP_PQuat6_Byte_Ordering[icm_20948_DMP_PQuat6_Bytes] =
    {
        1, 0, 3, 2, 5, 4 // Also used for Raw_Accel, Compass
};
const int DMP_Raw_Gyro_Byte_Ordering[icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes] =
    {
        1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10};
const int DMP_Activity_Recognition_Byte_Ordering[icm_20948_DMP_Activity_Recognition_Bytes] =
    {
        0, 1, 5, 4, 3, 2};
const int DMP_Secondary_On_Off_Byte_Ordering[icm_20948_DMP_Secondary_On_Off_Bytes] =
    {
        1, 0};

const uint16_t inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX] =
    {
        // Data output control 1 register bit definition
        // 16-bit accel                                0x8000
        // 16-bit gyro                                 0x4000
        // 16-bit compass                              0x2000
        // 16-bit ALS                                  0x1000
        // 32-bit 6-axis quaternion                    0x0800
        // 32-bit 9-axis quaternion + heading accuracy 0x0400
        // 16-bit pedometer quaternion                 0x0200
        // 32-bit Geomag rv + heading accuracy         0x0100
        // 16-bit Pressure                             0x0080
        // 32-bit calibrated gyro                      0x0040
        // 32-bit calibrated compass                   0x0020
        // Pedometer Step Detector                     0x0010
        // Header 2                                    0x0008
        // Pedometer Step Indicator Bit 2              0x0004
        // Pedometer Step Indicator Bit 1              0x0002
        // Pedometer Step Indicator Bit 0              0x0001
        // Unsupported Sensors are 0xFFFF

        0xFFFF, // 0  Meta Data
        0x8008, // 1  Accelerometer
        0x0028, // 2  Magnetic Field
        0x0408, // 3  Orientation
        0x4048, // 4  Gyroscope
        0x1008, // 5  Light
        0x0088, // 6  Pressure
        0xFFFF, // 7  Temperature
        0xFFFF, // 8  Proximity <----------- fixme
        0x0808, // 9  Gravity
        0x8808, // 10 Linear Acceleration
        0x0408, // 11 Rotation Vector
        0xFFFF, // 12 Humidity
        0xFFFF, // 13 Ambient Temperature
        0x2008, // 14 Magnetic Field Uncalibrated
        0x0808, // 15 Game Rotation Vector
        0x4008, // 16 Gyroscope Uncalibrated
        0x0000, // 17 Significant Motion
        0x0018, // 18 Step Detector
        0x0010, // 19 Step Counter <----------- fixme
        0x0108, // 20 Geomagnetic Rotation Vector
        0xFFFF, // 21 ANDROID_SENSOR_HEART_RATE,
        0xFFFF, // 22 ANDROID_SENSOR_PROXIMITY,

        0x8008, // 23 ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
        0x0028, // 24 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
        0x0408, // 25 ANDROID_SENSOR_WAKEUP_ORIENTATION,
        0x4048, // 26 ANDROID_SENSOR_WAKEUP_GYROSCOPE,
        0x1008, // 27 ANDROID_SENSOR_WAKEUP_LIGHT,
        0x0088, // 28 ANDROID_SENSOR_WAKEUP_PRESSURE,
        0x0808, // 29 ANDROID_SENSOR_WAKEUP_GRAVITY,
        0x8808, // 30 ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
        0x0408, // 31 ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
        0xFFFF, // 32 ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
        0xFFFF, // 33 ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
        0x2008, // 34 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
        0x0808, // 35 ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
        0x4008, // 36 ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
        0x0018, // 37 ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
        0x0010, // 38 ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
        0x0108, // 39 ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
        0xFFFF, // 40 ANDROID_SENSOR_WAKEUP_HEART_RATE,
        0x0000, // 41 ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
        0x8008, // 42 Raw Acc
        0x4048, // 43 Raw Gyr
};

static const char *const TAG = "icm20948.sensor";
unsigned long startupTime = esp_timer_get_time() / 1000;
const unsigned long calibration_timeout = 5*60*1000; // 5 minutes timeout for calibration

void ICM20948Component::setup() {
  ESP_LOGD(TAG, "Setup");
  uint32_t hash = fnv1_hash(BIAS_STORE_PREF_KEY);
  bias_store_pref_ = global_preferences->make_preference<biasStore>(hash);
  if (bias_store_pref_.load(&bias_store_)) {
    ESP_LOGD(TAG, "Loaded preferences: %s", BIAS_STORE_PREF_KEY);
  }

  bool initialized = false;
  while (!initialized) {
    device_._dmp_firmware_available = true; // Assume DMP firmware is available

    device_._firmware_loaded = false; // Initialize _firmware_loaded
    device_._last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of setBank_.
    device_._last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.
    device_._gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    device_._gyroSFpll = 0;
    device_._enabled_Android_0 = 0;      // Keep track of which Android sensors are enabled: 0-31
    device_._enabled_Android_1 = 0;      // Keep track of which Android sensors are enabled: 32-
    device_._enabled_Android_intr_0 = 0; // Keep track of which Android sensor interrupts are enabled: 0-31
    device_._enabled_Android_intr_1 = 0; // Keep track of which Android sensor interrupts are enabled: 32-

    status_ = startupDefault_(true);
    if (status_ != ICM_20948_Stat_Ok)
    {
      ESP_LOGE(TAG, "begin: startupDefault returned: %d", status_);
    }

    ESP_LOGD(TAG, "begin: startupDefault returned: %d", status_);

    if (status_ != ICM_20948_Stat_Ok)
    {
      // steps_.push_back("Trying again...");
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    else
    {
      initialized = true;
    }
  }

  ESP_LOGE(TAG, "Device connected successfully!");
  // steps_.push_back("Device connected successfully!");

  bool success = true; 
  success &= (initializeDMP_() == ICM_20948_Stat_Ok);
  return;
  success &= (enableDMPSensor_(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  // Enable additional sensors / features
  success &= (enableDMPSensor_(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (enableDMPSensor_(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (enableDMPSensor_(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // TODO: make rate configurable
  success &= (setDMPODRrate_(DMP_ODR_Reg_Quat9, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
  success &= (setDMPODRrate_(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (setDMPODRrate_(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
  success &= (setDMPODRrate_(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
  success &= (setDMPODRrate_(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (setDMPODRrate_(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

  // Enable the FIFO
  success &= (enableFIFO_() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (enableDMP_() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (resetDMP_() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (resetFIFO_() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    ESP_LOGD(TAG, "DMP enabled!");
  }
  else
  {
    ESP_LOGE(TAG, "Enable DMP failed!");
    this->mark_failed();
    return;
  }

  if (isBiasStoreValid_(&bias_store_))
  {
    // ESP_LOGI("ICM20948", "Running bias calibration from boot flag...");
    // this->run_bias_calibration();
    // calibrate_on_boot_.save(false);  // Clear it
    ESP_LOGD(TAG, "Bias data in EEPROM is valid. Restoring it...");
    success &= (setBiasGyroX_(bias_store_.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (setBiasGyroY_(bias_store_.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (setBiasGyroZ_(bias_store_.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (setBiasAccelX_(bias_store_.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (setBiasAccelY_(bias_store_.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (setBiasAccelZ_(bias_store_.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (setBiasCPassX_(bias_store_.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (setBiasCPassY_(bias_store_.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (setBiasCPassZ_(bias_store_.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      ESP_LOGD(TAG, "Biases restored.");
      printBiases_(&bias_store_);
      return;
    }
    else
      ESP_LOGE(TAG, "Bias restore failed!");
  }

  // If we reach here, we either have no valid bias data or restoring it failed
  if (bias_store_.calibrate_on_boot) {
    ESP_LOGD(TAG, "Bias calibration on boot is enabled. Running calibration...");
    ESP_LOGD(TAG, "The biases will be saved in two minutes.");
    ESP_LOGD(TAG, "Before then:");
    ESP_LOGD(TAG, "* Rotate the sensor around all three axes");
    ESP_LOGD(TAG, "* Hold the sensor stationary in all six orientations for a few seconds");

    startupTime = esp_timer_get_time() / 1000;
  } else {
    ESP_LOGD(TAG, "Bias calibration on boot is disabled. Will not run calibration."); 
  }
}

void ICM20948Component::dump_config() {
  ESP_LOGCONFIG(TAG, "DUMP CONFIG");
  ESP_LOGCONFIG(TAG, "ICM20948Component");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Current Status: %d", status_);

  // for (const auto &step : steps_) {
  //   ESP_LOGCONFIG(TAG, "  Step: %s", step.c_str());
  // }

  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Mag X", this->mag_x_sensor_);
  LOG_SENSOR("  ", "Mag Y", this->mag_y_sensor_);
  LOG_SENSOR("  ", "Mag Z", this->mag_z_sensor_);
  LOG_SENSOR("  ", "Quaternion W", this->qw_sensor_);
  LOG_SENSOR("  ", "Quaternion X", this->qx_sensor_);
  LOG_SENSOR("  ", "Quaternion Y", this->qy_sensor_);
  LOG_SENSOR("  ", "Quaternion Z", this->qz_sensor_);
  LOG_SENSOR("  ", "Roll", this->roll_sensor_);
  LOG_SENSOR("  ", "Pitch", this->pitch_sensor_);
  LOG_SENSOR("  ", "Yaw", this->yaw_sensor_);
}

void ICM20948Component::loop() {
  // while (esp_timer_get_time() / 1000 - startupTime < 5000) { //TODO
  //   return; // Wait for 5 seconds before processing data
  // }

  icm_20948_DMP_data_t data;
  readDMPdataFromFIFO_(&data);

  if ((status_ == ICM_20948_Stat_Ok) || (status_ == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) // Check for orientation data (Quat9)
    {
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(std::max(0.0, 1.0 - (q1 * q1 + q2 * q2 + q3 * q3))); // Calculate q_w from q_x, q_y, q_z

      if (this->qw_sensor_ != nullptr) {
        this->qw_sensor_->publish_state(q0);
      }
      if (this->qx_sensor_ != nullptr) {
        this->qx_sensor_->publish_state(q2);
      }
      if (this->qy_sensor_ != nullptr) {
        this->qy_sensor_->publish_state(q1);
      }
      if (this->qz_sensor_ != nullptr) {
        this->qz_sensor_->publish_state(-q3);
      }

      // Calculate roll, pitch, yaw from quaternion
      if (this->roll_sensor_ != nullptr) {
        double roll = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        this->roll_sensor_->publish_state(roll * 180.0 / M_PI); // Convert to degrees
      }
      if (this->pitch_sensor_ != nullptr) {
        double pitch = asin(-2.0 * (q1 * q3 - q0  * q2));
        this->pitch_sensor_->publish_state(pitch * 180.0 / M_PI); // Convert to degrees
      }
      if (this->yaw_sensor_ != nullptr) {
        double yaw = atan2(2.0 * (q2 * q3 + q0 * q1), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); 
        this->yaw_sensor_->publish_state(yaw * 180.0 / M_PI); // Convert to degrees
      }
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
    {
      if (this->accel_x_sensor_ != nullptr) {
        this->accel_x_sensor_->publish_state((float)data.Raw_Accel.Data.X);
      }
      if (this->accel_y_sensor_ != nullptr) {
        this->accel_y_sensor_->publish_state((float)data.Raw_Accel.Data.Y);
      }
      if (this->accel_z_sensor_ != nullptr) {
        this->accel_z_sensor_->publish_state((float)data.Raw_Accel.Data.Z);
      }
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      if (this->gyro_x_sensor_ != nullptr) {
        this->gyro_x_sensor_->publish_state((float)data.Raw_Gyro.Data.X);
      }
      if (this->gyro_y_sensor_ != nullptr) {
        this->gyro_y_sensor_->publish_state((float)data.Raw_Gyro.Data.Y);
      }
      if (this->gyro_z_sensor_ != nullptr) {
        this->gyro_z_sensor_->publish_state((float)data.Raw_Gyro.Data.Z);
      }
    }

    if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
    {
      if (this->mag_x_sensor_ != nullptr) {
        this->mag_x_sensor_->publish_state((float)data.Compass.Data.X);
      }
      if (this->mag_y_sensor_ != nullptr) {
        this->mag_y_sensor_->publish_state((float)data.Compass.Data.Y);
      }
      if (this->mag_z_sensor_ != nullptr) {
        this->mag_z_sensor_->publish_state((float)data.Compass.Data.Z);
      }
    }
  }

  if (bias_store_.calibrate_on_boot && !bias_store_.bias_calibrated) {
    // Run bias calibration
    if (esp_timer_get_time() / 1000 - startupTime > calibration_timeout) { // After x minutes
      ESP_LOGD(TAG, "Calibration timeout reached. Saving biases...");
      
      biasStore store;
          
      bool success = (getBiasGyroX_(&store.biasGyroX) == ICM_20948_Stat_Ok);
      success &= (getBiasGyroY_(&store.biasGyroY) == ICM_20948_Stat_Ok);
      success &= (getBiasGyroZ_(&store.biasGyroZ) == ICM_20948_Stat_Ok);
      success &= (getBiasAccelX_(&store.biasAccelX) == ICM_20948_Stat_Ok);
      success &= (getBiasAccelY_(&store.biasAccelY) == ICM_20948_Stat_Ok);
      success &= (getBiasAccelZ_(&store.biasAccelZ) == ICM_20948_Stat_Ok);
      success &= (getBiasCPassX_(&store.biasCPassX) == ICM_20948_Stat_Ok);
      success &= (getBiasCPassY_(&store.biasCPassY) == ICM_20948_Stat_Ok);
      success &= (getBiasCPassZ_(&store.biasCPassZ) == ICM_20948_Stat_Ok);
      updateBiasStoreSum_(&store);

      if (!success) {
        ESP_LOGE(TAG, "Failed to read biases from device. Calibration failed.");
        this->status_set_warning("Bias calibration failed");
        bias_store_.calibrate_on_boot = false;
        return;
      }

      bias_store_.biasGyroX = store.biasGyroX;
      bias_store_.biasGyroY = store.biasGyroY;
      bias_store_.biasGyroZ = store.biasGyroZ;
      bias_store_.biasAccelX = store.biasAccelX;
      bias_store_.biasAccelY = store.biasAccelY;
      bias_store_.biasAccelZ = store.biasAccelZ;
      bias_store_.biasCPassX = store.biasCPassX;
      bias_store_.biasCPassY = store.biasCPassY;
      bias_store_.biasCPassZ = store.biasCPassZ;
      bias_store_.sum = store.sum;
      bias_store_.bias_calibrated = true; // Set the flag to true

      bias_store_.calibrate_on_boot = false; // Clear the flag
      bias_store_pref_.save(&bias_store_); // Save the updated preferences
      ESP_LOGD(TAG, "Bias calibration completed and saved.");
    }
  }

  this->status_clear_warning();
}

float ICM20948Component::get_setup_priority() const { return setup_priority::LATE; }

//Gyro Bias
ICM_20948_Status_e ICM20948Component::setBiasGyroX_(int32_t newValue)
{
  // steps_.push_back("setBiasGyroX_");
  unsigned char gyro_bias_reg[4];
  gyro_bias_reg[0] = (unsigned char)(newValue >> 24);
  gyro_bias_reg[1] = (unsigned char)(newValue >> 16);
  gyro_bias_reg[2] = (unsigned char)(newValue >> 8);
  gyro_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(GYRO_BIAS_X, 4, (const unsigned char*)&gyro_bias_reg);
}

ICM_20948_Status_e ICM20948Component::setBiasGyroY_(int32_t newValue)
{
  // steps_.push_back("setBiasGyroY_");
  unsigned char gyro_bias_reg[4];
  gyro_bias_reg[0] = (unsigned char)(newValue >> 24);
  gyro_bias_reg[1] = (unsigned char)(newValue >> 16);
  gyro_bias_reg[2] = (unsigned char)(newValue >> 8);
  gyro_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(GYRO_BIAS_Y, 4, (const unsigned char*)&gyro_bias_reg);
}

ICM_20948_Status_e ICM20948Component::setBiasGyroZ_(int32_t newValue)
{
  // steps_.push_back("setBiasGyroZ_");
  unsigned char gyro_bias_reg[4];
  gyro_bias_reg[0] = (unsigned char)(newValue >> 24);
  gyro_bias_reg[1] = (unsigned char)(newValue >> 16);
  gyro_bias_reg[2] = (unsigned char)(newValue >> 8);
  gyro_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(GYRO_BIAS_Z, 4, (const unsigned char*)&gyro_bias_reg);
}

ICM_20948_Status_e ICM20948Component::getBiasGyroX_( int32_t* bias)
{
  // steps_.push_back("getBiasGyroX_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_write_mems_(GYRO_BIAS_X, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

ICM_20948_Status_e ICM20948Component::getBiasGyroY_( int32_t* bias)
{
  // steps_.push_back("getBiasGyroY_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_write_mems_(GYRO_BIAS_Y, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

ICM_20948_Status_e ICM20948Component::getBiasGyroZ_( int32_t* bias)
{
  // steps_.push_back("getBiasGyroZ_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_write_mems_(GYRO_BIAS_Z, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

//Accel Bias
ICM_20948_Status_e ICM20948Component::setBiasAccelX_(int32_t newValue)
{
  // steps_.push_back("setBiasAccelX_");
  unsigned char accel_bias_reg[4];
  accel_bias_reg[0] = (unsigned char)(newValue >> 24);
  accel_bias_reg[1] = (unsigned char)(newValue >> 16);
  accel_bias_reg[2] = (unsigned char)(newValue >> 8);
  accel_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(ACCEL_BIAS_X, 4, (const unsigned char*)&accel_bias_reg);
}

ICM_20948_Status_e ICM20948Component::setBiasAccelY_(int32_t newValue)
{
  // steps_.push_back("setBiasAccelY_");
  unsigned char accel_bias_reg[4];
  accel_bias_reg[0] = (unsigned char)(newValue >> 24);
  accel_bias_reg[1] = (unsigned char)(newValue >> 16);
  accel_bias_reg[2] = (unsigned char)(newValue >> 8);
  accel_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(ACCEL_BIAS_Y, 4, (const unsigned char*)&accel_bias_reg);
}

ICM_20948_Status_e ICM20948Component::setBiasAccelZ_(int32_t newValue)
{
  // steps_.push_back("setBiasAccelZ_");
  unsigned char accel_bias_reg[4];
  accel_bias_reg[0] = (unsigned char)(newValue >> 24);
  accel_bias_reg[1] = (unsigned char)(newValue >> 16);
  accel_bias_reg[2] = (unsigned char)(newValue >> 8);
  accel_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(ACCEL_BIAS_Z, 4, (const unsigned char*)&accel_bias_reg);
}


ICM_20948_Status_e ICM20948Component::getBiasAccelX_( int32_t* bias)
{
  // steps_.push_back("getBiasAccelX_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_read_mems_(ACCEL_BIAS_X, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

ICM_20948_Status_e ICM20948Component::getBiasAccelY_( int32_t* bias)
{
  // steps_.push_back("getBiasAccelY_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_read_mems_(ACCEL_BIAS_Y, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

ICM_20948_Status_e ICM20948Component::getBiasAccelZ_( int32_t* bias)
{
  // steps_.push_back("getBiasAccelZ_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_read_mems_(ACCEL_BIAS_Z, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

//CPass Bias
ICM_20948_Status_e ICM20948Component::setBiasCPassX_( int32_t newValue)
{
  // steps_.push_back("setBiasCPassX_");
  unsigned char cpass_bias_reg[4];
  cpass_bias_reg[0] = (unsigned char)(newValue >> 24);
  cpass_bias_reg[1] = (unsigned char)(newValue >> 16);
  cpass_bias_reg[2] = (unsigned char)(newValue >> 8);
  cpass_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(CPASS_BIAS_X, 4, (const unsigned char*)&cpass_bias_reg);
}

ICM_20948_Status_e ICM20948Component::setBiasCPassY_( int32_t newValue)
{
  // steps_.push_back("setBiasCPassY_");
  unsigned char cpass_bias_reg[4];
  cpass_bias_reg[0] = (unsigned char)(newValue >> 24);
  cpass_bias_reg[1] = (unsigned char)(newValue >> 16);
  cpass_bias_reg[2] = (unsigned char)(newValue >> 8);
  cpass_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(CPASS_BIAS_Y, 4, (const unsigned char*)&cpass_bias_reg);
}

ICM_20948_Status_e ICM20948Component::setBiasCPassZ_( int32_t newValue)
{
  // steps_.push_back("setBiasCPassZ_");
  unsigned char cpass_bias_reg[4];
  cpass_bias_reg[0] = (unsigned char)(newValue >> 24);
  cpass_bias_reg[1] = (unsigned char)(newValue >> 16);
  cpass_bias_reg[2] = (unsigned char)(newValue >> 8);
  cpass_bias_reg[3] = (unsigned char)(newValue & 0xff);
  return inv_icm20948_write_mems_(CPASS_BIAS_Z, 4, (const unsigned char*)&cpass_bias_reg);
}

ICM_20948_Status_e ICM20948Component::getBiasCPassX_( int32_t* bias)
{
  // steps_.push_back("getBiasCPassX_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_read_mems_(CPASS_BIAS_X, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

ICM_20948_Status_e ICM20948Component::getBiasCPassY_( int32_t* bias)
{
  // steps_.push_back("getBiasCPassY_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_read_mems_(CPASS_BIAS_Y, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

ICM_20948_Status_e ICM20948Component::getBiasCPassZ_( int32_t* bias)
{
  // steps_.push_back("getBiasCPassZ_");
  unsigned char bias_data[4] = { 0 };
  status_ = inv_icm20948_read_mems_(CPASS_BIAS_Z, 4, bias_data);
  union {
    int32_t signed32;
    uint32_t unsigned32;
  } signedUnsigned32;
  signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
  *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
  return status_;
}

std::string ICM20948Component::statusToString_(ICM_20948_Status_e stat)
{
  // steps_.push_back("statusToString_");
  switch (stat)
  {
  case ICM_20948_Stat_Ok:
    return "All is well.";
  case ICM_20948_Stat_Err:
    return "General Error";
  case ICM_20948_Stat_NotImpl:
    return "Not Implemented";
  case ICM_20948_Stat_ParamErr:
    return "Parameter Error";
  case ICM_20948_Stat_WrongID:
    return "Wrong ID";
  case ICM_20948_Stat_InvalSensor:
    return "Invalid Sensor";
  case ICM_20948_Stat_NoData:
    return "Data Underflow";
  case ICM_20948_Stat_SensorNotSupported:
    return "Sensor Not Supported";
  case ICM_20948_Stat_DMPNotSupported:
    return "DMP Firmware Not Supported. Is #define ICM_20948_USE_DMP commented in util/ICM_20948_C.h?";
  case ICM_20948_Stat_DMPVerifyFail:
    return "DMP Firmware Verification Failed";
  case ICM_20948_Stat_FIFONoDataAvail:
    return "No FIFO Data Available";
  case ICM_20948_Stat_FIFOIncompleteData:
    return "DMP data in FIFO was incomplete";
  case ICM_20948_Stat_FIFOMoreDataAvail:
    return "More FIFO Data Available";
  case ICM_20948_Stat_UnrecognisedDMPHeader:
    return "Unrecognised DMP Header";
  case ICM_20948_Stat_UnrecognisedDMPHeader2:
    return "Unrecognised DMP Header2";
  case ICM_20948_Stat_InvalDMPRegister:
    return "Invalid DMP Register";
  default:
    return "Unknown Status";
  }
}

// Device Level
ICM_20948_Status_e ICM20948Component::setBank_(uint8_t bank)
{
  // steps_.push_back("setBank_: " + std::to_string(bank));
  if (bank > 3)
  {
    return ICM_20948_Stat_ParamErr;
  } // Only 4 possible banks

  if (bank == device_._last_bank) // Do we need to change bank?
    return ICM_20948_Stat_Ok;   // Bail if we don't need to change bank to avoid unnecessary bus traffic

  device_._last_bank = bank;   // Store the requested bank (before we bit-shift)
  bank = (bank << 4) & 0x30; // bits 5:4 of REG_BANK_SEL
  return write_i2c(REG_BANK_SEL, &bank, 1);
}

ICM_20948_Status_e ICM20948Component::swReset_(void)
{
  // steps_.push_back("swReset_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  ICM_20948_PWR_MGMT_1_t reg;

  setBank_(0); // Must be in the right bank

  retval = read_i2c( AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  reg.DEVICE_RESET = 1;

  retval = write_i2c( AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::sleep_(bool on)
{
  // steps_.push_back("sleep_");

  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  ICM_20948_PWR_MGMT_1_t reg;

  setBank_(0); // Must be in the right bank

  retval = read_i2c(AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  if (on)
  {
    reg.SLEEP = 1;
  }
  else
  {
    reg.SLEEP = 0;
  }

  retval = write_i2c(AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::lowPower_(bool on)
{
  // steps_.push_back("lowPower_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  ICM_20948_PWR_MGMT_1_t reg;

  setBank_(0); // Must be in the right bank

  retval = read_i2c(AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  if (on)
  {
    reg.LP_EN = 1;
  }
  else
  {
    reg.LP_EN = 0;
  }

  retval = write_i2c(AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::setClockSource_(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
  // steps_.push_back("setClockSource_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  ICM_20948_PWR_MGMT_1_t reg;

  setBank_(0); // Must be in the right bank

  retval = write_i2c(AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  reg.CLKSEL = source;

  retval = write_i2c( AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::getWhoAmI_(uint8_t *whoami)
{
  // steps_.push_back("getWhoAmI_");
  if (whoami == NULL)
  {
    return ICM_20948_Stat_ParamErr;
  }
  setBank_(0); // Must be in the right bank
  return read_i2c(AGB0_REG_WHO_AM_I, whoami, 1);
}

ICM_20948_Status_e ICM20948Component::checkID_(void)
{
  // steps_.push_back("checkID_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  uint8_t whoami = 0x00;
  retval = getWhoAmI_(&whoami);
  // steps_.push_back(std::format("{:02X}", whoami));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (whoami != ICM_20948_WHOAMI)
  {
    return ICM_20948_Stat_WrongID;
  }
  return retval;
}

// Internal Sensor Options
ICM_20948_Status_e ICM20948Component::setSampleMode_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode)
{
  // steps_.push_back("setSampleMode_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  ICM_20948_LP_CONFIG_t reg;

  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mst)))
  {
    return ICM_20948_Stat_SensorNotSupported;
  }

  retval = setBank_(0); // Must be in the right bank
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  retval = read_i2c(AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    reg.ACCEL_CYCLE = mode;
  } // Set all desired sensors to this setting
  if (sensors & ICM_20948_Internal_Gyr)
  {
    reg.GYRO_CYCLE = mode;
  }
  if (sensors & ICM_20948_Internal_Mst)
  {
    reg.I2C_MST_CYCLE = mode;
  }

  retval = write_i2c(AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // Check the data was written correctly
  retval = read_i2c(AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (sensors & ICM_20948_Internal_Acc)
  {
    if (reg.ACCEL_CYCLE != mode) retval = ICM_20948_Stat_Err;
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    if (reg.GYRO_CYCLE != mode) retval = ICM_20948_Stat_Err;
  }
  if (sensors & ICM_20948_Internal_Mst)
  {
    if (reg.I2C_MST_CYCLE != mode) retval = ICM_20948_Stat_Err;
  }

  vTaskDelay(pdMS_TO_TICKS(1));
  return retval;
}

ICM_20948_Status_e ICM20948Component::setFullScale_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss)
{
  // steps_.push_back("setFullScale_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return ICM_20948_Stat_SensorNotSupported;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    ICM_20948_ACCEL_CONFIG_t reg;
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    reg.ACCEL_FS_SEL = fss.a;
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    // Check the data was written correctly
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    if (reg.ACCEL_FS_SEL != fss.a) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    ICM_20948_GYRO_CONFIG_1_t reg;
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    reg.GYRO_FS_SEL = fss.g;
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    // Check the data was written correctly
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    if (reg.GYRO_FS_SEL != fss.g) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::setDLPFcfg_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_dlpcfg_t cfg)
{
  // steps_.push_back("setDLPFcfg_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return ICM_20948_Stat_SensorNotSupported;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    ICM_20948_ACCEL_CONFIG_t reg;
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    reg.ACCEL_DLPFCFG = cfg.a;
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    // Check the data was written correctly
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    if (reg.ACCEL_DLPFCFG != cfg.a) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    ICM_20948_GYRO_CONFIG_1_t reg;
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    reg.GYRO_DLPFCFG = cfg.g;
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    // Check the data was written correctly
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    if (reg.GYRO_DLPFCFG != cfg.g) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::enableDLPF_(ICM_20948_InternalSensorID_bm sensors, bool enable)
{
  // steps_.push_back("EnableDLPF_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return ICM_20948_Stat_SensorNotSupported;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    ICM_20948_ACCEL_CONFIG_t reg;
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    if (enable)
    {
      reg.ACCEL_FCHOICE = 1;
    }
    else
    {
      reg.ACCEL_FCHOICE = 0;
    }
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    // Check the data was written correctly
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t)));
    if (enable)
    {
      if (reg.ACCEL_FCHOICE != 1) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
    }
    else
    {
      if (reg.ACCEL_FCHOICE != 0) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
    }
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    ICM_20948_GYRO_CONFIG_1_t reg;
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    if (enable)
    {
      reg.GYRO_FCHOICE = 1;
    }
    else
    {
      reg.GYRO_FCHOICE = 0;
    }
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    // Check the data was written correctly
    retval = (ICM_20948_Status_e)(retval | read_i2c(AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t)));
    if (enable)
    {
      if (reg.GYRO_FCHOICE != 1) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
    }
    else
    {
      if (reg.GYRO_FCHOICE != 0) retval = (ICM_20948_Status_e)(retval | ICM_20948_Stat_Err);
    }
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::setSampleRate_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt)
{
  // steps_.push_back("setSampleRate_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return ICM_20948_Stat_SensorNotSupported;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    uint8_t div1 = (smplrt.a >> 8); // Thank you @yanivamichy #109
    uint8_t div2 = (smplrt.a & 0xFF);
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_ACCEL_SMPLRT_DIV_1, &div1, 1));
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_ACCEL_SMPLRT_DIV_2, &div2, 1));
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    retval = (ICM_20948_Status_e)(retval | setBank_(2)); // Must be in the right bank
    uint8_t div = (smplrt.g);
    retval = (ICM_20948_Status_e)(retval | write_i2c(AGB2_REG_GYRO_SMPLRT_DIV, &div, 1));
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::intEnableDMP_(bool enable)
{
  // steps_.push_back("initEnableDMP_");
  ICM_20948_INT_enable_t en;                          // storage
  status_ = intEnable_(NULL, &en); // read phase
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  en.DMP_INT1_EN = enable;                           // change the setting
  status_ = intEnable_(&en, &en); // write phase w/ readback
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  if (en.DMP_INT1_EN != enable)
  {
    status_ = ICM_20948_Stat_Err;
    return status_;
  }
  return status_;
}

ICM_20948_Status_e ICM20948Component::intEnableRawDataReady_(bool enable)
{
  // steps_.push_back("intEnableRawDataReady_");
  ICM_20948_INT_enable_t en;                          // storage
  status_ = intEnable_(NULL, &en); // read phase
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  en.RAW_DATA_0_RDY_EN = enable;                     // change the setting
  status_ = intEnable_(&en, &en); // write phase w/ readback
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  if (en.RAW_DATA_0_RDY_EN != enable)
  {
    status_ = ICM_20948_Stat_Err;
    return status_;
  }
  return status_;
}

ICM_20948_Status_e ICM20948Component::intEnableOverflowFIFO_(uint8_t bm_enable)
{
  // steps_.push_back("intEnableOverflowFIFO_");
  ICM_20948_INT_enable_t en;                          // storage
  status_ = intEnable_(NULL, &en); // read phase
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
  en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
  en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
  en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
  en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
  status_ = intEnable_(&en, &en); // write phase w/ readback
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  return status_;
}

// Interface Options
ICM_20948_Status_e ICM20948Component::i2cMasterPassthrough_(bool passthrough)
{
  // steps_.push_back("i2cMasterPassthrough_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_INT_PIN_CFG_t reg;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  retval = read_i2c(AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_INT_PIN_CFG_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  reg.BYPASS_EN = passthrough;
  retval = write_i2c(AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_INT_PIN_CFG_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  return retval;
}

ICM_20948_Status_e ICM20948Component::i2cMasterEnable_(bool enable)
{
  // steps_.push_back("i2cMasterEnable_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  // Disable BYPASS_EN
  retval = i2cMasterPassthrough_(false);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  ICM_20948_I2C_MST_CTRL_t ctrl;
  retval = setBank_(3);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  retval = read_i2c(AGB3_REG_I2C_MST_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_MST_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  ctrl.I2C_MST_CLK = 0x07; // corresponds to 345.6 kHz, good for up to 400 kHz
  ctrl.I2C_MST_P_NSR = 1;
  retval = write_i2c(AGB3_REG_I2C_MST_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_MST_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  ICM_20948_USER_CTRL_t reg;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  retval = read_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&reg, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (enable)
  {
    reg.I2C_MST_EN = 1;
  }
  else
  {
    reg.I2C_MST_EN = 0;
  }
  retval = write_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&reg, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  return retval;
}

ICM_20948_Status_e ICM20948Component::i2cMasterReset_()
{
  // steps_.push_back("i2cMasterReset_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_USER_CTRL_t ctrl;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  ctrl.I2C_MST_RST = 1; //Reset!

  retval = write_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::i2cControllerConfigurePeripheral_(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  // steps_.push_back("i2cControllerConfigurePeripheral_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t periph_addr_reg;
  uint8_t periph_reg_reg;
  uint8_t periph_ctrl_reg;
  uint8_t periph_do_reg;

  switch (peripheral)
  {
  case 0:
    periph_addr_reg = AGB3_REG_I2C_PERIPH0_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH0_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH0_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH0_DO;
    break;
  case 1:
    periph_addr_reg = AGB3_REG_I2C_PERIPH1_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH1_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH1_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH1_DO;
    break;
  case 2:
    periph_addr_reg = AGB3_REG_I2C_PERIPH2_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH2_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH2_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH2_DO;
    break;
  case 3:
    periph_addr_reg = AGB3_REG_I2C_PERIPH3_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH3_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH3_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH3_DO;
    break;
  default:
    return ICM_20948_Stat_ParamErr;
  }

  retval = setBank_(3);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // Set the peripheral address and the Rw flag
  ICM_20948_I2C_PERIPHX_ADDR_t address;
  address.ID = addr;
  if (Rw)
  {
    address.RNW = 1;
  }
  else
  {
    address.RNW = 0; // Make sure bit is clear (just in case there is any garbage in that RAM location)
  }
  retval = write_i2c(periph_addr_reg, (uint8_t *)&address, sizeof(ICM_20948_I2C_PERIPHX_ADDR_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // If we are setting up a write, configure the Data Out register too
  if (!Rw)
  {
    ICM_20948_I2C_PERIPHX_DO_t dataOutByte;
    dataOutByte.DO = dataOut;
    retval = write_i2c(periph_do_reg, (uint8_t *)&dataOutByte, sizeof(ICM_20948_I2C_PERIPHX_DO_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
  }

  // Set the peripheral sub-address (register address)
  ICM_20948_I2C_PERIPHX_REG_t subaddress;
  subaddress.REG = reg;
  retval = write_i2c(periph_reg_reg, (uint8_t *)&subaddress, sizeof(ICM_20948_I2C_PERIPHX_REG_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // Set up the control info
  ICM_20948_I2C_PERIPHX_CTRL_t ctrl;
  ctrl.LENG = len;
  ctrl.EN = enable;
  ctrl.REG_DIS = data_only;
  ctrl.GRP = grp;
  ctrl.BYTE_SW = swap;
  retval = write_i2c(periph_ctrl_reg, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPHX_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  return retval;
}

ICM_20948_Status_e ICM20948Component::i2cControllerPeriph4Transaction_(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  // steps_.push_back("i2cControllerPeriph4Transaction_");
  // Thanks MikeFair! // https://github.com/kriswiner/MPU9250/issues/86
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  addr = (((Rw) ? 0x80 : 0x00) | addr);

  retval = setBank_(3);
  retval = write_i2c(AGB3_REG_I2C_PERIPH4_ADDR, (uint8_t *)&addr, 1);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = setBank_(3);
  retval = write_i2c(AGB3_REG_I2C_PERIPH4_REG, (uint8_t *)&reg, 1);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  ICM_20948_I2C_PERIPH4_CTRL_t ctrl;
  ctrl.EN = 1;
  ctrl.INT_EN = false;
  ctrl.DLY = 0;
  ctrl.REG_DIS = !send_reg_addr;

  ICM_20948_I2C_MST_STATUS_t i2c_mst_status;
  bool txn_failed = false;
  uint16_t nByte = 0;

  while (nByte < len)
  {
    if (!Rw)
    {
      retval = setBank_(3);
      retval = write_i2c(AGB3_REG_I2C_PERIPH4_DO, (uint8_t *)&(data[nByte]), 1);
      if (retval != ICM_20948_Stat_Ok)
      {
        return retval;
      }
    }

    // Kick off txn
    retval = setBank_(3);
    retval = write_i2c(AGB3_REG_I2C_PERIPH4_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPH4_CTRL_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }

    // long tsTimeout = millis() + 3000;  // Emergency timeout for txn (hard coded to 3 secs)
    uint32_t max_cycles = 1000;
    uint32_t count = 0;
    bool peripheral4Done = false;
    while (!peripheral4Done)
    {
      retval = setBank_(0);
      retval = read_i2c(AGB0_REG_I2C_MST_STATUS, (uint8_t *)&i2c_mst_status, 1);

      peripheral4Done = (i2c_mst_status.I2C_PERIPH4_DONE /*| (millis() > tsTimeout) */); //Avoid forever-loops
      peripheral4Done |= (count >= max_cycles);
      count++;
    }
    txn_failed = (i2c_mst_status.I2C_PERIPH4_NACK /*| (millis() > tsTimeout) */);
    txn_failed |= (count >= max_cycles);
    if (txn_failed)
      break;

    if (Rw)
    {
      retval = setBank_(3);
      retval = read_i2c(AGB3_REG_I2C_PERIPH4_DI, &data[nByte], 1);
    }

    nByte++;
  }

  if (txn_failed)
  {
    //We often fail here if mag is stuck
    return ICM_20948_Stat_Err;
  }

  return retval;
}

ICM_20948_Status_e ICM20948Component::i2cMasterSingleW_(uint8_t addr, uint8_t reg, uint8_t data)
{
  // steps_.push_back("i2cMasterSingleW_");
  return i2cControllerPeriph4Transaction_(addr, reg, &data, 1, false, true);
  
}
uint8_t ICM20948Component::i2cMasterSingleR_(uint8_t addr, uint8_t reg)
{
  // steps_.push_back("i2cMasterSingleR_");
  uint8_t data;
  i2cControllerPeriph4Transaction_(addr, reg, &data, 1, true, true);

  return data;
}

ICM_20948_Status_e ICM20948Component::startupDefault_(bool minimal)
{
  // steps_.push_back("startupDefault_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  retval = checkID_();
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: checkID returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = swReset_();
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: swReset returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  retval = sleep_(false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: sleep returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = lowPower_(false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: lowPower returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = startupMagnetometer_(minimal); // Pass the minimal startup flag to startupMagnetometer
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: startupMagnetometer returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  if (minimal) // Return now if minimal is true
  {
    // steps_.push_back("startupDefault: minimal startup complete");
    ESP_LOGE(TAG, "startupDefault: minimal startup complete!");
    return status_;
  }

  return status_;

  retval = setSampleMode_((ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: setSampleMode returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  } // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

  ICM_20948_fss_t FSS;
  FSS.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  retval = setFullScale_((ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: setFullScale returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d473bw_n499bw;
  dlpcfg.g = gyr_d361bw4_n376bw5;
  retval = setDLPFcfg_((ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: setDLPFcfg returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = enableDLPF_(ICM_20948_Internal_Acc, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: enableDLPF (Acc) returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = enableDLPF_(ICM_20948_Internal_Gyr, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupDefault: enableDLPF (Gyr) returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  return status_;
}

uint8_t ICM20948Component::readMag_(AK09916_Reg_Addr_e reg)
{
  // steps_.push_back("readMag_");
  return i2cMasterSingleR_(MAG_AK09916_I2C_ADDR, reg); // i2cMasterSingleR updates status too
}

ICM_20948_Status_e ICM20948Component::writeMag_(AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
  // steps_.push_back("writeMag_");
  return i2cMasterSingleW_(MAG_AK09916_I2C_ADDR, reg, *pdata);
}

ICM_20948_Status_e ICM20948Component::resetMag_()
{
  // steps_.push_back("resetMag_");
  uint8_t SRST = 1;
  return i2cMasterSingleW_(MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, SRST);
}

// FIFO
ICM_20948_Status_e ICM20948Component::enableFIFO_(bool enable)
{
  // steps_.push_back("enableFIFO_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_USER_CTRL_t ctrl;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  if (enable)
    ctrl.FIFO_EN = 1;
  else
    ctrl.FIFO_EN = 0;

  retval = write_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::resetFIFO_(void)
{
  // steps_.push_back("resetFIFO_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_FIFO_RST_t ctrl;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  ctrl.FIFO_RESET = 0x1F; // Datasheet says "FIFO_RESET[4:0]"

  retval = write_i2c(AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  //delay ???
  ctrl.FIFO_RESET = 0x1E; // The InvenSense Nucleo examples write 0x1F followed by 0x1E

  retval = write_i2c(AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  return retval;
}

ICM_20948_Status_e ICM20948Component::setFIFOmode_(bool snapshot)
{
  // steps_.push_back("setFIFOmode_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_FIFO_MODE_t ctrl;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_MODE_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  if (snapshot)
    ctrl.FIFO_MODE = 0x1F; // Datasheet says "FIFO_MODE[4:0]"
  else
    ctrl.FIFO_MODE = 0;

  retval = write_i2c(AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_MODE_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::getFIFOcount_(uint16_t *count)
{
  // steps_.push_back("getFIFOcount_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_FIFO_COUNTH_t ctrlh;
  ICM_20948_FIFO_COUNTL_t ctrll;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_FIFO_COUNT_H, (uint8_t *)&ctrlh, sizeof(ICM_20948_FIFO_COUNTH_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  ctrlh.FIFO_COUNTH &= 0x1F; // Datasheet says "FIFO_CNT[12:8]"

  retval = read_i2c(AGB0_REG_FIFO_COUNT_L, (uint8_t *)&ctrll, sizeof(ICM_20948_FIFO_COUNTL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  *count = (((uint16_t)ctrlh.FIFO_COUNTH) << 8) | (uint16_t)ctrll.FIFO_COUNTL;

  return retval;
}

ICM_20948_Status_e ICM20948Component::readFIFO_(uint8_t *data, uint8_t len)
{
  // steps_.push_back("readFIFO_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_FIFO_R_W, data, len);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  return retval;
}

// DMP
ICM_20948_Status_e ICM20948Component::enableDMP_(bool enable)
{
  // steps_.push_back("enableDMP_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_USER_CTRL_t ctrl;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  if (enable)
    ctrl.DMP_EN = 1;
  else
    ctrl.DMP_EN = 0;

  retval = write_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::resetDMP_(void)
{
  // steps_.push_back("resetDMP_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_USER_CTRL_t ctrl;
  retval = setBank_(0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  retval = read_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  ctrl.DMP_RST = 1;

  retval = write_i2c(AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_20948_Status_e ICM20948Component::loadDMPFirmware_(void)
{
  // steps_.push_back("loadDMPFirmware_");
  unsigned short load_addr = DMP_LOAD_START; // Start address for DMP firmware
  const unsigned char *data_start = dmp3_image;
  unsigned short size_start = sizeof(dmp3_image);

  int write_size;
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;
  unsigned short memaddr;
  const unsigned char *data;
  unsigned short size;
  unsigned char data_cmp[INV_MAX_SERIAL_READ];
  int flag = 0;

  if (device_._dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported;

  if (device_._firmware_loaded)
    return ICM_20948_Stat_Ok; // Bail with no error if firmware is already loaded

  result = sleep_(false); // Make sure chip is awake
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  result = lowPower_(false); // Make sure chip is not in low power state
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  return result;

  // Write DMP memory

  data = data_start;
  size = size_start;
  memaddr = load_addr;

  while (size > 0)
  {
    //write_size = min(size, INV_MAX_SERIAL_WRITE); // Write in chunks of INV_MAX_SERIAL_WRITE
    if (size <= INV_MAX_SERIAL_WRITE) // Write in chunks of INV_MAX_SERIAL_WRITE
      write_size = size;
    else
      write_size = INV_MAX_SERIAL_WRITE;
    if ((memaddr & 0xff) + write_size > 0x100)
    {
      // Moved across a bank
      write_size = (memaddr & 0xff) + write_size - 0x100;
    }

    result = inv_icm20948_write_mems_(memaddr, write_size, (unsigned char *)data);

    if (result != ICM_20948_Stat_Ok)
      return result;
    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }

  // Verify DMP memory
  data = data_start;
  size = size_start;
  memaddr = load_addr;
  while (size > 0)
  {
    //write_size = min(size, INV_MAX_SERIAL_READ); // Read in chunks of INV_MAX_SERIAL_READ
    if (size <= INV_MAX_SERIAL_READ) // Read in chunks of INV_MAX_SERIAL_READ
      write_size = size;
    else
      write_size = INV_MAX_SERIAL_READ;
    if ((memaddr & 0xff) + write_size > 0x100)
    {
      // Moved across a bank
      write_size = (memaddr & 0xff) + write_size - 0x100;
    }
    result = inv_icm20948_read_mems_(memaddr, write_size, data_cmp);
    if (result != ICM_20948_Stat_Ok)
      flag++;                               // Error, DMP not written correctly
    if (memcmp(data_cmp, data, write_size)) // Compare the data
      return ICM_20948_Stat_DMPVerifyFail;
    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }

  //Enable LP_EN since we disabled it at begining of this function.
  result = lowPower_(true); // Put chip into low power state
  if (result != ICM_20948_Stat_Ok)
    return result;

  if (!flag)
  {
    //Serial.println("DMP Firmware was updated successfully..");
    device_._firmware_loaded = true;
  }

  return result;
}

ICM_20948_Status_e ICM20948Component::setDMPstartAddress_(unsigned short address)
{
  // steps_.push_back("setDMPstartAddress_");
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;

  if (device_._dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported;

  unsigned char start_address[2];

  start_address[0] = (unsigned char)(address >> 8);
  start_address[1] = (unsigned char)(address & 0xff);

  result = setBank_(2); // Set bank 2
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  // Write the sensor control bits into memory address AGB2_REG_PRGM_START_ADDRH
  result = write_i2c(AGB2_REG_PRGM_START_ADDRH, (uint8_t *)start_address, 2);

  return result;
}

ICM_20948_Status_e ICM20948Component::enableDMPSensor_(enum inv_icm20948_sensor sensor, bool state)
{
  // steps_.push_back("enableDMPSensor_");
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;

  uint16_t inv_event_control = 0; // Use this to store the value for MOTION_EVENT_CTL
  uint16_t data_rdy_status = 0;   // Use this to store the value for DATA_RDY_STATUS

  if (device_._dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported; // Bail if DMP is not supported

  uint8_t androidSensor = sensor_type_2_android_sensor(sensor); // Convert sensor from enum inv_icm20948_sensor to Android numbering

  if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
    return ICM_20948_Stat_SensorNotSupported; // Bail if the sensor is not supported (TO DO: Support B2S etc)

  // Convert the Android sensor into a bit mask for DATA_OUT_CTL1
  uint16_t delta = inv_androidSensor_to_control_bits[androidSensor];
  if (delta == 0xFFFF)
    return ICM_20948_Stat_SensorNotSupported; // Bail if the sensor is not supported

  // Convert the Android sensor number into a bitmask and set or clear that bit in _enabled_Android_0 / _enabled_Android_1
  unsigned long androidSensorAsBitMask;
  if (androidSensor < 32) // Sensors 0-31
  {
    androidSensorAsBitMask = 1L << androidSensor;
    if (state == 0) // Should we disable the sensor?
    {
      device_._enabled_Android_0 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
    }
    else
    {
      device_._enabled_Android_0 |= androidSensorAsBitMask; // Set the bit to enable the sensor
    }
  }
  else // Sensors 32-
  {
    androidSensorAsBitMask = 1L << (androidSensor - 32);
    if (state == 0) // Should we disable the sensor?
    {
      device_._enabled_Android_1 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
    }
    else
    {
      device_._enabled_Android_1 |= androidSensorAsBitMask; // Set the bit to enable the sensor
    }
  }

  // Now we know androidSensor is valid, reconstruct the value for DATA_OUT_CTL1 from _enabled_Android_0 and _enabled_Android_0
  delta = 0; // Clear delta
  for (int i = 0; i < 32; i++)
  {
    androidSensorAsBitMask = 1L << i;
    if ((device_._enabled_Android_0 & androidSensorAsBitMask) > 0) // Check if the Android sensor (0-31) is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i]; // If it is, or the required bits into delta
    }
    if ((device_._enabled_Android_1 & androidSensorAsBitMask) > 0) // Check if the Android sensor (32-) is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i + 32]; // If it is, or the required bits into delta
    }
    // Also check which bits need to be set in the Data Ready Status and Motion Event Control registers
    // Compare to INV_NEEDS_ACCEL_MASK, INV_NEEDS_GYRO_MASK and INV_NEEDS_COMPASS_MASK
    // See issue #150 - thank you @dobodu
    if (((device_._enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK) > 0)
    || ((device_._enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Accel;
      inv_event_control |= DMP_Motion_Event_Control_Accel_Calibr;
    }
    if (((device_._enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK) > 0)
    || ((device_._enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Gyro;
      inv_event_control |= DMP_Motion_Event_Control_Gyro_Calibr;
    }
    if (((device_._enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK) > 0)
    || ((device_._enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Secondary_Compass;
      inv_event_control |= DMP_Motion_Event_Control_Compass_Calibr;
    }
  }

  result = sleep_(false); // Make sure chip is awake
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  result = lowPower_(false); // Make sure chip is not in low power state
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  // Check if Accel, Gyro/Gyro_Calibr or Compass_Calibr/Quat9/GeoMag/Compass are to be enabled. If they are then we need to request the accuracy data via header2.
  uint16_t delta2 = 0;
  if ((delta & DMP_Data_Output_Control_1_Accel) > 0)
  {
    delta2 |= DMP_Data_Output_Control_2_Accel_Accuracy;
  }
  if (((delta & DMP_Data_Output_Control_1_Gyro_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Gyro) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Gyro_Accuracy;
  }
  if (((delta & DMP_Data_Output_Control_1_Compass_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Compass) > 0) || ((delta & DMP_Data_Output_Control_1_Quat9) > 0) || ((delta & DMP_Data_Output_Control_1_Geomag) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Compass_Accuracy;
  }
  // TO DO: Add DMP_Data_Output_Control_2_Pickup etc. if required

  // Write the sensor control bits into memory address DATA_OUT_CTL1
  unsigned char data_output_control_reg[2];
  data_output_control_reg[0] = (unsigned char)(delta >> 8);
  data_output_control_reg[1] = (unsigned char)(delta & 0xff);
  device_._dataOutCtl1 = delta; // Diagnostics
  result = inv_icm20948_write_mems_(DATA_OUT_CTL1, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  // Write the 'header2' sensor control bits into memory address DATA_OUT_CTL2
  data_output_control_reg[0] = (unsigned char)(delta2 >> 8);
  data_output_control_reg[1] = (unsigned char)(delta2 & 0xff);
  device_._dataOutCtl2 = delta2; // Diagnostics
  result = inv_icm20948_write_mems_(DATA_OUT_CTL2, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  // Set the DATA_RDY_STATUS register
  data_output_control_reg[0] = (unsigned char)(data_rdy_status >> 8);
  data_output_control_reg[1] = (unsigned char)(data_rdy_status & 0xff);
  device_._dataRdyStatus = data_rdy_status; // Diagnostics
  result = inv_icm20948_write_mems_(DATA_RDY_STATUS, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  // Check which extra bits need to be set in the Motion Event Control register
  if ((delta & DMP_Data_Output_Control_1_Quat9) > 0)
  {
    inv_event_control |= DMP_Motion_Event_Control_9axis;
  }
  if (((delta & DMP_Data_Output_Control_1_Step_Detector) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_0) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_1) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_2) > 0))
  {
    inv_event_control |= DMP_Motion_Event_Control_Pedometer_Interrupt;
  }
  if ((delta & DMP_Data_Output_Control_1_Geomag) > 0)
  {
    inv_event_control |= DMP_Motion_Event_Control_Geomag;
  }

  // Set the MOTION_EVENT_CTL register
  data_output_control_reg[0] = (unsigned char)(inv_event_control >> 8);
  data_output_control_reg[1] = (unsigned char)(inv_event_control & 0xff);
  device_._motionEventCtl = inv_event_control; // Diagnostics
  result = inv_icm20948_write_mems_(MOTION_EVENT_CTL, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  result = lowPower_(true); // Put chip into low power state
  if (result != ICM_20948_Stat_Ok)
    return result;

  return result;
}

ICM_20948_Status_e ICM20948Component::enableDMPSensorInt_(enum inv_icm20948_sensor sensor, bool state)
{
  // steps_.push_back("enableDMPSensorInt_");
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;

  if (device_._dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported; // Bail if DMP is not supported

  uint8_t androidSensor = sensor_type_2_android_sensor(sensor); // Convert sensor from enum inv_icm20948_sensor to Android numbering

  if (androidSensor > ANDROID_SENSOR_NUM_MAX)
    return ICM_20948_Stat_SensorNotSupported; // Bail if the sensor is not supported

  // Convert the Android sensor into a bit mask for DATA_OUT_CTL1
  uint16_t delta = inv_androidSensor_to_control_bits[androidSensor];
  if (delta == 0xFFFF)
    return ICM_20948_Stat_SensorNotSupported; // Bail if the sensor is not supported

  // Convert the Android sensor number into a bitmask and set or clear that bit in _enabled_Android_intr_0 / _enabled_Android_intr_1
  unsigned long androidSensorAsBitMask;
  if (androidSensor < 32) // Sensors 0-31
  {
    androidSensorAsBitMask = 1L << androidSensor;
    if (state == 0) // Should we disable the sensor interrupt?
    {
      device_._enabled_Android_intr_0 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor interrupt
    }
    else
    {
      device_._enabled_Android_intr_0 |= androidSensorAsBitMask; // Set the bit to enable the sensor interrupt
    }
  }
  else // Sensors 32-
  {
    androidSensorAsBitMask = 1L << (androidSensor - 32);
    if (state == 0) // Should we disable the sensor?
    {
      device_._enabled_Android_intr_1 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor interrupt
    }
    else
    {
      device_._enabled_Android_intr_1 |= androidSensorAsBitMask; // Set the bit to enable the sensor interrupt
    }
  }

  // Now we know androidSensor is valid, reconstruct the value for DATA_INTR_CTL from _enabled_Android_intr_0 and _enabled_Android_intr_0
  delta = 0; // Clear delta
  for (int i = 0; i < 32; i++)
  {
    androidSensorAsBitMask = 1L << i;
    if ((device_._enabled_Android_intr_0 & androidSensorAsBitMask) > 0) // Check if the Android sensor (0-31) interrupt is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i]; // If it is, or the required bits into delta
    }
    if ((device_._enabled_Android_intr_1 & androidSensorAsBitMask) > 0) // Check if the Android sensor (32-) interrupt is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i + 32]; // If it is, or the required bits into delta
    }
  }

  result = sleep_(false); // Make sure chip is awake
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  result = lowPower_(false); // Make sure chip is not in low power state
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  unsigned char data_intr_ctl[2];

  data_intr_ctl[0] = (unsigned char)(delta >> 8);
  data_intr_ctl[1] = (unsigned char)(delta & 0xff);
  device_._dataIntrCtl = delta; // Diagnostics
  
  // Write the interrupt control bits into memory address DATA_INTR_CTL
  result = inv_icm20948_write_mems_(DATA_INTR_CTL, 2, (const unsigned char *)&data_intr_ctl);

  result = lowPower_(true); // Put chip into low power state
  if (result != ICM_20948_Stat_Ok)
    return result;

  return result;
}

ICM_20948_Status_e ICM20948Component::writeDMPmems_(unsigned short reg, unsigned int length, const unsigned char *data)
{
  // steps_.push_back("writeDMPmems_");
  if (device_._dmp_firmware_available == true) // Should we attempt to write to the DMP?
  {
    status_ = inv_icm20948_write_mems_(reg, length, data);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::readDMPmems_(unsigned short reg, unsigned int length, unsigned char *data)
{
  // steps_.push_back("readDMPmems_");
  if (device_._dmp_firmware_available == true) // Should we attempt to read from the DMP?
  {
    status_ = inv_icm20948_read_mems_(reg, length, data);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setDMPODRrate_(enum DMP_ODR_Registers odr_reg, int interval)
{
  // steps_.push_back("setDMPODRrate_");
  // Set the ODR registers and clear the ODR counter

  // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate (225Hz) / ODR ) - 1
  // E.g. For a 25Hz ODR rate, value= (225/25) -1 = 8.

  // During run-time, if an ODR is changed, the corresponding rate counter must be reset.
  // To reset, write 2-byte {0,0} to DMP using keys below for a particular sensor:

  ICM_20948_Status_e result = ICM_20948_Stat_Ok;
  ICM_20948_Status_e result2 = ICM_20948_Stat_Ok;

  if (device_._dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported;

  unsigned char odr_reg_val[2];
  odr_reg_val[0] = (unsigned char)(interval >> 8);
  odr_reg_val[1] = (unsigned char)(interval & 0xff);

  unsigned char odr_count_zero[2] = {0x00, 0x00};

  result = sleep_(false); // Make sure chip is awake
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  result = lowPower_(false); // Make sure chip is not in low power state
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  switch (odr_reg)
  {
  case DMP_ODR_Reg_Cpass_Calibr:
  {
    result = inv_icm20948_write_mems_(ODR_CPASS_CALIBR, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_CPASS_CALIBR, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Gyro_Calibr:
  {
    result = inv_icm20948_write_mems_(ODR_GYRO_CALIBR, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_GYRO_CALIBR, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Pressure:
  {
    result = inv_icm20948_write_mems_(ODR_PRESSURE, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_PRESSURE, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Geomag:
  {
    result = inv_icm20948_write_mems_(ODR_GEOMAG, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_GEOMAG, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_PQuat6:
  {
    result = inv_icm20948_write_mems_(ODR_PQUAT6, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_PQUAT6, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Quat9:
  {
    result = inv_icm20948_write_mems_(ODR_QUAT9, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_QUAT9, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Quat6:
  {
    result = inv_icm20948_write_mems_(ODR_QUAT6, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_QUAT6, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_ALS:
  {
    result = inv_icm20948_write_mems_(ODR_ALS, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_ALS, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Cpass:
  {
    result = inv_icm20948_write_mems_(ODR_CPASS, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_CPASS, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Gyro:
  {
    result = inv_icm20948_write_mems_(ODR_GYRO, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_GYRO, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Accel:
  {
    result = inv_icm20948_write_mems_(ODR_ACCEL, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems_(ODR_CNTR_ACCEL, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  default:
    result = ICM_20948_Stat_InvalDMPRegister;
    break;
  }

  result = lowPower_(true); // Put chip into low power state
  if (result != ICM_20948_Stat_Ok)
    return result;

  if (result2 > result)
    result = result2; // Return the highest error

  return result;
}

ICM_20948_Status_e ICM20948Component::readDMPdataFromFIFO_(icm_20948_DMP_data_t *data)
{
  // steps_.push_back("readDMPdataFromFIFO_");
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;
  uint8_t fifoBytes[icm_20948_DMP_Maximum_Bytes]; // Interim storage for the FIFO data

  if (device_._dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported;

  // Check how much data is in the FIFO
  uint16_t fifo_count;
  result = getFIFOcount_(&fifo_count);
  if (result != ICM_20948_Stat_Ok)
    return result;

  if (fifo_count < icm_20948_DMP_Header_Bytes) // Has a 2-byte header arrived?
    return ICM_20948_Stat_FIFONoDataAvail;     // Bail if no header is available

  // Read the header (2 bytes)
  data->header = 0; // Clear the existing header
  uint16_t aShort = 0;
  result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Header_Bytes);
  if (result != ICM_20948_Stat_Ok)
    return result;
  for (int i = 0; i < icm_20948_DMP_Header_Bytes; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8)); // MSB first
  }
  data->header = aShort;                    // Store the header in data->header
  fifo_count -= icm_20948_DMP_Header_Bytes; // Decrement the count

  // If the header indicates a header2 is present then read that now
  data->header2 = 0;                                  // Clear the existing header2
  if ((data->header & DMP_header_bitmap_Header2) > 0) // If the header2 bit is set
  {
    if (fifo_count < icm_20948_DMP_Header2_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Header2_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if no header2 is available
    // Read the header (2 bytes)
    aShort = 0;
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Header2_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Header2_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->header2 = aShort;                    // Store the header2 in data->header2
    fifo_count -= icm_20948_DMP_Header2_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Accel) > 0) // case DMP_header_bitmap_Accel:
  {
    if (fifo_count < icm_20948_DMP_Raw_Accel_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Raw_Accel_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Raw_Accel_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Raw_Accel_Bytes; i++)
    {
      data->Raw_Accel.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Raw_Accel_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro) > 0) // case DMP_header_bitmap_Gyro:
  {
    if (fifo_count < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes)) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes))
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes));
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes); i++)
    {
      data->Raw_Gyro.Bytes[DMP_Raw_Gyro_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes); // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Compass) > 0) // case DMP_header_bitmap_Compass:
  {
    if (fifo_count < icm_20948_DMP_Compass_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Compass_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Compass_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Compass_Bytes; i++)
    {
      data->Compass.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Compass_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_ALS) > 0) // case DMP_header_bitmap_ALS:
  {
    if (fifo_count < icm_20948_DMP_ALS_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_ALS_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_ALS_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_ALS_Bytes; i++)
    {
      data->ALS[i] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_ALS_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Quat6) > 0) // case DMP_header_bitmap_Quat6:
  {
    if (fifo_count < icm_20948_DMP_Quat6_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Quat6_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Quat6_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Quat6_Bytes; i++)
    {
      data->Quat6.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Quat6_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Quat9) > 0) // case DMP_header_bitmap_Quat9:
  {
    if (fifo_count < icm_20948_DMP_Quat9_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Quat9_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Quat9_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Quat9_Bytes; i++)
    {
      data->Quat9.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Quat9_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_PQuat6) > 0) // case DMP_header_bitmap_PQuat6:
  {
    if (fifo_count < icm_20948_DMP_PQuat6_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_PQuat6_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_PQuat6_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_PQuat6_Bytes; i++)
    {
      data->PQuat6.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_PQuat6_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Geomag) > 0) // case DMP_header_bitmap_Geomag:
  {
    if (fifo_count < icm_20948_DMP_Geomag_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Geomag_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Geomag_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Geomag_Bytes; i++)
    {
      data->Geomag.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Geomag_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Pressure) > 0) // case DMP_header_bitmap_Pressure:
  {
    if (fifo_count < icm_20948_DMP_Pressure_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Pressure_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Pressure_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Pressure_Bytes; i++)
    {
      data->Pressure[i] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_Pressure_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro_Calibr) > 0) // case DMP_header_bitmap_Gyro_Calibr:
  {
    // lcm20948MPUFifoControl.c suggests icm_20948_DMP_Gyro_Calibr_Bytes is not supported
    // and looking at DMP frames which have the Gyro_Calibr bit set, that certainly seems to be true.
    // So, we'll skip this...:
    /*
			if (fifo_count < icm_20948_DMP_Gyro_Calibr_Bytes) // Check if we need to read the FIFO count again
			{
					result = getFIFOcount_(&fifo_count);
					if (result != ICM_20948_Stat_Ok)
							return result;
			}
			if (fifo_count < icm_20948_DMP_Gyro_Calibr_Bytes)
					return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
			result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Gyro_Calibr_Bytes);
			if (result != ICM_20948_Stat_Ok)
					return result;
			for (int i = 0; i < icm_20948_DMP_Gyro_Calibr_Bytes; i++)
			{
					data->Gyro_Calibr.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
			}
			fifo_count -= icm_20948_DMP_Gyro_Calibr_Bytes; // Decrement the count
			*/
  }

  if ((data->header & DMP_header_bitmap_Compass_Calibr) > 0) // case DMP_header_bitmap_Compass_Calibr:
  {
    if (fifo_count < icm_20948_DMP_Compass_Calibr_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Compass_Calibr_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Compass_Calibr_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Compass_Calibr_Bytes; i++)
    {
      data->Compass_Calibr.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Compass_Calibr_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Step_Detector) > 0) // case DMP_header_bitmap_Step_Detector:
  {
    if (fifo_count < icm_20948_DMP_Step_Detector_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Step_Detector_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Step_Detector_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    uint32_t aWord = 0;
    for (int i = 0; i < icm_20948_DMP_Step_Detector_Bytes; i++)
    {
      aWord |= ((uint32_t)fifoBytes[i]) << (24 - (i * 8)); // MSB first
    }
    data->Pedometer_Timestamp = aWord;
    fifo_count -= icm_20948_DMP_Step_Detector_Bytes; // Decrement the count
  }

  // Now check for header2 features

  if ((data->header2 & DMP_header2_bitmap_Accel_Accuracy) > 0) // case DMP_header2_bitmap_Accel_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Accel_Accuracy_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Accel_Accuracy_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Accel_Accuracy_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Accel_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Accel_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Accel_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Gyro_Accuracy) > 0) // case DMP_header2_bitmap_Gyro_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Gyro_Accuracy_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Gyro_Accuracy_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Gyro_Accuracy_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Gyro_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Gyro_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Gyro_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Compass_Accuracy) > 0) // case DMP_header2_bitmap_Compass_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Compass_Accuracy_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Compass_Accuracy_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Compass_Accuracy_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Compass_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Compass_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Compass_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Fsync) > 0) // case DMP_header2_bitmap_Fsync:
  {
    // lcm20948MPUFifoControl.c suggests icm_20948_DMP_Fsync_Detection_Bytes is not supported.
    // So, we'll skip this just in case...:
    /*
			if (fifo_count < icm_20948_DMP_Fsync_Detection_Bytes) // Check if we need to read the FIFO count again
			{
					result = getFIFOcount_(&fifo_count);
					if (result != ICM_20948_Stat_Ok)
							return result;
			}
			if (fifo_count < icm_20948_DMP_Fsync_Detection_Bytes)
					return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
			aShort = 0;
			result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Fsync_Detection_Bytes);
			if (result != ICM_20948_Stat_Ok)
					return result;
			for (int i = 0; i < icm_20948_DMP_Fsync_Detection_Bytes; i++)
			{
					aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
			}
			data->Fsync_Delay_Time = aShort;
			fifo_count -= icm_20948_DMP_Fsync_Detection_Bytes; // Decrement the count
			*/
  }

  if ((data->header2 & DMP_header2_bitmap_Pickup) > 0) // case DMP_header2_bitmap_Pickup:
  {
    if (fifo_count < icm_20948_DMP_Pickup_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Pickup_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Pickup_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Pickup_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Pickup = aShort;
    fifo_count -= icm_20948_DMP_Pickup_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Activity_Recog) > 0) // case DMP_header2_bitmap_Activity_Recog:
  {
    if (fifo_count < icm_20948_DMP_Activity_Recognition_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Activity_Recognition_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Activity_Recognition_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Activity_Recognition_Bytes; i++)
    {
      data->Activity_Recognition.Bytes[DMP_Activity_Recognition_Byte_Ordering[i]] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_Activity_Recognition_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Secondary_On_Off) > 0) // case DMP_header2_bitmap_Secondary_On_Off:
  {
    if (fifo_count < icm_20948_DMP_Secondary_On_Off_Bytes) // Check if we need to read the FIFO count again
    {
      result = getFIFOcount_(&fifo_count);
      if (result != ICM_20948_Stat_Ok)
        return result;
    }
    if (fifo_count < icm_20948_DMP_Secondary_On_Off_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Secondary_On_Off_Bytes);
    if (result != ICM_20948_Stat_Ok)
      return result;
    for (int i = 0; i < icm_20948_DMP_Secondary_On_Off_Bytes; i++)
    {
      data->Secondary_On_Off.Bytes[DMP_Secondary_On_Off_Byte_Ordering[i]] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_Secondary_On_Off_Bytes; // Decrement the count
  }

  // Finally, extract the footer (gyro count)
  if (fifo_count < icm_20948_DMP_Footer_Bytes) // Check if we need to read the FIFO count again
  {
    result = getFIFOcount_(&fifo_count);
    if (result != ICM_20948_Stat_Ok)
      return result;
  }
  if (fifo_count < icm_20948_DMP_Footer_Bytes)
    return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
  aShort = 0;
  result = readFIFO_(&fifoBytes[0], icm_20948_DMP_Footer_Bytes);
  if (result != ICM_20948_Stat_Ok)
    return result;
  for (int i = 0; i < icm_20948_DMP_Footer_Bytes; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
  }
  data->Footer = aShort;
  fifo_count -= icm_20948_DMP_Footer_Bytes; // Decrement the count

  if (fifo_count > 0) // Check if there is still data waiting to be read
    return ICM_20948_Stat_FIFOMoreDataAvail;

  return result;
}

ICM_20948_Status_e ICM20948Component::setGyroSF_(unsigned char div, int gyro_level)
{
  // steps_.push_back("setGyroSF_");
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;

  if (device_._dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported;

  // gyro_level should be set to 4 regardless of fullscale, due to the addition of API dmp_icm20648_set_gyro_fsr()
  gyro_level = 4;

  // First read the TIMEBASE_CORRECTION_PLL register from Bank 1
  int8_t pll; // Signed. Typical value is 0x18
  result = setBank_(1);
  result = read_i2c(AGB1_REG_TIMEBASE_CORRECTION_PLL, (uint8_t *)&pll, 1);
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  device_._gyroSFpll = pll; // Record the PLL value so we can debug print it

  // Now calculate the Gyro SF using code taken from the InvenSense example (inv_icm20948_set_gyro_sf)

  long gyro_sf;

  unsigned long long const MagicConstant = 264446880937391LL;
  unsigned long long const MagicConstantScale = 100000LL;
  unsigned long long ResultLL;

  if (pll & 0x80)
  {
    ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 - (pll & 0x7F)) / MagicConstantScale);
  }
  else
  {
    ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 + pll) / MagicConstantScale);
  }
  /*
	    In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
	    Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
	    the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
	*/
  if (ResultLL > 0x7FFFFFFF)
    gyro_sf = 0x7FFFFFFF;
  else
    gyro_sf = (long)ResultLL;

  device_._gyroSF = gyro_sf; // Record value so we can debug print it

  // Finally, write the value to the DMP GYRO_SF register
  unsigned char gyro_sf_reg[4];
  gyro_sf_reg[0] = (unsigned char)(gyro_sf >> 24);
  gyro_sf_reg[1] = (unsigned char)(gyro_sf >> 16);
  gyro_sf_reg[2] = (unsigned char)(gyro_sf >> 8);
  gyro_sf_reg[3] = (unsigned char)(gyro_sf & 0xff);
  result = inv_icm20948_write_mems_(GYRO_SF, 4, (const unsigned char*)&gyro_sf_reg);

  return result;
}

// Combine all of the DMP start-up code from the earlier DMP examples
// This function is defined as __attribute__((weak)) so you can overwrite it if you want to,
//   e.g. to modify the sample rate
ICM_20948_Status_e ICM20948Component::initializeDMP_(void)
{
  // steps_.push_back("initializeDMP_");
  // First, let's check if the DMP is available
  if (device_._dmp_firmware_available != true)
  {
    ESP_LOGE(TAG, "startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
    return ICM_20948_Stat_DMPNotSupported;
  }

  ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  result = i2cControllerConfigurePeripheral_(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = i2cControllerConfigurePeripheral_(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;
  
  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank_(3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write_i2c(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = setClockSource_(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails
  
  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank_(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write_i2c(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register
  
  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = setSampleMode_(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;
  
  // Disable the FIFO
  result = enableFIFO_(false); if (result > worstResult) worstResult = result;
  
  // Disable the DMP
  result = enableDMP_(false); if (result > worstResult) worstResult = result;
  
  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = setFullScale_((ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;
  // return result;
  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = enableDLPF_(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank_(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write_i2c(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write_i2c(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = intEnableRawDataReady_(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = resetFIFO_(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.g = 4; // 225Hz
  //mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate_((ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress_(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware_(); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress_(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank_(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write_i2c(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank_(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write_i2c(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = writeDMPmems_(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems_(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems_(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems_(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems_(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF_(19, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems_(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems_(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems_(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems_(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems_(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems_(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

  return worstResult;
}

ICM_20948_Status_e ICM20948Component::startupMagnetometer_(bool minimal)
{
  // steps_.push_back("startupMagnetometer_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  i2cMasterPassthrough_(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  i2cMasterEnable_(true);

  resetMag_();

  //After a ICM reset the Mag sensor may stop responding over the I2C master
  //Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < MAX_MAGNETOMETER_STARTS)
  {
    tries++;

    //See if we can read the WhoIAm register correctly
    retval = magWhoIAm_();
    if (retval == ICM_20948_Stat_Ok)
      break; //WIA matched!

    i2cMasterReset_(); //Otherwise, reset the master I2C and try again

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (tries == MAX_MAGNETOMETER_STARTS)
  {
    ESP_LOGE(TAG, "startupMagnetometer: reached MAX_MAGNETOMETER_STARTS (%d). Returning ICM_20948_Stat_WrongID", (int)MAX_MAGNETOMETER_STARTS);
    status_ = ICM_20948_Stat_WrongID;
    return status_;
  }
  else
  {
    ESP_LOGE(TAG, "startupMagnetometer: successful magWhoIAm after %d trie(s).", (int)tries);
  }

  //Return now if minimal is true. The mag will be configured manually for the DMP
  if (minimal) // Return now if minimal is true
  {
    ESP_LOGE(TAG, "startupMagnetometer: minimal startup complete!");
    return status_;
  }

  //Set up magnetometer
  AK09916_CNTL2_Reg_t reg;
  reg.MODE = AK09916_mode_cont_100hz;
  reg.reserved_0 = 0; // Make sure the unused bits are clear. Probably redundant, but prevents confusion when looking at the I2C traffic
  retval = writeMag_(AK09916_REG_CNTL2, (uint8_t *)&reg);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupMagnetometer: writeMag returned: %d", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = i2cControllerConfigurePeripheral_(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "startupMagnetometer: i2cMasterConfigurePeripheral returned: %d", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  return status_;
}

ICM_20948_Status_e ICM20948Component::magWhoIAm_(void)
{
  // steps_.push_back("magWhoIAm_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t whoiam1, whoiam2;
  whoiam1 = readMag_(AK09916_REG_WIA1);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status_;
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "magWhoIAm: whoiam1: %d", ((int)whoiam1));
    ESP_LOGE(TAG, " (should be 72) readMag set status to: %d", statusToString_(status_));
    return retval;
  }
  whoiam2 = readMag_(AK09916_REG_WIA2);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status_;
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(TAG, "magWhoIAm: whoiam1: %d", (int)whoiam1);
    ESP_LOGE(TAG, " (should be 72) whoiam2: %d", (int)whoiam2);
    ESP_LOGE(TAG, " (should be 9) readMag set status to: %d", statusToString_(status_));
    return retval;
  }

  if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
  {
    retval = ICM_20948_Stat_Ok;
    status_ = retval;
    return status_;
  }

  ESP_LOGD(TAG, "magWhoIAm: whoiam1: %d (should be 72) whoiam2: %d (should be 9). Returning ICM_20948_Stat_WrongID", (int)whoiam1, (int)whoiam2);

  retval = ICM_20948_Stat_WrongID;
  status_ = retval;
  return status_;
}

void ICM20948Component::updateBiasStoreSum_(biasStore *store) // Update the bias store checksum
{
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

bool ICM20948Component::isBiasStoreValid_(biasStore *store) // Returns true if the header and checksum are valid
{
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}

void ICM20948Component::printBiases_(biasStore *store)
{
  ESP_LOGD(TAG, "Gyro X: %d", store->biasGyroX);
  ESP_LOGD(TAG, "Gyro Y: %d", store->biasGyroY);
  ESP_LOGD(TAG, "Gyro Z: %d", store->biasGyroZ);
  ESP_LOGD(TAG, "Accel X: %d", store->biasAccelX);
  ESP_LOGD(TAG, "Accel Y: %d", store->biasAccelY);
  ESP_LOGD(TAG, "Accel Z: %d", store->biasAccelZ);
  ESP_LOGD(TAG, "CPass X: %d", store->biasCPassX);
  ESP_LOGD(TAG, "CPass Y: %d", store->biasCPassY);
  ESP_LOGD(TAG, "CPass Z: %d", store->biasCPassZ);

}

void ICM20948Component::set_calibrate_next_boot(bool enabled) {
  if (enabled) {
    ESP_LOGD(TAG, "Setting calibration to be applied on next boot");
    bias_store_.calibrate_on_boot = true;
  } else {
    ESP_LOGD(TAG, "Clearing calibration to be applied on next boot");
    bias_store_.calibrate_on_boot = false;
  }

  // Save the bias store to flash
  updateBiasStoreSum_(&bias_store_);
  ESP_LOGD(TAG, "Saving bias store to flash");


  if (!bias_store_pref_.save(&bias_store_)) {
    ESP_LOGE(TAG, "Failed to save bias store to flash");
  } else {
    ESP_LOGD(TAG, "Bias store saved successfully");
  }
}

ICM_20948_Status_e ICM20948Component::write_i2c(uint8_t reg, uint8_t *data, uint32_t len) {
  // steps_.push_back("write_i2c");

  esphome::i2c::ErrorCode status = write_register(reg, data, len, true);
  if (status == esphome::i2c::ErrorCode::NO_ERROR) {
    // // steps_.push_back("write_i2c: write data succeeded");
  } else {
    // steps_.push_back("write_i2c: write data failed");
    // steps_.push_back("write_i2c: register: " + std::to_string(static_cast<int>(reg)));
    // steps_.push_back("write_i2c: ErrorCode: " + std::to_string(static_cast<int>(status)));
    ESP_LOGE(TAG, "I2C write to reg 0x%02X failed", reg);
    return ICM_20948_Stat_Err;
  }

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM20948Component::read_i2c(uint8_t reg, uint8_t *buff, uint32_t len) {
  // steps_.push_back("read_i2c" + std::string(" reg: ") + std::to_string(static_cast<int>(reg)) + " len: " + std::to_string(len));

  esphome::i2c::ErrorCode status = read_register(reg, buff, len, false);
  if (status == esphome::i2c::ErrorCode::NO_ERROR) {
    // // steps_.push_back("read_i2c: read data succeeded");
  } else {
    // steps_.push_back("read_i2c: read data failed");
    // steps_.push_back("read_i2c: register: " + std::to_string(static_cast<int>(reg)));
    // steps_.push_back("read_i2c: ErrorCode: " + std::to_string(static_cast<int>(status)));
    ESP_LOGE(TAG, "I2C read from reg 0x%02X failed", reg);
    return ICM_20948_Stat_Err;
  }

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM20948Component::inv_icm20948_read_mems_(unsigned short reg, unsigned int length, unsigned char *data)
{
  // steps_.push_back("inv_icm20948_read_mems_");
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;
  unsigned int bytesRead = 0;
  unsigned int thisLen;
  unsigned char lBankSelected;
  unsigned char lStartAddrSelected;

  if (!data)
  {
    return ICM_20948_Stat_NoData;
  }

  result = setBank_(0); // Set bank 0
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  lBankSelected = (reg >> 8);

  if (lBankSelected != device_._last_mems_bank)
  {
    device_._last_mems_bank = lBankSelected;
    result = write_i2c(AGB0_REG_MEM_BANK_SEL, &lBankSelected, 1);
    if (result != ICM_20948_Stat_Ok)
    {
      return result;
    }
  }

  while (bytesRead < length)
  {
    lStartAddrSelected = (reg & 0xff);

    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
		   Contents are changed after read or write of the selected memory.
		   This register must be written prior to each access to initialize the register to the proper starting address.
		   The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */

    result = write_i2c(AGB0_REG_MEM_START_ADDR, &lStartAddrSelected, 1);
    if (result != ICM_20948_Stat_Ok)
    {
      return result;
    }

    if (length - bytesRead <= INV_MAX_SERIAL_READ)
      thisLen = length - bytesRead;
    else
      thisLen = INV_MAX_SERIAL_READ;

    /* Read data */

    result = read_i2c(AGB0_REG_MEM_R_W, &data[bytesRead], thisLen);
    if (result != ICM_20948_Stat_Ok)
    {
      return result;
    }

    bytesRead += thisLen;
    reg += thisLen;
  }

  return result;
}

ICM_20948_Status_e ICM20948Component::inv_icm20948_write_mems_(unsigned short reg, unsigned int length, const unsigned char *data) {
  // steps_.push_back("inv_icm20948_write_mems_");
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;
  unsigned int bytesWritten = 0;
  unsigned int thisLen;
  unsigned char lBankSelected;
  unsigned char lStartAddrSelected;

  if (!data)
  {
    return ICM_20948_Stat_NoData;
  }

  result = setBank_(0); // Set bank 0
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  lBankSelected = (reg >> 8);

  if (lBankSelected != device_._last_mems_bank)
  {
    device_._last_mems_bank = lBankSelected;
    result = write_i2c(AGB0_REG_MEM_BANK_SEL, &lBankSelected, 1);
    if (result != ICM_20948_Stat_Ok)
    {
      return result;
    }
  }

  while (bytesWritten < length)
  {
    lStartAddrSelected = (reg & 0xff);

    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */

    result = write_i2c(AGB0_REG_MEM_START_ADDR, &lStartAddrSelected, 1);
    if (result != ICM_20948_Stat_Ok)
    {
      return result;
    }

    if (length - bytesWritten <= INV_MAX_SERIAL_WRITE)
      thisLen = length - bytesWritten;
    else
      thisLen = INV_MAX_SERIAL_WRITE;

    /* Write data */

    result = write_i2c(AGB0_REG_MEM_R_W, (uint8_t *)&data[bytesWritten], thisLen);
    if (result != ICM_20948_Stat_Ok)
    {
      return result;
    }

    bytesWritten += thisLen;
    reg += thisLen;
  }

  return result;
}

ICM_20948_Status_e ICM20948Component::intEnable_(ICM_20948_INT_enable_t *write, ICM_20948_INT_enable_t *read) {
  // steps_.push_back("intEnable_");
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_INT_ENABLE_t en_0;
  ICM_20948_INT_ENABLE_1_t en_1;
  ICM_20948_INT_ENABLE_2_t en_2;
  ICM_20948_INT_ENABLE_3_t en_3;

  retval = setBank_(0); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    en_0.I2C_MST_INT_EN = write->I2C_MST_INT_EN;
    en_0.DMP_INT1_EN = write->DMP_INT1_EN;
    en_0.PLL_READY_EN = write->PLL_RDY_EN;
    en_0.WOM_INT_EN = write->WOM_INT_EN;
    en_0.reserved_0 = 0; // Clear RAM garbage
    en_0.REG_WOF_EN = write->REG_WOF_EN;
    en_1.RAW_DATA_0_RDY_EN = write->RAW_DATA_0_RDY_EN;
    en_1.reserved_0 = 0; // Clear RAM garbage
    en_2.individual.FIFO_OVERFLOW_EN_4 = write->FIFO_OVERFLOW_EN_4;
    en_2.individual.FIFO_OVERFLOW_EN_3 = write->FIFO_OVERFLOW_EN_3;
    en_2.individual.FIFO_OVERFLOW_EN_2 = write->FIFO_OVERFLOW_EN_2;
    en_2.individual.FIFO_OVERFLOW_EN_1 = write->FIFO_OVERFLOW_EN_1;
    en_2.individual.FIFO_OVERFLOW_EN_0 = write->FIFO_OVERFLOW_EN_0;
    en_2.individual.reserved_0 = 0; // Clear RAM garbage
    en_3.individual.FIFO_WM_EN_4 = write->FIFO_WM_EN_4;
    en_3.individual.FIFO_WM_EN_3 = write->FIFO_WM_EN_3;
    en_3.individual.FIFO_WM_EN_2 = write->FIFO_WM_EN_2;
    en_3.individual.FIFO_WM_EN_1 = write->FIFO_WM_EN_1;
    en_3.individual.FIFO_WM_EN_0 = write->FIFO_WM_EN_0;
    en_3.individual.reserved_0 = 0; // Clear RAM garbage

    retval = write_i2c(AGB0_REG_INT_ENABLE, (uint8_t *)&en_0, sizeof(ICM_20948_INT_ENABLE_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
    retval = write_i2c(AGB0_REG_INT_ENABLE_1, (uint8_t *)&en_1, sizeof(ICM_20948_INT_ENABLE_1_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
    retval = write_i2c(AGB0_REG_INT_ENABLE_2, (uint8_t *)&en_2, sizeof(ICM_20948_INT_ENABLE_2_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
    retval = write_i2c(AGB0_REG_INT_ENABLE_3, (uint8_t *)&en_3, sizeof(ICM_20948_INT_ENABLE_3_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    retval = read_i2c(AGB0_REG_INT_ENABLE, (uint8_t *)&en_0, sizeof(ICM_20948_INT_ENABLE_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
    retval = read_i2c(AGB0_REG_INT_ENABLE_1, (uint8_t *)&en_1, sizeof(ICM_20948_INT_ENABLE_1_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
    retval = read_i2c(AGB0_REG_INT_ENABLE_2, (uint8_t *)&en_2, sizeof(ICM_20948_INT_ENABLE_2_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
    retval = read_i2c(AGB0_REG_INT_ENABLE_3, (uint8_t *)&en_3, sizeof(ICM_20948_INT_ENABLE_3_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }

    read->I2C_MST_INT_EN = en_0.I2C_MST_INT_EN;
    read->DMP_INT1_EN = en_0.DMP_INT1_EN;
    read->PLL_RDY_EN = en_0.PLL_READY_EN;
    read->WOM_INT_EN = en_0.WOM_INT_EN;
    read->REG_WOF_EN = en_0.REG_WOF_EN;
    read->RAW_DATA_0_RDY_EN = en_1.RAW_DATA_0_RDY_EN;
    read->FIFO_OVERFLOW_EN_4 = en_2.individual.FIFO_OVERFLOW_EN_4;
    read->FIFO_OVERFLOW_EN_3 = en_2.individual.FIFO_OVERFLOW_EN_3;
    read->FIFO_OVERFLOW_EN_2 = en_2.individual.FIFO_OVERFLOW_EN_2;
    read->FIFO_OVERFLOW_EN_1 = en_2.individual.FIFO_OVERFLOW_EN_1;
    read->FIFO_OVERFLOW_EN_0 = en_2.individual.FIFO_OVERFLOW_EN_0;
    read->FIFO_WM_EN_4 = en_3.individual.FIFO_WM_EN_4;
    read->FIFO_WM_EN_3 = en_3.individual.FIFO_WM_EN_3;
    read->FIFO_WM_EN_2 = en_3.individual.FIFO_WM_EN_2;
    read->FIFO_WM_EN_1 = en_3.individual.FIFO_WM_EN_1;
    read->FIFO_WM_EN_0 = en_3.individual.FIFO_WM_EN_0;
  }

  return retval;
}

uint8_t ICM20948Component::sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor)
{
  switch (sensor)
  {
  case INV_ICM20948_SENSOR_ACCELEROMETER:
    return ANDROID_SENSOR_ACCELEROMETER; // 1
  case INV_ICM20948_SENSOR_GYROSCOPE:
    return ANDROID_SENSOR_GYROSCOPE; // 4
  case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
    return ANDROID_SENSOR_RAW_ACCELEROMETER; // 42
  case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
    return ANDROID_SENSOR_RAW_GYROSCOPE; // 43
  case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
    return ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED; // 14
  case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
    return ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED; // 16
  case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:
    return ANDROID_SENSOR_ACTIVITY_CLASSIFICATON; // 47
  case INV_ICM20948_SENSOR_STEP_DETECTOR:
    return ANDROID_SENSOR_STEP_DETECTOR; // 18
  case INV_ICM20948_SENSOR_STEP_COUNTER:
    return ANDROID_SENSOR_STEP_COUNTER; // 19
  case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
    return ANDROID_SENSOR_GAME_ROTATION_VECTOR; // 15
  case INV_ICM20948_SENSOR_ROTATION_VECTOR:
    return ANDROID_SENSOR_ROTATION_VECTOR; // 11
  case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
    return ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR; // 20
  case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
    return ANDROID_SENSOR_GEOMAGNETIC_FIELD; // 2
  case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
    return ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION; // 17
  case INV_ICM20948_SENSOR_FLIP_PICKUP:
    return ANDROID_SENSOR_FLIP_PICKUP; // 46
  case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:
    return ANDROID_SENSOR_WAKEUP_TILT_DETECTOR; // 41
  case INV_ICM20948_SENSOR_GRAVITY:
    return ANDROID_SENSOR_GRAVITY; // 9
  case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
    return ANDROID_SENSOR_LINEAR_ACCELERATION; // 10
  case INV_ICM20948_SENSOR_ORIENTATION:
    return ANDROID_SENSOR_ORIENTATION; // 3
  case INV_ICM20948_SENSOR_B2S:
    return ANDROID_SENSOR_B2S; // 45
  default:
    return ANDROID_SENSOR_NUM_MAX;
  }
}

inv_icm20948_sensor ICM20948Component::inv_icm20948_sensor_android_2_sensor_type(int sensor)
{
  switch (sensor)
  {
  case ANDROID_SENSOR_ACCELEROMETER:
    return INV_ICM20948_SENSOR_ACCELEROMETER;
  case ANDROID_SENSOR_GYROSCOPE:
    return INV_ICM20948_SENSOR_GYROSCOPE;
  case ANDROID_SENSOR_RAW_ACCELEROMETER:
    return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
  case ANDROID_SENSOR_RAW_GYROSCOPE:
    return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
  case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
    return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
  case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
    return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
  case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:
    return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
  case ANDROID_SENSOR_STEP_DETECTOR:
    return INV_ICM20948_SENSOR_STEP_DETECTOR;
  case ANDROID_SENSOR_STEP_COUNTER:
    return INV_ICM20948_SENSOR_STEP_COUNTER;
  case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
  case ANDROID_SENSOR_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_ROTATION_VECTOR;
  case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
  case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
    return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
  case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
    return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
  case ANDROID_SENSOR_FLIP_PICKUP:
    return INV_ICM20948_SENSOR_FLIP_PICKUP;
  case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:
    return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
  case ANDROID_SENSOR_GRAVITY:
    return INV_ICM20948_SENSOR_GRAVITY;
  case ANDROID_SENSOR_LINEAR_ACCELERATION:
    return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
  case ANDROID_SENSOR_ORIENTATION:
    return INV_ICM20948_SENSOR_ORIENTATION;
  case ANDROID_SENSOR_B2S:
    return INV_ICM20948_SENSOR_B2S;
  default:
    return INV_ICM20948_SENSOR_MAX;
  }
}

} // namespace icm20948
}  // namespace esphome