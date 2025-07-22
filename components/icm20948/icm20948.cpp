#include "icm20948.h"

#define MAX_MAGNETOMETER_STARTS 10

#define BIAS_STORE_PREF_KEY "iKYM4Rqc2EhHLxEV2gZLz86hpYYQaIxlmIfR0WW22Kap5MXE2lg4T0DSFYVaqpO"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace esphome {
namespace icm20948 {

static const char *const TAG = "icm20948.sensor";
unsigned long startupTime = millis();
const unsigned long calibration_timeout = 5*60*1000; // 5 minutes timeout for calibration

ICM_20948_Serif_t mySerif = {
    ICM20948Component::write_i2c, // write
    ICM20948Component::read_i2c,  // read
    NULL,
};

void ICM20948Component::setup() {
  uint32_t hash = fnv1_hash(BIAS_STORE_PREF_KEY);
  bias_store_pref_ = global_preferences->make_preference<biasStore>(hash);
  bias_store_pref_.load(&bias_store_);
  if (bias_store_pref_.load(&bias_store_)) {
    ESP_LOGD(TAG, "Loaded preferences: %s", BIAS_STORE_PREF_KEY);
  }

  // Initialize the ICM_20948_Device_t structure
  status_ = ICM_20948_init_struct(&device_);
  mySerif.user = (void *)this;
  ICM_20948_link_serif(&device_, &mySerif);

  bool initialized = false;
  while (!initialized) {
    device_._dmp_firmware_available = true; // Assume DMP firmware is available

    device_._firmware_loaded = false; // Initialize _firmware_loaded
    device_._last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_20948_set_bank.
    device_._last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.
    device_._gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    device_._gyroSFpll = 0;
    device_._enabled_Android_0 = 0;      // Keep track of which Android sensors are enabled: 0-31
    device_._enabled_Android_1 = 0;      // Keep track of which Android sensors are enabled: 32-
    device_._enabled_Android_intr_0 = 0; // Keep track of which Android sensor interrupts are enabled: 0-31
    device_._enabled_Android_intr_1 = 0; // Keep track of which Android sensor interrupts are enabled: 32-

    status_ = startupDefault(device_._dmp_firmware_available);
    if (status_ != ICM_20948_Stat_Ok)
    {
      ESP_LOGD(TAG, "begin: startupDefault returned: %d", status_);
    }

    if (status_ != ICM_20948_Stat_Ok)
    {
      ESP_LOGD(TAG, "Trying again...");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    else
    {
      initialized = true;
    }
  }

  ESP_LOGD(TAG, "Device connected successfully!");

  bool success = true; 
  success &= (initializeDMP() == ICM_20948_Stat_Ok);
  success &= (enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  // Enable additional sensors / features
  success &= (enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // TODO: make rate configurable
  success &= (setDMPODRrate(DMP_ODR_Reg_Quat9, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
  success &= (setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
  success &= (setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
  success &= (setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

  // Enable the FIFO
  success &= (enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    ESP_LOGD(TAG, "DMP enabled!");
  }
  else
  {
    ESP_LOGD(TAG, "Enable DMP failed!");
    this->mark_failed();
    return;
  }

  if (isBiasStoreValid_(&bias_store_))
  {
    // ESP_LOGI("ICM20948", "Running bias calibration from boot flag...");
    // this->run_bias_calibration();
    // calibrate_on_boot_.save(false);  // Clear it
    ESP_LOGD(TAG, "Bias data in EEPROM is valid. Restoring it...");
    success &= (setBiasGyroX(bias_store_.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (setBiasGyroY(bias_store_.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (setBiasGyroZ(bias_store_.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (setBiasAccelX(bias_store_.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (setBiasAccelY(bias_store_.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (setBiasAccelZ(bias_store_.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (setBiasCPassX(bias_store_.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (setBiasCPassY(bias_store_.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (setBiasCPassZ(bias_store_.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      ESP_LOGD(TAG, "Biases restored.");
      printBiases_(&bias_store_);
      return;
    }
    else
      ESP_LOGD(TAG, "Bias restore failed!");
  }

  // If we reach here, we either have no valid bias data or restoring it failed
  if (bias_store_.calibrate_on_boot) {
    ESP_LOGD(TAG, "Bias calibration on boot is enabled. Running calibration...");
    ESP_LOGD(TAG, "The biases will be saved in two minutes.");
    ESP_LOGD(TAG, "Before then:");
    ESP_LOGD(TAG, "* Rotate the sensor around all three axes");
    ESP_LOGD(TAG, "* Hold the sensor stationary in all six orientations for a few seconds");

    startupTime = millis();
  } else {
    ESP_LOGD(TAG, "Bias calibration on boot is disabled. Will not run calibration."); 
  }
}

void ICM20948Component::dump_config() {
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
  icm_20948_DMP_data_t data;
  readDMPdataFromFIFO(&data);

  while (millis() - startupTime < 5000) { //TODO 
    return; // Wait for 5 seconds before processing data
  }

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
    if (millis() - startupTime > calibration_timeout) { // After x minutes
      ESP_LOGD(TAG, "Calibration timeout reached. Saving biases...");
      
      biasStore store;
          
      bool success = (getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
      success &= (getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
      success &= (getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
      success &= (getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
      success &= (getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
      success &= (getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
      success &= (getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
      success &= (getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
      success &= (getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);
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

float ICM20948Component::get_setup_priority() const { return setup_priority::DATA; }

//Gyro Bias
ICM_20948_Status_e ICM20948Component::setBiasGyroX(int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char gyro_bias_reg[4];
    gyro_bias_reg[0] = (unsigned char)(newValue >> 24);
    gyro_bias_reg[1] = (unsigned char)(newValue >> 16);
    gyro_bias_reg[2] = (unsigned char)(newValue >> 8);
    gyro_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, GYRO_BIAS_X, 4, (const unsigned char*)&gyro_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setBiasGyroY(int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char gyro_bias_reg[4];
    gyro_bias_reg[0] = (unsigned char)(newValue >> 24);
    gyro_bias_reg[1] = (unsigned char)(newValue >> 16);
    gyro_bias_reg[2] = (unsigned char)(newValue >> 8);
    gyro_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, GYRO_BIAS_Y, 4, (const unsigned char*)&gyro_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setBiasGyroZ(int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char gyro_bias_reg[4];
    gyro_bias_reg[0] = (unsigned char)(newValue >> 24);
    gyro_bias_reg[1] = (unsigned char)(newValue >> 16);
    gyro_bias_reg[2] = (unsigned char)(newValue >> 8);
    gyro_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, GYRO_BIAS_Z, 4, (const unsigned char*)&gyro_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasGyroX( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, GYRO_BIAS_X, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasGyroY( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, GYRO_BIAS_Y, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasGyroZ( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, GYRO_BIAS_Z, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

//Accel Bias
ICM_20948_Status_e ICM20948Component::setBiasAccelX(int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char accel_bias_reg[4];
    accel_bias_reg[0] = (unsigned char)(newValue >> 24);
    accel_bias_reg[1] = (unsigned char)(newValue >> 16);
    accel_bias_reg[2] = (unsigned char)(newValue >> 8);
    accel_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, ACCEL_BIAS_X, 4, (const unsigned char*)&accel_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setBiasAccelY(int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char accel_bias_reg[4];
    accel_bias_reg[0] = (unsigned char)(newValue >> 24);
    accel_bias_reg[1] = (unsigned char)(newValue >> 16);
    accel_bias_reg[2] = (unsigned char)(newValue >> 8);
    accel_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, ACCEL_BIAS_Y, 4, (const unsigned char*)&accel_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setBiasAccelZ(int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char accel_bias_reg[4];
    accel_bias_reg[0] = (unsigned char)(newValue >> 24);
    accel_bias_reg[1] = (unsigned char)(newValue >> 16);
    accel_bias_reg[2] = (unsigned char)(newValue >> 8);
    accel_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, ACCEL_BIAS_Z, 4, (const unsigned char*)&accel_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}


ICM_20948_Status_e ICM20948Component::getBiasAccelX( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, ACCEL_BIAS_X, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasAccelY( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, ACCEL_BIAS_Y, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasAccelZ( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, ACCEL_BIAS_Z, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
//CPass Bias
ICM_20948_Status_e ICM20948Component::setBiasCPassX( int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char cpass_bias_reg[4];
    cpass_bias_reg[0] = (unsigned char)(newValue >> 24);
    cpass_bias_reg[1] = (unsigned char)(newValue >> 16);
    cpass_bias_reg[2] = (unsigned char)(newValue >> 8);
    cpass_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, CPASS_BIAS_X, 4, (const unsigned char*)&cpass_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setBiasCPassY( int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char cpass_bias_reg[4];
    cpass_bias_reg[0] = (unsigned char)(newValue >> 24);
    cpass_bias_reg[1] = (unsigned char)(newValue >> 16);
    cpass_bias_reg[2] = (unsigned char)(newValue >> 8);
    cpass_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, CPASS_BIAS_Y, 4, (const unsigned char*)&cpass_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setBiasCPassZ( int32_t newValue)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char cpass_bias_reg[4];
    cpass_bias_reg[0] = (unsigned char)(newValue >> 24);
    cpass_bias_reg[1] = (unsigned char)(newValue >> 16);
    cpass_bias_reg[2] = (unsigned char)(newValue >> 8);
    cpass_bias_reg[3] = (unsigned char)(newValue & 0xff);
    status_ = inv_icm20948_write_mems(&device_, CPASS_BIAS_Z, 4, (const unsigned char*)&cpass_bias_reg);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasCPassX( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, CPASS_BIAS_X, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasCPassY( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, CPASS_BIAS_Y, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::getBiasCPassZ( int32_t* bias)
{
  if (device_._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    status_ = inv_icm20948_read_mems(&device_, CPASS_BIAS_Z, 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

std::string ICM20948Component::statusToString_(ICM_20948_Status_e stat)
{
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
ICM_20948_Status_e ICM20948Component::setBank(uint8_t bank)
{
  status_ = ICM_20948_set_bank(&device_, bank);
  return status_;
}

ICM_20948_Status_e ICM20948Component::swReset(void)
{
  status_ = ICM_20948_sw_reset(&device_);
  return status_;
}

ICM_20948_Status_e ICM20948Component::sleep(bool on)
{
  status_ = ICM_20948_sleep(&device_, on);
  return status_;
}

ICM_20948_Status_e ICM20948Component::lowPower(bool on)
{
  status_ = ICM_20948_low_power(&device_, on);
  return status_;
}

ICM_20948_Status_e ICM20948Component::setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
  status_ = ICM_20948_set_clock_source(&device_, source);
  return status_;
}

ICM_20948_Status_e ICM20948Component::checkID(void)
{
  status_ = ICM_20948_check_id(&device_);
  if (status_ != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "checkID: ICM_20948_check_id returned: ");
    ESP_LOGD(TAG, statusToString_(status_).c_str());
  }
  return status_;
}

// Internal Sensor Options
ICM_20948_Status_e ICM20948Component::setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode)
{
  status_ = ICM_20948_set_sample_mode(&device_, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode);
  vTaskDelay(pdMS_TO_TICKS(1)); // Give the ICM20948 time to change the sample mode (see issue #8)
  return status_;
}

ICM_20948_Status_e ICM20948Component::setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss)
{
  status_ = ICM_20948_set_full_scale(&device_, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss);
  return status_;
}

ICM_20948_Status_e ICM20948Component::setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg)
{
  status_ = ICM_20948_set_dlpf_cfg(&device_, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg);
  return status_;
}

ICM_20948_Status_e ICM20948Component::enableDLPF(uint8_t sensor_id_bm, bool enable)
{
  status_ = ICM_20948_enable_dlpf(&device_, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable);
  return status_;
}

ICM_20948_Status_e ICM20948Component::setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt)
{
  status_ = ICM_20948_set_sample_rate(&device_, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt);
  return status_;
}

ICM_20948_Status_e ICM20948Component::intEnableDMP(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  status_ = ICM_20948_int_enable(&device_, NULL, &en); // read phase
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  en.DMP_INT1_EN = enable;                           // change the setting
  status_ = ICM_20948_int_enable(&device_, &en, &en); // write phase w/ readback
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

ICM_20948_Status_e ICM20948Component::intEnableRawDataReady(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  status_ = ICM_20948_int_enable(&device_, NULL, &en); // read phase
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  en.RAW_DATA_0_RDY_EN = enable;                     // change the setting
  status_ = ICM_20948_int_enable(&device_, &en, &en); // write phase w/ readback
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

ICM_20948_Status_e ICM20948Component::intEnableOverflowFIFO(uint8_t bm_enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  status_ = ICM_20948_int_enable(&device_, NULL, &en); // read phase
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
  en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
  en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
  en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
  en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
  status_ = ICM_20948_int_enable(&device_, &en, &en); // write phase w/ readback
  if (status_ != ICM_20948_Stat_Ok)
  {
    return status_;
  }
  return status_;
}

// Interface Options
ICM_20948_Status_e ICM20948Component::i2cMasterPassthrough(bool passthrough)
{
  status_ = ICM_20948_i2c_master_passthrough(&device_, passthrough);
  return status_;
}

ICM_20948_Status_e ICM20948Component::i2cMasterEnable(bool enable)
{
  status_ = ICM_20948_i2c_master_enable(&device_, enable);
  return status_;
}

ICM_20948_Status_e ICM20948Component::i2cMasterReset()
{
  status_ = ICM_20948_i2c_master_reset(&device_);
  return status_;
}

ICM_20948_Status_e ICM20948Component::i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  status_ = ICM_20948_i2c_controller_configure_peripheral(&device_, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, dataOut);
  return status_;
}

ICM_20948_Status_e ICM20948Component::i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  status_ = ICM_20948_i2c_controller_periph4_txn(&device_, addr, reg, data, len, Rw, send_reg_addr);
  return status_;
}

ICM_20948_Status_e ICM20948Component::i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data)
{
  status_ = ICM_20948_i2c_master_single_w(&device_, addr, reg, &data);
  return status_;
}
uint8_t ICM20948Component::i2cMasterSingleR(uint8_t addr, uint8_t reg)
{
  uint8_t data = 0;
  status_ = ICM_20948_i2c_master_single_r(&device_, addr, reg, &data);
  if (status_ != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "i2cMasterSingleR: ICM_20948_i2c_master_single_r returned: %s", statusToString_(status_).c_str());
  }
  return data;
}

ICM_20948_Status_e ICM20948Component::startupDefault(bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  retval = checkID();
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: checkID returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = swReset();
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: swReset returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  retval = sleep(false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: sleep returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = lowPower(false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: lowPower returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = startupMagnetometer(minimal); // Pass the minimal startup flag to startupMagnetometer
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: startupMagnetometer returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  if (minimal) // Return now if minimal is true
  {
    ESP_LOGD(TAG, "startupDefault: minimal startup complete!");
    return status_;
  }

  retval = setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: setSampleMode returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  } // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

  ICM_20948_fss_t FSS;
  FSS.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  retval = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: setFullScale returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d473bw_n499bw;
  dlpcfg.g = gyr_d361bw4_n376bw5;
  retval = setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: setDLPFcfg returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = enableDLPF(ICM_20948_Internal_Acc, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: enableDLPF (Acc) returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = enableDLPF(ICM_20948_Internal_Gyr, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupDefault: enableDLPF (Gyr) returned: %s", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  return status_;
}

// direct read/write
ICM_20948_Status_e ICM20948Component::read(uint8_t reg, uint8_t *pdata, uint32_t len)
{
  status_ = ICM_20948_execute_r(&device_, reg, pdata, len);
  return (status_);
}

ICM_20948_Status_e ICM20948Component::write(uint8_t reg, uint8_t *pdata, uint32_t len)
{
  status_ = ICM_20948_execute_w(&device_, reg, pdata, len);
  return (status_);
}

uint8_t ICM20948Component::readMag(AK09916_Reg_Addr_e reg)
{
  uint8_t data = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, reg); // i2cMasterSingleR updates status too
  return data;
}

ICM_20948_Status_e ICM20948Component::writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
  status_ = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, reg, *pdata);
  return status_;
}

ICM_20948_Status_e ICM20948Component::resetMag()
{
  uint8_t SRST = 1;
  // SRST: Soft reset
  // “0”: Normal
  // “1”: Reset
  // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
  status_ = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, SRST);
  return status_;
}

// FIFO

ICM_20948_Status_e ICM20948Component::enableFIFO(bool enable)
{
  status_ = ICM_20948_enable_FIFO(&device_, enable);
  return status_;
}

ICM_20948_Status_e ICM20948Component::resetFIFO(void)
{
  status_ = ICM_20948_reset_FIFO(&device_);
  return status_;
}

ICM_20948_Status_e ICM20948Component::setFIFOmode(bool snapshot)
{
  // Default to Stream (non-Snapshot) mode
  status_ = ICM_20948_set_FIFO_mode(&device_, snapshot);
  return status_;
}

ICM_20948_Status_e ICM20948Component::getFIFOcount(uint16_t *count)
{
  status_ = ICM_20948_get_FIFO_count(&device_, count);
  return status_;
}

ICM_20948_Status_e ICM20948Component::readFIFO(uint8_t *data, uint8_t len)
{
  status_ = ICM_20948_read_FIFO(&device_, data, len);
  return status_;
}

// DMP

ICM_20948_Status_e ICM20948Component::enableDMP(bool enable)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to enable the DMP?
  {
    status_ = ICM_20948_enable_DMP(&device_, enable == true ? 1 : 0);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::resetDMP(void)
{
  status_ = ICM_20948_reset_DMP(&device_);
  return status_;
}

ICM_20948_Status_e ICM20948Component::loadDMPFirmware(void)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to load the DMP firmware?
  {
    status_ = ICM_20948_firmware_load(&device_);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setDMPstartAddress(unsigned short address)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to set the start address?
  {
    status_ = ICM_20948_set_dmp_start_address(&device_, address);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to enable the sensor?
  {
    status_ = inv_icm20948_enable_dmp_sensor(&device_, sensor, enable == true ? 1 : 0);
    ESP_LOGD(TAG, "enableDMPSensor:  _enabled_Android_0: %d", (int)device_._enabled_Android_0);
    ESP_LOGD(TAG, "  _enabled_Android_1: %d", (int)device_._enabled_Android_1);
    ESP_LOGD(TAG, "  _dataOutCtl1: %d", (int)device_._dataOutCtl1);
    ESP_LOGD(TAG, "  _dataOutCtl2: %d", (int)device_._dataOutCtl2);
    ESP_LOGD(TAG, "  _dataRdyStatus: %d", (int)device_._dataRdyStatus);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::enableDMPSensorInt(enum inv_icm20948_sensor sensor, bool enable)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to enable the sensor interrupt?
  {
    status_ = inv_icm20948_enable_dmp_sensor_int(&device_, sensor, enable == true ? 1 : 0);
    ESP_LOGD(TAG, "enableDMPSensorInt:  _enabled_Android_intr_0: %d", (int)device_._enabled_Android_intr_0);
    ESP_LOGD(TAG, "  _enabled_Android_intr_1: %d", (int)device_._enabled_Android_intr_1);
    ESP_LOGD(TAG, "  _dataIntrCtl: %d", (int)device_._dataIntrCtl);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::writeDMPmems(unsigned short reg, unsigned int length, const unsigned char *data)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to write to the DMP?
  {
    status_ = inv_icm20948_write_mems(&device_, reg, length, data);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::readDMPmems(unsigned short reg, unsigned int length, unsigned char *data)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to read from the DMP?
  {
    status_ = inv_icm20948_read_mems(&device_, reg, length, data);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setDMPODRrate(enum DMP_ODR_Registers odr_reg, int interval)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to set the DMP ODR?
  {
    // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate (225Hz) / ODR ) - 1
    // E.g. For a 25Hz ODR rate, value= (225/25) - 1 = 8.

    status_ = inv_icm20948_set_dmp_sensor_period(&device_, odr_reg, interval);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::readDMPdataFromFIFO(icm_20948_DMP_data_t *data)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to set the data from the FIFO?
  {
    status_ = inv_icm20948_read_dmp_data(&device_, data);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM20948Component::setGyroSF(unsigned char div, int gyro_level)
{
  if (device_._dmp_firmware_available == true) // Should we attempt to set the Gyro SF?
  {
    status_ = inv_icm20948_set_gyro_sf(&device_, div, gyro_level);
    ESP_LOGD(TAG, "setGyroSF:  pll: %d", (int)device_._gyroSFpll);
    ESP_LOGD(TAG, "Gyro SF is: %d", (int)device_._gyroSF);
    return status_;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

// Combine all of the DMP start-up code from the earlier DMP examples
// This function is defined as __attribute__((weak)) so you can overwrite it if you want to,
//   e.g. to modify the sample rate
ICM_20948_Status_e ICM20948Component::initializeDMP(void)
{
  // First, let's check if the DMP is available
  if (device_._dmp_firmware_available != true)
  {
    ESP_LOGD(TAG, "startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
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
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
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
  result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = setClockSource(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

  // Disable the FIFO
  result = enableFIFO(false); if (result > worstResult) worstResult = result;

  // Disable the DMP
  result = enableDMP(false); if (result > worstResult) worstResult = result;

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
  result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = resetFIFO(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.g = 4; // 225Hz
  //mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware(); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

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
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(19, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

  return worstResult;
}

ICM_20948_Status_e ICM20948Component::startupMagnetometer(bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  i2cMasterPassthrough(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  i2cMasterEnable(true);

  resetMag();

  //After a ICM reset the Mag sensor may stop responding over the I2C master
  //Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < MAX_MAGNETOMETER_STARTS)
  {
    tries++;

    //See if we can read the WhoIAm register correctly
    retval = magWhoIAm();
    if (retval == ICM_20948_Stat_Ok)
      break; //WIA matched!

    i2cMasterReset(); //Otherwise, reset the master I2C and try again

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (tries == MAX_MAGNETOMETER_STARTS)
  {
    ESP_LOGD(TAG, "startupMagnetometer: reached MAX_MAGNETOMETER_STARTS (%d). Returning ICM_20948_Stat_WrongID", (int)MAX_MAGNETOMETER_STARTS);
    status_ = ICM_20948_Stat_WrongID;
    return status_;
  }
  else
  {
    ESP_LOGD(TAG, "startupMagnetometer: successful magWhoIAm after %d trie(s).", (int)tries);
  }

  //Return now if minimal is true. The mag will be configured manually for the DMP
  if (minimal) // Return now if minimal is true
  {
    ESP_LOGD(TAG, "startupMagnetometer: minimal startup complete!");
    return status_;
  }

  //Set up magnetometer
  AK09916_CNTL2_Reg_t reg;
  reg.MODE = AK09916_mode_cont_100hz;
  reg.reserved_0 = 0; // Make sure the unused bits are clear. Probably redundant, but prevents confusion when looking at the I2C traffic
  retval = writeMag(AK09916_REG_CNTL2, (uint8_t *)&reg);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupMagnetometer: writeMag returned: %d", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  retval = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "startupMagnetometer: i2cMasterConfigurePeripheral returned: %d", statusToString_(retval).c_str());
    status_ = retval;
    return status_;
  }

  return status_;
}

ICM_20948_Status_e ICM20948Component::magWhoIAm(void)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t whoiam1, whoiam2;
  whoiam1 = readMag(AK09916_REG_WIA1);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status_;
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "magWhoIAm: whoiam1: %d", ((int)whoiam1));
    ESP_LOGD(TAG, " (should be 72) readMag set status to: %d", statusToString_(status));
    return retval;
  }
  whoiam2 = readMag(AK09916_REG_WIA2);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status_;
  if (retval != ICM_20948_Stat_Ok)
  {
    ESP_LOGD(TAG, "magWhoIAm: whoiam1: %d", (int)whoiam1);
    ESP_LOGD(TAG, " (should be 72) whoiam2: %d", (int)whoiam2);
    ESP_LOGD(TAG, " (should be 9) readMag set status to: %d", statusToString_(status));
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

} // namespace icm20948
}  // namespace esphome