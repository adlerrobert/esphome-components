#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"

#include "ICM_20948_TYPES.h"
#include "ICM_20948_DMP.h"
#include "ICM_20948_REGISTERS.h"
#include "ICM_20948_ENUMERATIONS.h"
#include "AK09916_ENUMERATIONS.h"
#include "AK09916_REGISTERS.h"

#include <vector>
#include <string>

namespace esphome {
namespace icm20948 {

struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
  bool calibrate_on_boot = false; // Whether to calibrate on boot
  bool bias_calibrated = false;   // Whether the biases have been calibrated
};

class ICM20948Component : public Component, public i2c::I2CDevice {
 public:
  // general functions
  void setup() override;
  void dump_config() override;

  void loop() override;
  float get_setup_priority() const override;

  // sensor setterss
  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; } 
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  void set_mag_x_sensor(sensor::Sensor *mag_x_sensor) { mag_x_sensor_ = mag_x_sensor; }
  void set_mag_y_sensor(sensor::Sensor *mag_y_sensor) { mag_y_sensor_ = mag_y_sensor; }
  void set_mag_z_sensor(sensor::Sensor *mag_z_sensor) { mag_z_sensor_ = mag_z_sensor; }
  void set_qw_sensor(sensor::Sensor *qw_sensor) { qw_sensor_ = qw_sensor; }
  void set_qx_sensor(sensor::Sensor *qx_sensor) { qx_sensor_ = qx_sensor; }
  void set_qy_sensor(sensor::Sensor *qy_sensor) { qy_sensor_ = qy_sensor; }
  void set_qz_sensor(sensor::Sensor *qz_sensor) { qz_sensor_ = qz_sensor; }
  void set_roll_sensor(sensor::Sensor *roll_sensor) { roll_sensor_ = roll_sensor; }
  void set_pitch_sensor(sensor::Sensor *pitch_sensor) { pitch_sensor_ = pitch_sensor; }
  void set_yaw_sensor(sensor::Sensor *yaw_sensor) { yaw_sensor_ = yaw_sensor; }

  void set_calibrate_next_boot(bool enabled);

private:
  //Gyro Bias
  ICM_20948_Status_e setBiasGyroX_(int32_t newValue);
  ICM_20948_Status_e setBiasGyroY_(int32_t newValue);
  ICM_20948_Status_e setBiasGyroZ_(int32_t newValue);
  ICM_20948_Status_e getBiasGyroX_(int32_t* bias);
  ICM_20948_Status_e getBiasGyroY_(int32_t* bias);
  ICM_20948_Status_e getBiasGyroZ_(int32_t* bias);
  //Accel Bias
  ICM_20948_Status_e setBiasAccelX_(int32_t newValue);
  ICM_20948_Status_e setBiasAccelY_(int32_t newValue);
  ICM_20948_Status_e setBiasAccelZ_(int32_t newValue);
  ICM_20948_Status_e getBiasAccelX_(int32_t* bias);
  ICM_20948_Status_e getBiasAccelY_(int32_t* bias);
  ICM_20948_Status_e getBiasAccelZ_(int32_t* bias);
  //CPass Bias
  ICM_20948_Status_e setBiasCPassX_(int32_t newValue);
  ICM_20948_Status_e setBiasCPassY_(int32_t newValue);
  ICM_20948_Status_e setBiasCPassZ_(int32_t newValue);
  ICM_20948_Status_e getBiasCPassX_(int32_t* bias);
  ICM_20948_Status_e getBiasCPassY_(int32_t* bias);
  ICM_20948_Status_e getBiasCPassZ_(int32_t* bias);

  std::string statusToString_(ICM_20948_Status_e stat);

  // Device Lev
  ICM_20948_Status_e setBank_(uint8_t bank);                                // Sets the bank
  ICM_20948_Status_e swReset_(void);                                        // Performs a SW reset
  ICM_20948_Status_e sleep_(bool on = false);                               // Set sleep mode for the chip
  ICM_20948_Status_e lowPower_(bool on = true);                             // Set low power mode for the chip
  ICM_20948_Status_e setClockSource_(ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
  ICM_20948_Status_e getWhoAmI_(uint8_t *whoami);
  ICM_20948_Status_e checkID_(void);                                        // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

  // Internal Sensor Options
  ICM_20948_Status_e setSampleMode_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode);
  ICM_20948_Status_e setFullScale_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss);
  ICM_20948_Status_e setDLPFcfg_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_dlpcfg_t cfg);
  ICM_20948_Status_e enableDLPF_(ICM_20948_InternalSensorID_bm sensors, bool enable);
  ICM_20948_Status_e setSampleRate_(ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt);

  ICM_20948_Status_e intEnableDMP_(bool enable);
  ICM_20948_Status_e intEnableRawDataReady_(bool enable);
  ICM_20948_Status_e intEnableOverflowFIFO_(uint8_t bm_enable);

  // Interface Options
  ICM_20948_Status_e i2cMasterPassthrough_(bool passthrough = true);
  ICM_20948_Status_e i2cMasterEnable_(bool enable = true);
  ICM_20948_Status_e i2cMasterReset_();

  //Used for configuring peripherals 0-3
  ICM_20948_Status_e i2cControllerConfigurePeripheral_(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false, uint8_t dataOut = 0);
  ICM_20948_Status_e i2cControllerPeriph4Transaction_(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

  //Used for configuring the Magnetometer
  ICM_20948_Status_e i2cMasterSingleW_(uint8_t addr, uint8_t reg, uint8_t data);
  uint8_t i2cMasterSingleR_(uint8_t addr, uint8_t reg);

  // Default Setup
  ICM_20948_Status_e startupDefault_(bool minimal = false); // If minimal is true, several startup steps are skipped. If ICM_20948_USE_DMP is defined, .begin will call startupDefault with minimal set to true.

  //Mag specific
  ICM_20948_Status_e startupMagnetometer_(bool minimal = false); // If minimal is true, several startup steps are skipped. The mag then needs to be set up manually for the DMP.
  ICM_20948_Status_e magWhoIAm_(void);
  uint8_t readMag_(AK09916_Reg_Addr_e reg);
  ICM_20948_Status_e writeMag_(AK09916_Reg_Addr_e reg, uint8_t *pdata);
  ICM_20948_Status_e resetMag_();

  //FIFO
  ICM_20948_Status_e enableFIFO_(bool enable = true);
  ICM_20948_Status_e resetFIFO_(void);
  ICM_20948_Status_e setFIFOmode_(bool snapshot = false); // Default to Stream (non-Snapshot) mode
  ICM_20948_Status_e getFIFOcount_(uint16_t *count);
  ICM_20948_Status_e readFIFO_(uint8_t *data, uint8_t len = 1);

  ICM_20948_Status_e enableDMP_(bool enable = true);
  ICM_20948_Status_e resetDMP_(void);
  ICM_20948_Status_e loadDMPFirmware_(void);
  ICM_20948_Status_e setDMPstartAddress_(unsigned short address = DMP_START_ADDRESS);
  ICM_20948_Status_e enableDMPSensor_(inv_icm20948_sensor sensor, bool state = true);
  ICM_20948_Status_e enableDMPSensorInt_(inv_icm20948_sensor sensor, bool state = true);
  ICM_20948_Status_e writeDMPmems_(unsigned short reg, unsigned int length, const unsigned char *data);
  ICM_20948_Status_e readDMPmems_(unsigned short reg, unsigned int length, unsigned char *data);
  ICM_20948_Status_e setDMPODRrate_(DMP_ODR_Registers odr_reg, int interval);
  ICM_20948_Status_e readDMPdataFromFIFO_(icm_20948_DMP_data_t *data);
  ICM_20948_Status_e setGyroSF_(unsigned char div, int gyro_level);
  ICM_20948_Status_e initializeDMP_(void);

  void updateBiasStoreSum_(biasStore *store);
  bool isBiasStoreValid_(biasStore *store);
  void printBiases_(biasStore *store);

  ICM_20948_Status_e write_i2c(uint8_t reg, uint8_t *data, uint32_t len);
  ICM_20948_Status_e read_i2c(uint8_t reg, uint8_t *buff, uint32_t len);

  ICM_20948_Status_e inv_icm20948_read_mems_(unsigned short reg, unsigned int length, unsigned char *data);
  ICM_20948_Status_e inv_icm20948_write_mems_(unsigned short reg, unsigned int length, const unsigned char *data);

  ICM_20948_Status_e intEnable_(ICM_20948_INT_enable_t *write, ICM_20948_INT_enable_t *read);

  uint8_t sensor_type_2_android_sensor(inv_icm20948_sensor sensor);
  inv_icm20948_sensor inv_icm20948_sensor_android_2_sensor_type(int sensor);

  ESPPreferenceObject bias_store_pref_;
  biasStore bias_store_;

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
  sensor::Sensor *qw_sensor_{nullptr};
  sensor::Sensor *qx_sensor_{nullptr};
  sensor::Sensor *qy_sensor_{nullptr};
  sensor::Sensor *qz_sensor_{nullptr};
  sensor::Sensor *roll_sensor_{nullptr};
  sensor::Sensor *pitch_sensor_{nullptr};
  sensor::Sensor *yaw_sensor_{nullptr};

  ICM_20948_Device_t device_;
  ICM_20948_Status_e status_;

  bool initialized_{false};
  std::vector<std::string> steps_;
};

template<typename... Ts> class SetCalibrateNextBootAction : public Action<Ts...> {
public:
  explicit SetCalibrateNextBootAction(ICM20948Component *parent) : parent(parent) {};

  TEMPLATABLE_VALUE(bool, calibrate_next_boot);

  void play(Ts... x) override {
    auto enabled = this->calibrate_next_boot_.value(x...);
    this->parent->set_calibrate_next_boot(enabled);
  }

protected:
  ICM20948Component *parent{nullptr};
};

}  // namespace icm20948
}  // namespace esphome