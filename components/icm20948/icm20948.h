#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"

#define ICM_20948_USE_DMP

#include "SparkFun_ICM-20948_ArduinoLibrary/src/util/ICM_20948_C.h"
#include "SparkFun_ICM-20948_ArduinoLibrary/src/util/AK09916_REGISTERS.h"

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

  static ICM_20948_Status_e write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
  static ICM_20948_Status_e read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

private:
  //Gyro Bias
  ICM_20948_Status_e setBiasGyroX(int32_t newValue);
  ICM_20948_Status_e setBiasGyroY(int32_t newValue);
  ICM_20948_Status_e setBiasGyroZ(int32_t newValue);
  ICM_20948_Status_e getBiasGyroX(int32_t* bias);
  ICM_20948_Status_e getBiasGyroY(int32_t* bias);
  ICM_20948_Status_e getBiasGyroZ(int32_t* bias);
  //Accel Bias
  ICM_20948_Status_e setBiasAccelX(int32_t newValue);
  ICM_20948_Status_e setBiasAccelY(int32_t newValue);
  ICM_20948_Status_e setBiasAccelZ(int32_t newValue);
  ICM_20948_Status_e getBiasAccelX(int32_t* bias);
  ICM_20948_Status_e getBiasAccelY(int32_t* bias);
  ICM_20948_Status_e getBiasAccelZ(int32_t* bias);
  //CPass Bias
  ICM_20948_Status_e setBiasCPassX(int32_t newValue);
  ICM_20948_Status_e setBiasCPassY(int32_t newValue);
  ICM_20948_Status_e setBiasCPassZ(int32_t newValue);
  ICM_20948_Status_e getBiasCPassX(int32_t* bias);
  ICM_20948_Status_e getBiasCPassY(int32_t* bias);
  ICM_20948_Status_e getBiasCPassZ(int32_t* bias);

  std::string statusToString_(ICM_20948_Status_e stat);

  // Device Level
  ICM_20948_Status_e setBank(uint8_t bank);                                // Sets the bank
  ICM_20948_Status_e swReset(void);                                        // Performs a SW reset
  ICM_20948_Status_e sleep(bool on = false);                               // Set sleep mode for the chip
  ICM_20948_Status_e lowPower(bool on = true);                             // Set low power mode for the chip
  ICM_20948_Status_e setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
  ICM_20948_Status_e checkID(void);                                        // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

  // Internal Sensor Options
  ICM_20948_Status_e setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
  ICM_20948_Status_e setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss);
  ICM_20948_Status_e setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg);
  ICM_20948_Status_e enableDLPF(uint8_t sensor_id_bm, bool enable);
  ICM_20948_Status_e setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt);

  ICM_20948_Status_e intEnableDMP(bool enable);
  ICM_20948_Status_e intEnableRawDataReady(bool enable);
  ICM_20948_Status_e intEnableOverflowFIFO(uint8_t bm_enable);

  // Interface Options
  ICM_20948_Status_e i2cMasterPassthrough(bool passthrough = true);
  ICM_20948_Status_e i2cMasterEnable(bool enable = true);
  ICM_20948_Status_e i2cMasterReset();

  //Used for configuring peripherals 0-3
  ICM_20948_Status_e i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false, uint8_t dataOut = 0);
  ICM_20948_Status_e i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

  //Used for configuring the Magnetometer
  ICM_20948_Status_e i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data);
  uint8_t i2cMasterSingleR(uint8_t addr, uint8_t reg);

  // Default Setup
  ICM_20948_Status_e startupDefault(bool minimal = false); // If minimal is true, several startup steps are skipped. If ICM_20948_USE_DMP is defined, .begin will call startupDefault with minimal set to true.

  // direct read/write
  ICM_20948_Status_e read(uint8_t reg, uint8_t *pdata, uint32_t len);
  ICM_20948_Status_e write(uint8_t reg, uint8_t *pdata, uint32_t len);

  //Mag specific
  ICM_20948_Status_e startupMagnetometer(bool minimal = false); // If minimal is true, several startup steps are skipped. The mag then needs to be set up manually for the DMP.
  ICM_20948_Status_e magWhoIAm(void);
  uint8_t readMag(AK09916_Reg_Addr_e reg);
  ICM_20948_Status_e writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata);
  ICM_20948_Status_e resetMag();

  //FIFO
  ICM_20948_Status_e enableFIFO(bool enable = true);
  ICM_20948_Status_e resetFIFO(void);
  ICM_20948_Status_e setFIFOmode(bool snapshot = false); // Default to Stream (non-Snapshot) mode
  ICM_20948_Status_e getFIFOcount(uint16_t *count);
  ICM_20948_Status_e readFIFO(uint8_t *data, uint8_t len = 1);

  ICM_20948_Status_e enableDMP(bool enable = true);
  ICM_20948_Status_e resetDMP(void);
  ICM_20948_Status_e loadDMPFirmware(void);
  ICM_20948_Status_e setDMPstartAddress(unsigned short address = DMP_START_ADDRESS);
  ICM_20948_Status_e enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable = true);
  ICM_20948_Status_e enableDMPSensorInt(enum inv_icm20948_sensor sensor, bool enable = true);
  ICM_20948_Status_e writeDMPmems(unsigned short reg, unsigned int length, const unsigned char *data);
  ICM_20948_Status_e readDMPmems(unsigned short reg, unsigned int length, unsigned char *data);
  ICM_20948_Status_e setDMPODRrate(enum DMP_ODR_Registers odr_reg, int interval);
  ICM_20948_Status_e readDMPdataFromFIFO(icm_20948_DMP_data_t *data);
  ICM_20948_Status_e setGyroSF(unsigned char div, int gyro_level);
  ICM_20948_Status_e initializeDMP(void);

  void updateBiasStoreSum_(biasStore *store);
  bool isBiasStoreValid_(biasStore *store);
  void printBiases_(biasStore *store);

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
};

template<typename... Ts> class SetCalibrateNextBootAction : public Action<Ts...> {
public:
  explicit SetCalibrateNextBootAction(ICM20948Component *parent) : parent(parent) {};

  TEMPLATABLE_VALUE(bool, calibrate_next_boot);

  void play(TS... x) override {
    auto enabled = this->calibrate_next_boot_.value(x...);
    this->parent->set_calibrate_next_boot(enabled);
  }

protected:
  ICM20948Component *parent{nullptr};
};

}  // namespace icm20948
}  // namespace esphome