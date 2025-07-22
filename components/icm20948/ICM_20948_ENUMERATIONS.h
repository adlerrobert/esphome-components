#pragma once

namespace esphome {
namespace icm20948 {

enum ICM_20948_LP_CONFIG_CYCLE_e
{
  ICM_20948_Sample_Mode_Continuous = 0x00,
  ICM_20948_Sample_Mode_Cycled,
};

// AGB0_REG_PWR_MGMT_1,

enum ICM_20948_PWR_MGMT_1_CLKSEL_e
{
  ICM_20948_Clock_Internal_20MHz = 0x00,
  ICM_20948_Clock_Auto,
  ICM_20948_Clock_TimingReset = 0x07
};

enum ICM_20948_GYRO_CONFIG_1_FS_SEL_e
{ // Full scale range options in degrees per second
  dps250 = 0x00,
  dps500,
  dps1000,
  dps2000,
};

enum ICM_20948_GYRO_CONFIG_1_DLPCFG_e
{ // Format is dAbwB_nXbwY - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
  gyr_d196bw6_n229bw8 = 0x00,
  gyr_d151bw8_n187bw6,
  gyr_d119bw5_n154bw3,
  gyr_d51bw2_n73bw3,
  gyr_d23bw9_n35bw9,
  gyr_d11bw6_n17bw8,
  gyr_d5bw7_n8bw9,
  gyr_d361bw4_n376bw5,
};

enum ICM_20948_ACCEL_CONFIG_FS_SEL_e
{
  gpm2 = 0x00,
  gpm4,
  gpm8,
  gpm16,
};

enum ICM_20948_ACCEL_CONFIG_DLPCFG_e
{ // Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
  acc_d246bw_n265bw = 0x00,
  acc_d246bw_n265bw_1,
  acc_d111bw4_n136bw,
  acc_d50bw4_n68bw8,
  acc_d23bw9_n34bw4,
  acc_d11bw5_n17bw,
  acc_d5bw7_n8bw3,
  acc_d473bw_n499bw,
};


} // namespace icm20948
} // namespace esphome