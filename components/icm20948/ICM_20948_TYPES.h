#pragma once

#include <cstdint>

// Define if the DMP will be supported
// Note: you must have 14290/14301 Bytes of program memory available to store the DMP firmware!
//#define ICM_20948_USE_DMP // Uncomment this line to enable DMP support. You can of course use ICM_20948_USE_DMP as a compiler flag too

// There are two versions of the InvenSense DMP firmware for the ICM20948 - with slightly different sizes
#define DMP_CODE_SIZE 14301 /* eMD-SmartMotion-ICM20948-1.1.0-MP */
//#define DMP_CODE_SIZE 14290 /* ICM20948_eMD_nucleo_1.0 */


#define ICM_20948_I2C_ADDR_AD0 0x68 // Or 0x69 when AD0 is high
#define ICM_20948_I2C_ADDR_AD1 0x69 //
#define ICM_20948_WHOAMI 0xEA

#define MAG_AK09916_I2C_ADDR 0x0C
#define MAG_AK09916_WHO_AM_I 0x4809
#define MAG_REG_WHO_AM_I 0x00

/** @brief Max size that can be read across I2C or SPI data lines */
#define INV_MAX_SERIAL_READ 16
/** @brief Max size that can be written across I2C or SPI data lines */
#define INV_MAX_SERIAL_WRITE 16

namespace esphome {
namespace icm20948 {

enum ICM_20948_Status_e
{
    ICM_20948_Stat_Ok = 0x00, // The only return code that means all is well
    ICM_20948_Stat_Err,       // A general error
    ICM_20948_Stat_NotImpl,   // Returned by virtual functions that are not implemented
    ICM_20948_Stat_ParamErr,
    ICM_20948_Stat_WrongID,
    ICM_20948_Stat_InvalSensor, // Tried to apply a function to a sensor that does not support it (e.g. DLPF to the temperature sensor)
    ICM_20948_Stat_NoData,
    ICM_20948_Stat_SensorNotSupported,
    ICM_20948_Stat_DMPNotSupported,    // DMP not supported (no #define ICM_20948_USE_DMP)
    ICM_20948_Stat_DMPVerifyFail,      // DMP was written but did not verify correctly
    ICM_20948_Stat_FIFONoDataAvail,    // FIFO contains no data
    ICM_20948_Stat_FIFOIncompleteData, // FIFO contained incomplete data
    ICM_20948_Stat_FIFOMoreDataAvail,  // FIFO contains more data
    ICM_20948_Stat_UnrecognisedDMPHeader,
    ICM_20948_Stat_UnrecognisedDMPHeader2,
    ICM_20948_Stat_InvalDMPRegister, // Invalid DMP Register

    ICM_20948_Stat_NUM,
    ICM_20948_Stat_Unknown,
};

enum ICM_20948_InternalSensorID_bm
{
    ICM_20948_Internal_Acc = (1 << 0),
    ICM_20948_Internal_Gyr = (1 << 1),
    ICM_20948_Internal_Mag = (1 << 2),
    ICM_20948_Internal_Tmp = (1 << 3),
    ICM_20948_Internal_Mst = (1 << 4), // I2C Master Ineternal
};     // A bitmask of internal sensor IDs

union ICM_20948_axis3bit16_t
{
    int16_t i16bit[3];
    uint8_t u8bit[6];
};

union ICM_20948_axis1bit16_t
{
    int16_t i16bit;
    uint8_t u8bit[2];
};

struct ICM_20948_fss_t
{
    uint8_t a : 2;
    uint8_t g : 2;
    uint8_t reserved_0 : 4;
}; // Holds full-scale settings to be able to extract measurements with units

struct ICM_20948_dlpcfg_t
{
    uint8_t a;
    uint8_t g;
}; // Holds digital low pass filter settings. Members are type ICM_20948_ACCEL_CONFIG_DLPCFG_e

struct ICM_20948_smplrt_t
{
    uint16_t a;
    uint8_t g;
};

struct ICM_20948_INT_enable_t
{
    uint8_t I2C_MST_INT_EN : 1;
    uint8_t DMP_INT1_EN : 1;
    uint8_t PLL_RDY_EN : 1;
    uint8_t WOM_INT_EN : 1;
    uint8_t REG_WOF_EN : 1;
    uint8_t RAW_DATA_0_RDY_EN : 1;
    uint8_t FIFO_OVERFLOW_EN_4 : 1;
    uint8_t FIFO_OVERFLOW_EN_3 : 1;
    uint8_t FIFO_OVERFLOW_EN_2 : 1;
    uint8_t FIFO_OVERFLOW_EN_1 : 1;
    uint8_t FIFO_OVERFLOW_EN_0 : 1;
    uint8_t FIFO_WM_EN_4 : 1;
    uint8_t FIFO_WM_EN_3 : 1;
    uint8_t FIFO_WM_EN_2 : 1;
    uint8_t FIFO_WM_EN_1 : 1;
    uint8_t FIFO_WM_EN_0 : 1;
};

union ICM_20948_axis3named_t
{
    ICM_20948_axis3bit16_t raw;
    struct axes
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
};

struct ICM_20948_AGMT_t
{
    ICM_20948_axis3named_t acc;
    ICM_20948_axis3named_t gyr;
    ICM_20948_axis3named_t mag;
    union tmp
    {
        ICM_20948_axis1bit16_t raw;
        int16_t val;
    } tmp;
    ICM_20948_fss_t fss; // Full-scale range settings for this measurement
    uint8_t magStat1;
    uint8_t magStat2;
};

struct ICM_20948_Device_t
{
    bool _dmp_firmware_available{false};    // Indicates if the DMP firmware has been included. It
    bool _firmware_loaded{false};           // Indicates if DMP has been loaded
    uint8_t _last_bank{4};              // Keep track of which bank was selected last - to avoid unnecessary writes
    uint8_t _last_mems_bank{0};         // Keep track of which bank was selected last - to avoid unnecessary writes
    int32_t _gyroSF{0};                 // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    int8_t _gyroSFpll{0};
    uint32_t _enabled_Android_0{0};      // Keep track of which Android sensors are enabled: 0-31
    uint32_t _enabled_Android_1{0};      // Keep track of which Android sensors are enabled: 32-
    uint32_t _enabled_Android_intr_0{0}; // Keep track of which Android sensor interrupts are enabled: 0-31
    uint32_t _enabled_Android_intr_1{0}; // Keep track of which Android sensor interrupts are enabled: 32-
    uint16_t _dataOutCtl1{0};            // Diagnostics: record the setting of DATA_OUT_CTL1
    uint16_t _dataOutCtl2{0};            // Diagnostics: record the setting of DATA_OUT_CTL2
    uint16_t _dataRdyStatus{0};          // Diagnostics: record the setting of DATA_RDY_STATUS
    uint16_t _motionEventCtl{0};         // Diagnostics: record the setting of MOTION_EVENT_CTL
    uint16_t _dataIntrCtl{0};            // Diagnostics: record the setting of DATA_INTR_CTL
};

} // namespace icm20948
} // namespace esphome