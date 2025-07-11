import esphome.codegen as cg
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_OFFSET,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BRIEFCASE_DOWNLOAD,
    ICON_MAGNET,
    ICON_SCREEN_ROTATION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_METER_PER_SECOND_SQUARED,
    UNIT_MICROTESLA,
)

CODEOWNERS = ["@adlerrobert"]
DEPENDENCIES = ["i2c"]

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"
CONF_MAG_X = "mag_x"
CONF_MAG_Y = "mag_y"
CONF_MAG_Z = "mag_z"
CONF_TEMP = "temp"
CONF_MAPPING_VECTOR = "mapping_vector"


mpu9255_ns = cg.esphome_ns.namespace("mpu9255")

MPU9255AccelerometerScales = mpu9255_ns.enum("MPU9255AccelerometerScales")
SCALE_ACC_OPTIONS = {
    "2g": MPU9255AccelerometerScales.MPU9255AccelerometerScales_2G,
    "4g": MPU9255AccelerometerScales.MPU9255AccelerometerScales_4G,
    "8g": MPU9255AccelerometerScales.MPU9255AccelerometerScales_8G,
    "16g": MPU9255AccelerometerScales.MPU9255AccelerometerScales_16G,
}

MPU9255GyroscopeScales = mpu9255_ns.enum("MPU9255GyroscopeScales")
SCALE_GYR_OPTIONS = {
    "250dps": MPU9255GyroscopeScales.MPU9255GyroscopeScales_250DPS,
    "500dps": MPU9255GyroscopeScales.MPU9255GyroscopeScales_500DPS,
    "1000dps": MPU9255GyroscopeScales.MPU9255GyroscopeScales_1000DPS,
    "2000dps": MPU9255GyroscopeScales.MPU9255GyroscopeScales_2000DPS,
}

MPU9255AccelerometerBandwidths = mpu9255_ns.enum("MPU9255AccelerometerBandwidths")
BANDWIDTH_ACC_OPTIONS = {
    "5Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_5Hz,
    "10Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_10Hz,
    "20Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_20Hz,
    "41Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_41Hz,
    "92Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_92Hz,
    "184Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_184Hz,
    "460Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_460Hz,
    "1113Hz": MPU9255AccelerometerBandwidths.MPU9255AccelerometerBandwidths_1113Hz,
}

MPU9255GyroscopeBandwidths = mpu9255_ns.enum("MPU9255GyroscopeBandwidths")
BANDWIDTH_GYR_OPTIONS = {
    "5Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_5Hz,
    "10Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_10Hz,
    "20Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_20Hz,
    "41Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_41Hz,
    "92Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_92Hz,
    "184Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_184Hz,
    "250Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_250Hz,
    "3600Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_3600Hz,
    "8800Hz": MPU9255GyroscopeBandwidths.MPU9255GyroscopeBandwidths_8800Hz,
}

MPU9255Component = mpu9255_ns.class_(
    "MPU9255Component", cg.PollingComponent, i2c.I2CDevice
)

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {
         cv.Optional(CONF_OFFSET): cv.float_,
    }
)
gyro_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {
        cv.Optional(CONF_OFFSET): cv.float_,
    }
)
mag_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_MICROTESLA,
    icon=ICON_MAGNET,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {
        cv.Optional(CONF_OFFSET): cv.float_,
        cv.Optional(CONF_MAPPING_VECTOR): cv.All(cv.ensure_list, [cv.float_], length=3),
    }
)
temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPU9255Component),
            cv.Optional(CONF_ACCEL_X): accel_schema,
            cv.Optional(CONF_ACCEL_Y): accel_schema,
            cv.Optional(CONF_ACCEL_Z): accel_schema,
            cv.Optional(CONF_GYRO_X): gyro_schema,
            cv.Optional(CONF_GYRO_Y): gyro_schema,
            cv.Optional(CONF_GYRO_Z): gyro_schema,
            cv.Optional(CONF_MAG_X): gyro_schema,
            cv.Optional(CONF_MAG_Y): gyro_schema,
            cv.Optional(CONF_MAG_Z): gyro_schema,
            cv.Optional(CONF_TEMP): temperature_schema,
            cv.Optional(CONF_ACCEL_SCALE): cv.Optional("Scale", default="2g"): cv.enum(
                SCALE_ACC_OPTIONS, upper=True
            ),
            cv.Optional(CONF_GYRO_SCALE): cv.Optional("Scale", default="250dps"): cv.enum(
                SCALE_GYR_OPTIONS, upper=True
            ),
            cv.Optional(CONF_ACCEL_BANDWIDTH): cv.Optional("Bandwidth", default="184Hz"): cv.enum(
                BANDWIDTH_ACC_OPTIONS, upper=True
            ),
            cv.Optional(CONF_GYRO_BANDWIDTH): cv.Optional("Bandwidth", default="184Hz"): cv.enum(
                BANDWIDTH_GYR_OPTIONS, upper=True),
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(i2c.i2c_device_schema(0x68))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for d in ["x", "y", "z"]:
        accel_key = f"accel_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))
            if CONF_OFFSET in config[accel_key]:
                offset = config[accel_key][CONF_OFFSET]
                cg.add(getattr(var, f"set_accel_offset_{d}")(offset))
        accel_key = f"gyro_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f"set_gyro_{d}_sensor")(sens))
            if CONF_OFFSET in config[accel_key]:
                offset = config[accel_key][CONF_OFFSET]
                cg.add(getattr(var, f"set_gyro_offset_{d}")(offset))
        accel_key = f"mag_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f"set_mag_{d}_sensor")(sens))
            if CONF_OFFSET in config[accel_key]:
                offset = config[accel_key][CONF_OFFSET]
                cg.add(getattr(var, f"set_mag_offset_{d}")(offset))
            if CONF_MAPPING_VECTOR in config[accel_key]:
                mapping_vector = config[accel_key][CONF_MAPPING_VECTOR]
                if len(mapping_vector) != 3:
                    raise cv.Invalid(
                        f"Mapping vector for {accel_key} must be a list of 3 floats."
                    )
                cg.add(getattr(var, f"set_mag_calibration_matrix_{d}")(mapping_vector[0], mapping_vector[1], mapping_vector[2]))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_ACCEL_SCALE in config:
        scale = config[CONF_ACCEL_SCALE]
        cg.add(var.set_accel_scale(MPU9255AccelerometerScales[scale.upper()]))
    if CONF_GYRO_SCALE in config:
        scale = config[CONF_GYRO_SCALE]
        cg.add(var.set_gyro_scale(MPU9255GyroscopeScales[scale.upper()])) 
    if CONF_ACCEL_BANDWIDTH in config:
        bandwidth = config[CONF_ACCEL_BANDWIDTH]
        cg.add(var.set_accel_bandwidth(MPU9255AccelerometerBandwidths[bandwidth.upper()]))
    if CONF_GYRO_BANDWIDTH in config:
        bandwidth = config[CONF_GYRO_BANDWIDTH]
        cg.add(var.set_gyro_bandwidth(MPU9255GyroscopeBandwidths[bandwidth.upper()]))