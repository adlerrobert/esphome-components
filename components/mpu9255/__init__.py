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
AUTO_LOAD = ["sensor"]

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
CONF_ACCEL_SCALE = "accel_scale"
CONF_GYRO_SCALE = "gyro_scale"
CONF_ACCEL_BANDWIDTH = "accel_bandwidth"
CONF_GYRO_BANDWIDTH = "gyro_bandwidth"

mpu9255_ns = cg.esphome_ns.namespace("mpu9255")

MS_ACC = mpu9255_ns.enum("MS_ACC")
SCALE_ACC_OPTIONS = {
    "2G": MS_ACC.MS_ACC_2G,
    "4G": MS_ACC.MS_ACC_4G,
    "8G": MS_ACC.MS_ACC_8G,
    "16G": MS_ACC.MS_ACC_16G,
}

MS_GYRO = mpu9255_ns.enum("MS_GYRO")
SCALE_GYR_OPTIONS = {
    "250DPS": MS_GYRO.MS_GYRO_250DPS,
    "500DPS": MS_GYRO.MS_GYRO_500DPS,
    "1000DPS": MS_GYRO.MS_GYRO_1000DPS,
    "2000DPS": MS_GYRO.MS_GYRO_2000DPS,
}

MB_ACC = mpu9255_ns.enum("MB_ACC")
BANDWIDTH_ACC_OPTIONS = {
    "5HZ": MB_ACC.MB_ACCS_5HZ,
    "10HZ": MB_ACC.MB_ACC_10HZ,
    "20HZ": MB_ACC.MB_ACC_20HZ,
    "41HZ": MB_ACC.MB_ACC_41HZ,
    "92HZ": MB_ACC.MB_ACC_92HZ,
    "184HZ": MB_ACC.MB_ACC_184HZ,
    "460HZ": MB_ACC.MB_ACC_460HZ,
    "1KHZ": MB_ACC.MB_ACC_1KHZ,
}

MB_GYRO = mpu9255_ns.enum("MB_GYRO")
BANDWIDTH_GYR_OPTIONS = {
    "5HZ": MB_GYRO.MB_GYRO_5HZ,
    "10HZ": MB_GYRO.MB_GYRO_10HZ,
    "20HZ": MB_GYRO.MB_GYRO_20HZ,
    "41HZ": MB_GYRO.MB_GYRO_41HZ,
    "92HZ": MB_GYRO.MB_GYRO_92HZ,
    "184HZ": MB_GYRO.MB_GYRO_184HZ,
    "250HZ": MB_GYRO.MB_GYRO_250HZ,
    "3KHZ": MB_GYRO.MB_GYRO_3KHZ,
    "8KHZ": MB_GYRO.MB_GYRO_8KHZ,
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
            cv.Optional(CONF_ACCEL_SCALE, default="2G"): cv.enum(
                SCALE_ACC_OPTIONS, upper=True
            ),
            cv.Optional(CONF_GYRO_SCALE, default="250DPS"): cv.enum(
                SCALE_GYR_OPTIONS, upper=True
            ),
            cv.Optional(CONF_ACCEL_BANDWIDTH, default="41HZ"): cv.enum(
                BANDWIDTH_ACC_OPTIONS, upper=True
            ),
            cv.Optional(CONF_GYRO_BANDWIDTH, default="41HZ"): cv.enum(
                BANDWIDTH_GYR_OPTIONS, upper=True
            ),
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
        cg.add(var.set_accel_scale(config[CONF_ACCEL_SCALE]))
    if CONF_GYRO_SCALE in config:
        scale = MS_GYRO[config[CONF_GYRO_SCALE]]
        cg.add(var.set_gyro_scale(config[CONF_GYRO_SCALE]))
    if CONF_ACCEL_BANDWIDTH in config:
        cg.add(var.set_accel_bandwidth(config[CONF_ACCEL_BANDWIDTH]))
    if CONF_GYRO_BANDWIDTH in config:
        cg.add(var.set_gyro_bandwidth(config[CONF_GYRO_BANDWIDTH]))