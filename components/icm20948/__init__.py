from esphome import automation
import esphome.codegen as cg
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_OFFSET,
    CONF_STATE,
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
CONF_QUAT_W = "quat_w"
CONF_QUAT_X = "quat_x"
CONF_QUAT_Y = "quat_y"
CONF_QUAT_Z = "quat_z"
CONF_ROLL = "roll"
CONF_PITCH = "pitch"
CONF_YAW = "yaw"

icm20948_ns = cg.esphome_ns.namespace("icm20948")

icm20948Component = icm20948_ns.class_(
    "ICM20948Component", cg.Component, i2c.I2CDevice
)

SetCalibrateNextBootAction = icm20948_ns.class_(
    "SetCalibrateNextBootAction", automation.Action
)

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
gyro_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
mag_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_MICROTESLA,
    icon=ICON_MAGNET,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)
quaternion_schema = sensor.sensor_schema(
    unit_of_measurement="",
    icon="mdi:alpha-q-circle",
    accuracy_decimals=4,
    state_class=STATE_CLASS_MEASUREMENT
)
roll_schema = sensor.sensor_schema(
    unit_of_measurement="°",
    icon="mdi:alpha-r-circle",
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT
)
pitch_schema = sensor.sensor_schema(
    unit_of_measurement="°",
    icon="mdi:alpha-p-circle",
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT
)
yaw_schema = sensor.sensor_schema(
    unit_of_measurement="°",
    icon="mdi:alpha-y-circle",
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(icm20948Component),
            cv.Optional(CONF_ACCEL_X): accel_schema,
            cv.Optional(CONF_ACCEL_Y): accel_schema,
            cv.Optional(CONF_ACCEL_Z): accel_schema,
            cv.Optional(CONF_GYRO_X): gyro_schema,
            cv.Optional(CONF_GYRO_Y): gyro_schema,
            cv.Optional(CONF_GYRO_Z): gyro_schema,
            cv.Optional(CONF_MAG_X): gyro_schema,
            cv.Optional(CONF_MAG_Y): gyro_schema,
            cv.Optional(CONF_MAG_Z): gyro_schema,
            cv.Optional(CONF_QUAT_W): quaternion_schema,
            cv.Optional(CONF_QUAT_X): quaternion_schema,    
            cv.Optional(CONF_QUAT_Y): quaternion_schema,
            cv.Optional(CONF_QUAT_Z): quaternion_schema,
            cv.Optional(CONF_ROLL): roll_schema,
            cv.Optional(CONF_PITCH): pitch_schema,
            cv.Optional(CONF_YAW): yaw_schema,
        }
    )
    .extend(i2c.i2c_device_schema(0x68))  # Default I2C address for ICM-20948 is 0x68
)

ICM_ACTION_SCHEMA = cv.maybe_simple_value(
    {
        cv.Required(CONF_ID): cv.use_id(icm20948Component),
        cv.Required(CONF_STATE): cv.boolean,
    },
    key=CONF_STATE,
)

@automation.register_action(
    "icm20948.set_calibrate_next_boot",
    SetCalibrateNextBootAction,
    ICM_ACTION_SCHEMA,
)
async def set_calibrate_next_boot_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    cg.add(var.set_calibrate_next_boot(config[CONF_STATE]))
    return var

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for d in ["x", "y", "z"]:
        key = f"accel_{d}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))
        key = f"gyro_{d}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_gyro_{d}_sensor")(sens))
        key = f"mag_{d}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_mag_{d}_sensor")(sens))
    for d in ["w", "x", "y", "z"]:
        key = f"quat_{d}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_q{d}_sensor")(sens))
    for d in ["roll", "pitch", "yaw"]:
        key = f"{d}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{d}_sensor")(sens))
