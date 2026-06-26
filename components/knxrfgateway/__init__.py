import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

CONF_SENSORS = "sensors"
CONF_KNX_ID  = "knx_id"

AUTO_LOAD = ["sensor"]

knxrfgateway_ns = cg.global_ns
KNXRFGateway = knxrfgateway_ns.class_("KNXRFGateway", cg.Component)

SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
).extend({
    cv.Required(CONF_KNX_ID): cv.string,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(KNXRFGateway),
    cv.Required(CONF_SENSORS): cv.All(
        cv.ensure_list(SENSOR_SCHEMA),
        cv.Length(min=1, max=32),
    ),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    cg.add_library("SmartRC-CC1101-Driver-Lib", None)
    cg.add_library("Crc16", "2.0.0", "https://github.com/vinmenn/Crc16.git")
    cg.add_library("SPI", None)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    for entry in config[CONF_SENSORS]:
        sens = await sensor.new_sensor(entry)
        cg.add(var.add_sensor(entry[CONF_KNX_ID], sens))
