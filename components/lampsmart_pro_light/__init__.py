import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32_ble
from esphome.const import (
    PLATFORM_ESP32,
)

AUTO_LOAD = ["esp32_ble"]
DEPENDENCIES = ["esp32"]

CONF_QUEUE_ID = "queue_id"

lampsmartpro_ns = cg.esphome_ns.namespace('lampsmartpro')
LampSmartProQueue = lampsmartpro_ns.class_('LampSmartProQueue', cg.Component, 
    cg.Parented.template(esp32_ble.ESP32BLE))

CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_QUEUE_ID): cv.declare_id(LampSmartProQueue),
            cv.GenerateID(esp32_ble.CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
        }
    ),
    cv.only_on([PLATFORM_ESP32]),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_QUEUE_ID])

    parent = await cg.get_variable(config[esp32_ble.CONF_BLE_ID])
    cg.add(var.set_parent(parent))

    await cg.register_component(var, config)

