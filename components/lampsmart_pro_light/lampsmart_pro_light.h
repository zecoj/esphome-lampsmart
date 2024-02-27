#pragma once

#include "esphome.h"
#ifdef USE_API
#include "esphome/components/api/custom_api_device.h"
#endif
#include "esphome/components/light/light_output.h"

namespace esphome {
namespace lampsmartpro {

const uint16_t DEVICE_TYPE_LAMP = 0x100;

const uint8_t CMD_PAIR = 0x28;
const uint8_t CMD_UNPAIR = 0x45;
const uint8_t CMD_TURN_ON = 0x10;
const uint8_t CMD_TURN_OFF = 0x11;
const uint8_t CMD_DIM = 0x21;

class LampSmartProCommand {
  public:
    LampSmartProCommand(uint32_t id, uint8_t cmd);
    virtual size_t build_packet(uint8_t *buf);
    void set_tx_count(uint8_t tx_count) { this->tx_count_ = tx_count; }
    void set_par1(uint8_t par1) { this->par1_ = par1; }
    void set_par2(uint8_t par2) { this->par2_ = par2; }
    
  protected:
    uint32_t identifier_;
    uint8_t cmd_;
    uint8_t par1_;
    uint8_t par2_;
    uint8_t tx_count_;
};

class LampSmartProCommandV3 : public LampSmartProCommand {
  using LampSmartProCommand::LampSmartProCommand;
  public:
    size_t build_packet(uint8_t *buf) override;
};

class LampSmartProLight : public light::LightOutput, public Component, public EntityBase, public Parented<esphome::esp32_ble::ESP32BLE>
#ifdef USE_API
  , public api::CustomAPIDevice
#endif
{
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;

  void set_cold_white_temperature(float cold_white_temperature) { cold_white_temperature_ = cold_white_temperature; }
  void set_warm_white_temperature(float warm_white_temperature) { warm_white_temperature_ = warm_white_temperature; }
  void set_constant_brightness(bool constant_brightness) { constant_brightness_ = constant_brightness; }
  void set_reversed(bool reversed) { reversed_ = reversed; }
  void set_min_brightness(uint8_t min_brightness) { min_brightness_ = min_brightness; }
  void set_tx_duration(uint32_t tx_duration) { tx_duration_ = tx_duration; }
  void setup_state(light::LightState *state) override { this->light_state_ = state; }
  void write_state(light::LightState *state) override;
  light::LightTraits get_traits() override;
  void on_pair();
  void on_unpair();

 protected:
  void send_packet(uint16_t cmd, uint8_t cold, uint8_t warm);

  float cold_white_temperature_{167};
  float warm_white_temperature_{333};
  bool constant_brightness_;
  bool reversed_;
  uint8_t min_brightness_;
  bool _is_off;
  uint8_t tx_count_;
  uint32_t tx_duration_;
  light::LightState *light_state_;
  uint32_t op_start_;
  bool ble_ready_;
  bool advertising_;
  QueueHandle_t commands_;
};

template<typename... Ts> class PairAction : public Action<Ts...> {
 public:
  explicit PairAction(esphome::light::LightState *state) : state_(state) {}

  void play(Ts... x) override {
    ((LampSmartProLight *)this->state_->get_output())->on_pair();
  }

 protected:
  esphome::light::LightState *state_;
};

template<typename... Ts> class UnpairAction : public Action<Ts...> {
 public:
  explicit UnpairAction(esphome::light::LightState *state) : state_(state) {}

  void play(Ts... x) override {
    ((LampSmartProLight *)this->state_->get_output())->on_unpair();
  }

 protected:
  esphome::light::LightState *state_;
};

} //namespace lampsmartpro
} //namespace esphome
