#include "lampsmart_pro_light.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <mbedtls/aes.h>
#include <freertos/queue.h>

namespace esphome {
namespace lampsmartpro {

static const char *TAG = "lampsmartpro";

const size_t MAX_PACKET_LEN = 31;

#pragma pack(push, 1)
typedef union {
  struct { /* Advertising Data for version 2/3*/
    uint8_t prefix[10];
    uint8_t packet_number;
    uint16_t type;
    uint32_t group_id;
    uint8_t group_index;
    uint8_t command;
    uint8_t _19;
    uint16_t _20;
    uint8_t channel1;
    uint8_t channel2;
    uint16_t signature_v3;
    uint8_t _26;
    uint16_t rand;
    uint16_t crc16;
  };
  uint8_t raw[MAX_PACKET_LEN];
} adv_data_v3_t;

typedef union {
  struct { /* Advertising Data for version 1*/
    uint8_t prefix[8];
    uint8_t command;
    uint16_t group_idx;
    uint8_t channel1;
    uint8_t channel2;
    uint8_t channel3;
    uint8_t tx_count;
    uint8_t outs;
    uint8_t src;
    uint8_t r2;
    uint16_t seed;
    uint16_t crc16;
  };
  uint8_t raw[22];
} adv_data_v1_t;
#pragma pack(pop)

static esp_ble_adv_params_t ADVERTISING_PARAMS = {
  .adv_int_min = 0x20,
  .adv_int_max = 0x20,
  .adv_type = ADV_TYPE_NONCONN_IND,
  .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
  .peer_addr =
    {
      0x00,
    },
  .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
  .channel_map = ADV_CHNL_ALL,
  .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t XBOXES[128] = {
  0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC,
  0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
  0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A,
  0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
  0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85,
  0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
  0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5,
  0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
  0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C,
  0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
  0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9,
  0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
  0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94,
  0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
  0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68,
  0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16
};

void v2_whiten(uint8_t *buf, uint8_t size, uint8_t seed, uint8_t salt) {
  for (uint8_t i = 0; i < size; ++i) {
    buf[i] ^= XBOXES[(seed + i + 9) & 0x1f + (salt & 0x3) * 0x20];
    buf[i] ^= seed;
  }
}

uint16_t v2_crc16_ccitt(uint8_t *src, uint8_t size, uint16_t crc16_result) {
  for (uint8_t i = 0; i < size; ++i) {
    crc16_result = crc16_result ^ (*(uint16_t*) &src[i]) << 8;
    for (uint8_t j = 8; j != 0; --j) {
      if ((crc16_result & 0x8000) == 0) {
        crc16_result <<= 1;
      } else {
        crc16_result = crc16_result << 1 ^ 0x1021;
      }
    }
  }

  return crc16_result;
}

void ble_whiten(uint8_t *buf, size_t start, size_t len, uint8_t seed) {
  uint8_t r = seed;
  for (size_t i=0; i < start+len; i++) {
    uint8_t b = 0;
    for (size_t j=0; j < 8; j++) {
      r <<= 1;
      if (r & 0x80) {
        r ^= 0x11;
        b |= 1 << j;
      }
      r &= 0x7F;
    }
    if (i >= start) {
      buf[i - start] ^= b;
    }
    //ESP_LOGD(TAG, "%0x", b);
  }
}

uint8_t reverse_byte(uint8_t x) {

  x = ((x & 0x55) << 1) | ((x & 0xAA) >> 1);
  x = ((x & 0x33) << 2) | ((x & 0xCC) >> 2);
  x = ((x & 0x0F) << 4) | ((x & 0xF0) >> 4);
  return x;
}

void LampSmartProLight::setup() {
#ifdef USE_API
  register_service(&LampSmartProLight::on_pair, light_state_ ? "pair_" + light_state_->get_object_id() : "pair");
  register_service(&LampSmartProLight::on_unpair, light_state_ ? "unpair_" + light_state_->get_object_id() : "unpair");
#endif
  this->queue_ = this->parent_;
}

light::LightTraits LampSmartProLight::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::COLD_WARM_WHITE});
  traits.set_min_mireds(this->cold_white_temperature_);
  traits.set_max_mireds(this->warm_white_temperature_);
  return traits;
}

void LampSmartProLight::write_state(light::LightState *state) {
  float cwf, wwf;
  state->current_values_as_cwww(&cwf, &wwf, this->constant_brightness_);

  if (!cwf && !wwf) {
    send_packet(CMD_TURN_OFF, 0, 0);
    _is_off = true;

    return;
  }

  uint8_t cwi = (uint8_t)(0xff * cwf);
  uint8_t wwi = (uint8_t)(0xff * wwf);

  if ((cwi < min_brightness_) && (wwi < min_brightness_)) {
    if (cwf > 0.000001) {
      cwi = min_brightness_;
    }
    
    if (wwf > 0.000001) {
      wwi = min_brightness_;
    }
  }

  ESP_LOGD(TAG, "LampSmartProLight::write_state called! Requested cw: %d, ww: %d", cwi, wwi);

  if (_is_off) {
    send_packet(CMD_TURN_ON, 0, 0);
    _is_off = false;
  }

  send_packet(CMD_DIM, cwi, wwi);
}

void LampSmartProLight::dump_config() {
  ESP_LOGCONFIG(TAG, "LampSmartProLight '%s'", light_state_ ? light_state_->get_name().c_str() : "");
  ESP_LOGCONFIG(TAG, "  Cold White Temperature: %f mireds", cold_white_temperature_);
  ESP_LOGCONFIG(TAG, "  Warm White Temperature: %f mireds", warm_white_temperature_);
  ESP_LOGCONFIG(TAG, "  Constant Brightness: %s", constant_brightness_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Minimum Brightness: %d", min_brightness_);
  ESP_LOGCONFIG(TAG, "  Transmission Duration: %d millis", tx_duration_);
}

void LampSmartProLight::on_pair() {
  ESP_LOGD(TAG, "LampSmartProLight::on_pair called!");
  send_packet(CMD_PAIR, 0, 0);
}

void LampSmartProLight::on_unpair() {
  ESP_LOGD(TAG, "LampSmartProLight::on_unpair called!");
  send_packet(CMD_UNPAIR, 0, 0);
}

void sign_packet_v3(adv_data_v3_t* packet) {
  uint16_t seed = packet->rand;
  uint8_t sigkey[16] = {0, 0, 0, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16};
  uint8_t tx_count = (uint8_t) packet->packet_number;

  sigkey[0] = seed & 0xff;
  sigkey[1] = (seed >> 8) & 0xff;
  sigkey[2] = tx_count;
  mbedtls_aes_context aes_ctx;
  mbedtls_aes_init(&aes_ctx);
  mbedtls_aes_setkey_enc(&aes_ctx, sigkey, sizeof(sigkey)*8);
  uint8_t aes_in[16], aes_out[16];
  memcpy(aes_in, &(packet->raw[8]), 16);
  mbedtls_aes_crypt_ecb(&aes_ctx, ESP_AES_ENCRYPT, aes_in, aes_out);
  mbedtls_aes_free(&aes_ctx);
  packet->signature_v3 = ((uint16_t*) aes_out)[0]; 
  if (packet->signature_v3 == 0) {
      packet->signature_v3 = 0xffff;
  }
}

size_t LampSmartProCommand::build_packet_v3(uint8_t* buf) {
  uint16_t seed = (uint16_t) rand();

  adv_data_v3_t *packet = (adv_data_v3_t*)buf;
  *packet = (adv_data_v3_t) {{
      .prefix = {0x02, 0x01, 0x02, 0x1B, 0x16, 0xF0, 0x08, 0x10, 0x80, 0x00},
      .packet_number = this->tx_count_,
      .type = DEVICE_TYPE_LAMP,
      .group_id = this->identifier_,
      .group_index = 0,
      .command = this->cmd_,
      ._19 = 0,
      ._20 = 0,
      .channel1 = this->par1_,
      .channel2 = this->par2_,
      .signature_v3 = 0,
      ._26 = 0,
      .rand = seed,
  }};

  if (this->variant_ == VARIANT_3) {
    sign_packet_v3(packet);
  }
  v2_whiten(&packet->raw[9], 0x12, (uint8_t) seed, 0);
  packet->crc16 = v2_crc16_ccitt(&packet->raw[7], 0x16, ~seed);
  
  return sizeof(*packet);
}
size_t LampSmartProCommand::build_packet_v1(uint8_t* buf, size_t base) {
  uint16_t seed = (uint16_t) rand() % 65525;

  adv_data_v1_t *packet = (adv_data_v1_t*)&buf[base];
  *packet = (adv_data_v1_t) {{
      .prefix = {0xAA, 0x98, 0x43, 0xAF, 0x0B, 0x46, 0x46, 0x46},
      .command = this->cmd_,
      .group_idx = static_cast<uint16_t>(this->identifier_ & 0xF0FF), // group_index is zero for now
      .channel1 = this->par1_,
      .channel2 = this->par2_,
      .channel3 = 0,
      .tx_count = this->tx_count_,
      .outs = 0,
      .src = static_cast<uint8_t>(seed ^ 1), //
      .r2 = static_cast<uint8_t>(seed ^ 1),  // mimics IR remote behavior
      .seed = htons(seed),
      .crc16 = 0,
  }};

  if (this->cmd_ == CMD_PAIR) {
    packet->channel1 = static_cast<uint8_t>(packet->group_idx & 0xFF);
    packet->channel2 = static_cast<uint8_t>(packet->group_idx >> 8);
    packet->channel3 = 0x81;
  }

  packet->crc16 = htons(v2_crc16_ccitt(&packet->raw[8], 12, ~seed));
  
  return sizeof(*packet);
}

size_t LampSmartProCommand::build_packet_v1a(uint8_t* buf) {

  uint8_t header[] = {0x02, 0x01, 0x02, 0x1B, 0x03, 0x77, 0xF8};
  const size_t base = 7;
  const size_t size = 31;
  
  for (size_t i=0; i<sizeof(header); i++) {
    buf[i] = header[i];
  }
  build_packet_v1(buf, base);
  
  uint16_t* crc16_2 = (uint16_t*) &buf[29];
  *crc16_2 = htons(v2_crc16_ccitt(&buf[base+8], 14, v2_crc16_ccitt(&buf[base+1], 5, 0xffff)));
  for (size_t i=base; i < size; i++) {
    buf[i] = reverse_byte(buf[i]);
  }
  ble_whiten(&buf[base], 8+base, size-base, 83);
  
  return size;
}

size_t LampSmartProCommand::build_packet_v1b(uint8_t* buf) {

  uint8_t header[] = {0x02, 0x01, 0x02, 0x1B, 0x03, 0xF9, 0x08, 0x49};
  const size_t base = 8;
  const size_t size = 31;
  
  for (size_t i=0; i<sizeof(header); i++) {
    buf[i] = header[i];
  }
  build_packet_v1(buf, base);
  
  buf[30] = 0xaa;
  for (size_t i=base; i < size; i++) {
    buf[i] = reverse_byte(buf[i]);
  }
  ble_whiten(&buf[base], 8+base, size-base, 83);
  
  return size;
}

size_t LampSmartProCommand::build_packet(uint8_t* buf) {
  size_t plen = 0;
  
  switch (this->variant_) {
    case VARIANT_3:
    case VARIANT_2:
      plen = this->build_packet_v3(buf);
      break;
    case VARIANT_1A:
      plen = this->build_packet_v1a(buf);
      break;
    case VARIANT_1B:
      plen = this->build_packet_v1b(buf);
      break;
  }
  
  return plen;
} 

void LampSmartProLight::send_packet(uint16_t cmd, uint8_t cold, uint8_t warm) {

  if (++this->tx_count_ == 0) {
    this->tx_count_ = 1;
  }

  uint32_t identifier = light_state_ ? light_state_->get_object_id_hash() : 0xcafebabe;
  LampSmartProCommand *command = new LampSmartProCommand(this->variant_, identifier, cmd);
  command->set_par1(this->reversed_ ? warm : cold);
  command->set_par2(this->reversed_ ? cold : warm);
  command->set_tx_count(this->tx_count_);
  command->set_tx_duration(this->tx_duration_);
  
  this->queue_->put(command);
}

const size_t MAX_QUEUE_LEN = 10;

LampSmartProQueue::LampSmartProQueue(void) {
  this->commands_ = xQueueCreate(MAX_QUEUE_LEN, sizeof(LampSmartProCommand *));
}

void LampSmartProQueue::put(LampSmartProCommand* command) {
  if (xQueueSend(this->commands_, &command, 0) != pdTRUE) {
    ESP_LOGE(TAG, "Error putting command to queue");
  }
}

void LampSmartProQueue::loop() {

  static LampSmartProCommand *command;
  if (!this->parent_->is_active()) {
    ESP_LOGD(TAG, "Cannot proceed while ESP32BLE is disabled.");
    return;
  }
  if (this->advertising_) {
    if (millis() - this->op_start_ > this->tx_duration_) {
      this->advertising_ = false;
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_stop_advertising());
      ESP_LOGD(TAG, "Advertising stop");
    } else {
      return;
    }
  }
  
  if (xQueueReceive(this->commands_, &command, 0) == pdTRUE) {
    uint8_t packet[MAX_PACKET_LEN];
    size_t packet_len = command->build_packet(packet);
    ESP_LOGD(TAG, "Prepared packet: %s", esphome::format_hex_pretty(packet, packet_len).c_str());
    if (packet_len > 0) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_tx_power_set(9, 7));
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_config_adv_data_raw(packet, packet_len));
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_start_advertising(&ADVERTISING_PARAMS));
      ESP_LOGD(TAG, "Advertising start");
      this->op_start_ = millis();
      this->tx_duration_ = command->get_tx_duration();
      this->advertising_ = true;
    }

    delete command;
  }
}

} // namespace lampsmartpro
} // namespace esphome

#endif
