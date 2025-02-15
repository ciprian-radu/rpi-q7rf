#pragma once

#include <stdint.h>
#include <stddef.h>
#include <climits>
#include <cstring>
#include <cstdio>
#include <bitset>
#include <chrono>

namespace q7rf {

static const uint8_t MSG_NONE = 0;
static const uint8_t MSG_HEAT_ON = 1;
static const uint8_t MSG_HEAT_OFF = 2;
static const uint8_t MSG_PAIR = 3;

class Q7RFSwitch {
 protected:
  uint16_t q7rf_device_id_ = 0;
  uint32_t q7rf_resend_interval_ = 60000;
  uint32_t q7rf_turn_on_watchdog_interval_ = 0;

  bool initialized_ = false;
  uint8_t msg_pair_[45];
  uint8_t msg_heat_on_[45];
  uint8_t msg_heat_off_[45];
  bool state_ = false;
  uint8_t pending_msg_ = MSG_NONE;
  unsigned long last_msg_time_ = 0;
  unsigned long last_turn_on_time_ = 0;
  uint8_t msg_errors_ = 0;

 private:
  bool reset_cc();
  void send_cc_cmd(uint8_t cmd);
  void read_cc_register(uint8_t reg, uint8_t *value);
  void read_cc_config_register(uint8_t reg, uint8_t *value);
  void write_cc_register(uint8_t reg, uint8_t *value, size_t length);
  void write_cc_config_register(uint8_t reg, uint8_t value);
  bool send_cc_data(const uint8_t *data, size_t length);

  bool send_msg(uint8_t msg);
  void set_state(bool state);
  long millis();

 public:
  Q7RFSwitch() {}
  void setup();
  void write_state(bool state);
  void dump_config();
  void update();

  void set_q7rf_device_id(uint16_t id);
  void set_q7rf_resend_interval(uint32_t interval);
  void set_q7rf_turn_on_watchdog_interval(uint32_t interval);
  void on_pairing();
};

}  // namespace q7rf
