// Bluedoor beacon — the second ESP32, carried in the car (env:beacon).
//
// Powered from the car's USB port, which is switched: ~10-15 s of grace after
// ignition off, brief re-power while a door is open. So this build keeps
// nothing: no WiFi, no OTA, no flash writes — power can vanish mid-anything
// and the next ignition just boots it fresh (measured 2026-08-30: 1.0 s from
// reset to the first advertisement an outside scanner hears, well inside even
// a door-open blip). Reflash over USB in the car or on the bench.
//
// It advertises its burned-in public MAC (the fixed identity BEACON_BLE_MAC
// wants — commissioning: read the MAC off this build's serial output, or off
// the scanner's SURVEY census, where the advertised name identifies it) with
// a name-carrying non-connectable packet every 100 ms at +9 dBm — the banner
// reports the power read back from the controller, not the one asked for, so a
// silently clamped radio cannot pass for a working one. The 2026-08-29
// phone-in-car test measured the car body at ~20 dB and the phone's +1 dBm
// topping out at -79 seen from the window; +9 dBm is the whole margin this
// link has, so max TX power is a requirement, not a tweak.
#if BEACON_BUILD

#include <Arduino.h>
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

static const char BEACON_NAME[] = "bluedoor-beacon";
#define ADV_INTERVAL_UNITS 0xA0   // 100 ms in 0.625 ms units — spec minimum for non-connectable
#define PIN_LED 2                 // devkit onboard LED: brief blip = advertising, for in-car sanity checks

static esp_ble_adv_params_t advParams = {
    .adv_int_min = ADV_INTERVAL_UNITS,
    .adv_int_max = ADV_INTERVAL_UNITS,
    .adv_type = ADV_TYPE_NONCONN_IND,   // broadcast only: nothing to connect to, nothing to scan-request
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,   // the burned-in MAC — the scanner filters on this
    .peer_addr = {0},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT)
    esp_ble_gap_start_advertising(&advParams);
}

// The controller stores TX power as a level, N12 = 0 in 3 dB steps. +9 dBm is
// the whole link margin here, so the beacon reports what the controller
// actually took rather than what it was asked for — a silently clamped radio
// would otherwise look identical to a working one.
static int txPowerDbm() {
  return -12 + 3 * (int)esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
}

static void printMac() {
  const uint8_t *m = esp_bt_dev_get_address();
  Serial.printf("beacon MAC (put in scanner config.h BEACON_BLE_MAC): "
                "%02X:%02X:%02X:%02X:%02X:%02X\n",
                m[0], m[1], m[2], m[3], m[4], m[5]);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  Serial.printf("bluedoor beacon %s built %s %s\n", FW_VERSION, __DATE__, __TIME__);

  if (!btStart() || esp_bluedroid_init() != ESP_OK || esp_bluedroid_enable() != ESP_OK) {
    // no radio = no beacon; a boot loop beats advertising nothing
    Serial.println("BT init failed, restarting");
    delay(2000);
    ESP.restart();
  }
  esp_ble_gap_register_callback(gapCallback);
  if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9) != ESP_OK)
    Serial.println("WARNING: TX power set failed — the link has no margin to spare");

  // Name goes in the advertising packet itself, not a scan response — the
  // scanner scans passively and would never see a scan response.
  uint8_t adv[31];
  uint8_t i = 0;
  adv[i++] = 2;
  adv[i++] = ESP_BLE_AD_TYPE_FLAG;
  adv[i++] = 0x04;   // BR/EDR not supported; non-discoverable is fine for a broadcast-only device
  adv[i++] = 1 + sizeof(BEACON_NAME) - 1;
  adv[i++] = ESP_BLE_AD_TYPE_NAME_CMPL;
  memcpy(&adv[i], BEACON_NAME, sizeof(BEACON_NAME) - 1);
  i += sizeof(BEACON_NAME) - 1;
  esp_ble_gap_config_adv_data_raw(adv, i);   // completion event starts advertising

  printMac();
  Serial.printf("advertising '%s' @100ms, TX %+d dBm\n", BEACON_NAME, txPowerDbm());
}

void loop() {
  // 50 ms LED blip every 3 s; serial heartbeat each minute for bench debugging
  digitalWrite(PIN_LED, HIGH);
  delay(50);
  digitalWrite(PIN_LED, LOW);
  static uint32_t lastHb = 0;
  if (millis() - lastHb >= 60u * 1000u) {
    lastHb = millis();
    Serial.printf("up %lus, advertising '%s' @100ms %+ddBm, ", millis() / 1000, BEACON_NAME,
                  txPowerDbm());
    printMac();
  }
  delay(2950);
}

#endif  // BEACON_BUILD
