// Bluedoor — secrets & identity. Copy to config.h and fill in real values.
// config.h is gitignored: the car MAC + WiFi credentials never enter git
// (see README.md, "Security and privacy").
#pragma once

#define WIFI_SSID       "your-wifi-ssid"
#define WIFI_PASS       "your-wifi-password"

// The car head unit's classic BT MAC ("Uconnect"), from Phase 0.
#define CAR_BT_MAC      "AA:BB:CC:DD:EE:FF"
// logger-ble builds watch this instead: the BLE beacon carried in the car.
// Not needed for range tests — BLE_SURVEY in tunables.h logs every advertiser
// heard (RSSI trail included), so any beacon identifies itself in the log.
// Fill this in once the real beacon exists; only the would-open state machine
// uses it.
#define BEACON_BLE_MAC  "AA:BB:CC:DD:EE:FF"
// Census hygiene: known fixed advertisers (the TV, ...) dropped from the SURVEY
// census before they cost a table slot or a log line. Comma-separated MACs,
// "" = keep everything. First 4 entries used.
#define SURVEY_IGNORE_MACS  ""

// Password for over-the-air reflashing (pio run -t upload --upload-port bluedoor.local).
// Blank or left as this placeholder = OTA stays disabled (USB reflash only).
#define OTA_PASSWORD    "change-me"

// v1 only: token required by the web controls (mode switch, manual pulse).
// This opens the house — pick a real one. Controls stay refused while the
// placeholder value is in place.
#define CONTROL_TOKEN   "change-me-too"

#define DEVICE_HOSTNAME "bluedoor"           // web page at http://bluedoor.local
#define TZ_INFO         "CET-1CEST,M3.5.0,M10.5.0/3"  // Europe/Rome
#define NTP_SERVER      "pool.ntp.org"
