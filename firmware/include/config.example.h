// Bluedoor — secrets & identity. Copy to config.h and fill in real values.
// config.h is gitignored: the car MAC + WiFi credentials never enter git
// (see PLAN.md "Data hygiene").
#pragma once

#define WIFI_SSID       "your-wifi-ssid"
#define WIFI_PASS       "your-wifi-password"

// The car head unit's classic BT MAC ("Uconnect"), from Phase 0.
#define CAR_BT_MAC      "AA:BB:CC:DD:EE:FF"

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
