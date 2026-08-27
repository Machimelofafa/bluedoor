// Bluedoor — detection & logging tunables (not secret; committed).
// Start values per PLAN.md Phase 2; the whole point of the Phase 0.5 week
// is to tune these from real log data.
#pragma once

// ---- state machine ----
// Absence required before arming (and before a sighting can count as arrival)
#define AWAY_MIN_MS            (10u * 60u * 1000u)
// "Near enough" RSSI threshold for a would-open (start ~-80 dBm, tune)
#define RSSI_TRIGGER_DBM       (-80)
// Strict arrival = approach signature: >= this many sightings ...
#define APPROACH_MIN_SIGHTINGS 3
// ... with total RSSI rise (max - first) of at least this many dB
#define APPROACH_MIN_RISE_DB   6
// Relaxed rule (logged for comparison, never the primary verdict):
// >= 2 sightings at/above RSSI_TRIGGER_DBM within this window
#define RELAXED_MIN_SIGHTINGS  2
#define RELAXED_WINDOW_MS      (60u * 1000u)
// Wake-in-place signature (logged as an explicit refusal): first sighting
// already strong and the whole encounter flat
#define WAKE_STRONG_DBM        (-60)
#define WAKE_FLAT_DB           4
// An encounter (sighting cluster while armed) ends without verdict after
// this much quiet, or this much total lingering
#define ENCOUNTER_QUIET_MS     (90u * 1000u)
#define ENCOUNTER_MAX_MS       (3u * 60u * 1000u)

// ---- classic BT scanning ----
// Inquiry length in 1.28 s units (4 = 5.12 s per cycle, restarted continuously)
#define INQ_LEN_UNITS          4
// Targeted remote-name probe (presence without RSSI, PLAN.md fallback method)
#define PROBE_INTERVAL_MS      (4u * 60u * 1000u)
#define PROBE_TIMEOUT_MS       (12u * 1000u)
// Skip probing if the car answered inquiry recently anyway
#define PROBE_SKIP_IF_SEEN_MS  (2u * 60u * 1000u)
// Watchdog: restart discovery if no inquiry-complete event for this long
#define SCAN_STUCK_MS          (120u * 1000u)

// ---- actuation (v1 builds only; PULSE_ENABLED comes from platformio.ini) ----
// Wiring per COMPONENTS.md: GPIO 26 -> 330R -> PC817 LED, PC817 output across
// the remote's button pads. GPIO 26 is non-strapping; the 10k hardware
// pulldown on the opto drive is NOT optional.
#define PIN_PULSE              26
// HYPOTHESIS: 250 ms reads as a clean button press on the MITTO 12V-UP.
// Verify at commissioning; some boards want a longer press.
#define PULSE_MS               250u
// v2 reed door-closed interlock — stubbed off in v1 (PLAN.md).
// When fitted: alarm-type contact GPIO 27 -> GND, internal pullup,
// magnet adjacent (door closed) = LOW.
#define REED_ENABLED           0
#define PIN_REED               27

// ---- logging ----
// Trajectory label on sighting-aggregate and encounter summaries:
// |last - first| >= this many dB logs as approaching/receding, else steady
// (steady + strong = parked car woken in place, e.g. door opened)
#define TREND_MIN_DB           6
// Disarmed sighting aggregate flushes early once sightings stop for this
// long, timestamping the moment the car went quiet (drove off / fell asleep)
#define SIGHT_AGG_QUIET_MS     (45u * 1000u)
#define HEARTBEAT_MS           (30u * 60u * 1000u)
// The LittleFS partition (min_spiffs.csv "spiffs") is 128 KB, not the ~190 KB
// the README claimed, and rotation keeps TWO files: 2 x 80 KB would have run
// the filesystem out of space mid-week. 2 x 48 KB fits with room for metadata.
#define LOG_ROTATE_BYTES       (48u * 1024u)   // 2 files x 48 KB in a 128 KB FS
#define SIGHT_AGG_WINDOW_MS    (60u * 1000u)   // aggregate disarmed sightings
#define RING_LINES             120             // in-RAM tail for the web page
#define RING_LINE_LEN          168             // fits a full ENC summary line

// ---- radio diagnostics (Phase 0.5 troubleshooting) ----
// Periodic census of what the inquiry radio actually hears — every device
// report, not just the car. Distinguishes "nothing was in range" from "the
// radio was starved", which free-running sighting logs cannot.
#define RADIO_STATS_MS         (5u * 60u * 1000u)
// WiFi/BT coexistence test. WiFi and classic BT share one antenna; if
// coexistence is eating inquiry time, radio-quiet windows will hear more than
// WiFi-up windows. Alternating them inside one run makes the two directly
// comparable — same day, same car, same mounting spot. Every boot starts ON,
// so OTA is always reachable within WIFI_TEST_ON_MS of a reset: no lockout,
// and setting WIFI_DUTY_TEST to 0 ends the test.
#define WIFI_DUTY_TEST         0
#define WIFI_TEST_ON_MS        (10u * 60u * 1000u)
#define WIFI_TEST_OFF_MS       (20u * 60u * 1000u)

// ---- BLE beacon mode (DETECT_BLE=1 builds, see platformio.ini env logger-ble) ----
// Commissioning survey: logs the strongest BLE advertiser's address once a
// minute so a beacon held against the box identifies itself. Set to 0 once
// BEACON_BLE_MAC is filled in — it is the only place this firmware records
// an address that is not the target's.
#define BLE_SURVEY             1
#define BLE_SURVEY_MS          (60u * 1000u)
// BLE scanning runs continuously; restart it this often as a liveness watchdog.
#define BLE_SCAN_RESTART_MS    (5u * 60u * 1000u)
