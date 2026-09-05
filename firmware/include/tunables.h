// Bluedoor — detection & logging tunables (not secret; committed).
// Start values per README.md Phase 2; the whole point of the Phase 0.5 week
// is to tune these from real log data.
#pragma once

// ---- state machine ----
// Absence required before arming (and before a sighting can count as arrival)
#define AWAY_MIN_MS            (10u * 60u * 1000u)
// "Near enough" RSSI threshold for a would-open. Two calibrations, because the
// two radios land ~10 dB apart at the same spot: the classic value was set for
// the car head unit, the BLE one for the in-car beacon. Beacon figure from the
// 2026-08-29 drive test (car body ~20 dB; a +1 dBm phone in the car topped out
// at -79 from the window, so -80 could never fire); the beacon's +9 dBm should
// read ~-84 in the garage, -77..-80 at the door. Identity comes from the fixed
// MAC, so a loose threshold cannot let another device false-trigger.
#if DETECT_BLE
#define RSSI_TRIGGER_DBM       (-90)
#else
#define RSSI_TRIGGER_DBM       (-80)
#endif
// Strict arrival = approach signature: >= this many sightings ...
#define APPROACH_MIN_SIGHTINGS 3
// ... with total RSSI rise (max - first) of at least this many dB
#define APPROACH_MIN_RISE_DB   6
// Relaxed rule (logged for comparison, never the primary verdict):
// >= 2 sightings at/above RSSI_TRIGGER_DBM within this window
#define RELAXED_MIN_SIGHTINGS  2
#define RELAXED_WINDOW_MS      (60u * 1000u)
// Wake-in-place signature (logged as an explicit refusal): first sighting
// already strong and the whole encounter flat. On the beacon this is the
// door-open blip — car USB re-powers for ~10-20 s when a door opens, so a
// parked car briefly shouts; scaled with the trigger, since "strong" only
// means anything relative to it.
#if DETECT_BLE
#define WAKE_STRONG_DBM        (-85)
#else
#define WAKE_STRONG_DBM        (-60)
#endif
#define WAKE_FLAT_DB           4
// An encounter (sighting cluster while armed) ends without verdict after
// this much quiet, or this much total lingering
#define ENCOUNTER_QUIET_MS     (90u * 1000u)
#define ENCOUNTER_MAX_MS       (3u * 60u * 1000u)

// ---- classic BT scanning ----
// Inquiry length in 1.28 s units (4 = 5.12 s per cycle, restarted continuously)
#define INQ_LEN_UNITS          4
// Targeted remote-name probe (presence without RSSI, README.md fallback method)
#define PROBE_INTERVAL_MS      (4u * 60u * 1000u)
#define PROBE_TIMEOUT_MS       (12u * 1000u)
// Skip probing if the car answered inquiry recently anyway
#define PROBE_SKIP_IF_SEEN_MS  (2u * 60u * 1000u)
// Watchdog: restart discovery if no inquiry-complete event for this long
#define SCAN_STUCK_MS          (120u * 1000u)

// ---- actuation (v1 builds only; PULSE_ENABLED comes from platformio.ini) ----
// Wiring per COMPONENTS.md: pulse pin -> 330R -> PC817 LED, PC817 output across
// the remote's button pads. Non-strapping pin; the 10k hardware pulldown on
// the opto drive is NOT optional.
// 2026-09-05 bench note: a loose Dupont contact once made pin 26 look dead
// (chip read its pad HIGH, header pin metered 0 V). The pin is fine; every
// pulse logs a pad read-back and a load check for that kind of hunt.
#define PIN_PULSE              26
// Web controls (mode switch, manual pulse) normally demand CONTROL_TOKEN from
// config.h. 0 = no token asked and no token fields on the page — bench work on
// a trusted LAN only; the page shows a red banner while this is 0. Set back to
// 1 before the scanner returns to the garage. (2026-09-05: 0 for the bench
// hunt, back to 1 for the garage install the same day.)
#define WEB_CONTROLS_NEED_TOKEN 1
// CONFIRMED 2026-09-05: four 250 ms pulses beside the door, four openings.
// (From two floors up the LED lit but the door stayed shut — range and/or
// rolling-code counter drift; see README. Test pulses within earshot.)
#define PULSE_MS               250u
// v2 reed door-closed interlock — stubbed off in v1 (README.md).
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
// Commissioning census: every BLE_SURVEY_MS, one SURVEY line per advertiser
// heard that interval (count + first/best/last RSSI + name). In this RF-quiet
// spot that is a handful of lines, and it makes range tests reflash-free: drive
// up with any advertiser and read its RSSI trail out of the log — no need to
// know its address beforehand (phones rotate theirs; give the phone advertiser
// a local name to make the trail self-identifying). This firmware deliberately
// records every address it hears while the census is on; set to 0 once
// BEACON_BLE_MAC is commissioned. The env:survey build extends the census to
// classic inquiry responses (lines tagged BT vs BLE) for drive-through tests.
#define BLE_SURVEY             1
// 5 s buckets: the whole arrival is fast (~3 s ramp descent + 4-10 s waiting
// at the door), so drive-through tests need street / descent / door-wait in
// separate buckets or the gradient smears into one line. Costs ~4x the log
// churn of the earlier 20 s setting — acceptable for the attended survey
// build; widen it again if a census build ever runs unattended for days.
#define BLE_SURVEY_MS          (5u * 1000u)
// Census table: max distinct addresses per interval; extras are counted and
// logged as one overflow line, strongest stay in the table.
#define BLE_SURVEY_SLOTS       12
// BLE scanning runs continuously; restart it this often as a liveness watchdog.
#define BLE_SCAN_RESTART_MS    (5u * 60u * 1000u)
