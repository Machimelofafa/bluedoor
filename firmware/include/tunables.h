// Bluedoor — detection & logging tunables (not secret; committed).
// Start values; tune them from your own logger data (README, "Tuning").
#pragma once

// ---- state machine ----
// Absence required before arming (and before a sighting can count as arrival)
#define AWAY_MIN_MS            (10u * 60u * 1000u)
// "Near enough" RSSI threshold for a would-open. Two calibrations, because the
// two radios land ~10 dB apart at the same spot: the classic value was set for
// the car head unit, the BLE one for the in-car beacon. The beacon figure comes
// from a drive test: the car body costs ~20 dB and a closed garage door another
// ~20, so a +9 dBm beacon reads around -84 in the garage and -77..-80 at the
// door. Identity comes from the fixed MAC, so a loose threshold cannot let
// another device false-trigger.
#if DETECT_BLE
#define RSSI_TRIGGER_DBM       (-90)
#else
#define RSSI_TRIGGER_DBM       (-80)
#endif
// Strict arrival = approach signature: >= this many sightings ...
#define APPROACH_MIN_SIGHTINGS 3
// ... with total RSSI rise (max - first) of at least this many dB
#define APPROACH_MIN_RISE_DB   6
// ... measured between the median of the first and of the last N sightings,
// not single samples: consecutive samples at rest spread up to 8 dB, and on
// 2026-09-06 10:42 three samples inside one second (-92 -84 -88) fired the
// rule on a car that had not moved. The ramp must also last at least this long
#define APPROACH_MEDIAN_N      4
#define APPROACH_MIN_MS        (2u * 1000u)
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
// After an encounter that ended WITHOUT a verdict (car lingered in range, then
// left or went quiet), re-arm after this much silence instead of AWAY_MIN_MS.
// 2026-09-06 00:38: the car waited 2 min at the ramp top (no ramp, no verdict),
// went silent 4.5 min, then came down the ramp into a scanner still disarmed
// for another 6 min. A verdict still enters the full lockout, and a sighting at
// or above WAKE_STRONG_DBM while disarmed (the car came in and parked) cancels
// the short re-arm, so the parked-car door-open blip keeps its 10 min cover.
#define REARM_NOVERDICT_MS     (2u * 60u * 1000u)

// ---- car presence (home / away) ----
// A parked car is silent, so "no beacon for 10 min" is true both away and in
// the garage. On 2026-09-06 three departures (beacon powering on at the parked
// level, car then driving out past the scanner) fired the strict rule, and
// both real arrivals were lost to the lockouts those verdicts caused. Presence
// is therefore tracked from the shape of each beacon session. A session is one
// run of sightings separated by PRESENCE_QUIET_MS of silence (60 s: a
// departure prep showed 40 s pauses). A session with a transit (peak >=
// PRESENCE_TRANSIT_DBM: the car passing the scanner, measured -67..-73 both
// ways; parked reads -83..-96, an idling fragment once peaked -77) sets
// presence from the time before and after that peak. An arrival is heard for
// seconds, passes, then sits parked with the engine on and the USB grace
// (54..88 s measured). A departure idles inside (17..170 s), passes, and is
// out of range in 7..12 s. So: longer after the peak than before = HOME, else
// AWAY. Received level cannot do this (the parked and ramp-top bands overlap;
// a level rule misread two of three transits on 2026-09-06). Sessions without
// a transit (door-open blip, a wait at the top of the ramp) change nothing.
// An accepted strict arrival sets HOME immediately, even if its session never
// reaches PRESENCE_TRANSIT_DBM. That same session cannot change presence again;
// a separate departure session is needed to restore AWAY. This applies in all
// run modes and is detection evidence, not confirmation that the door opened.
// While HOME no encounter can fire. Boot assumes HOME, so the failure after a
// reflash is a missed arrival, never a stray press; the status page can set
// presence by hand.
#define PRESENCE_QUIET_MS      (60u * 1000u)
#define PRESENCE_TRANSIT_DBM   (-75)

// ---- classic BT scanning ----
// Inquiry length in 1.28 s units (4 = 5.12 s per cycle, restarted continuously)
#define INQ_LEN_UNITS          4
// Targeted remote-name probe (presence without RSSI; classic-radio targets only)
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
// Bench note: a loose Dupont contact can make the pin look dead (chip reads
// its pad HIGH, header pin meters 0 V). Every pulse logs a pad read-back and a
// load check for exactly that kind of hunt.
#define PIN_PULSE              26
// Web controls (mode switch, manual pulse) normally demand CONTROL_TOKEN from
// config.h. 0 = no token asked and no token fields on the page — bench work on
// a trusted LAN only; the page shows a red banner while this is 0. Set back to
// 1 before the scanner goes anywhere near the door.
#define WEB_CONTROLS_NEED_TOKEN 1
// 250 ms reads as one clean press on a BFT MITTO. Door ignores it: lengthen;
// double-triggers: shorten. Test pulses within range of the door only (README:
// rolling-code counters advance on presses the door cannot hear).
#define PULSE_MS               250u
// v2 reed door-closed interlock — stubbed off in v1 (README, "Adapting it").
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
// Census: every BLE_SURVEY_MS, one SURVEY line per advertiser
// heard that interval (count + first/best/last RSSI + name). It makes range
// tests reflash-free: drive
// up with any advertiser and read its RSSI trail out of the log — no need to
// know its address beforehand (phones rotate theirs; give the phone advertiser
// a local name to make the trail self-identifying). This firmware deliberately
// records every address it hears while the census is on; set to 0 once
// BEACON_BLE_MAC is commissioned. The env:survey build extends the census to
// classic inquiry responses (lines tagged BT vs BLE) for drive-through tests.
#define BLE_SURVEY             0
// 5 s buckets: an arrival is only a few seconds long, so drive-through tests
// need street / approach / door-wait in separate buckets or the gradient
// smears into one line. Widen to 20 s if a census build runs unattended for
// days, to save log churn.
#define BLE_SURVEY_MS          (5u * 1000u)
// Census table: max distinct addresses per interval; extras are counted and
// logged as one overflow line, strongest stay in the table.
#define BLE_SURVEY_SLOTS       12
// BLE scanning runs continuously; restart it this often as a liveness watchdog.
#define BLE_SCAN_RESTART_MS    (5u * 60u * 1000u)
