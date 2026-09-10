// Bluedoor — detection & logging tunables (not secret; committed).
// Start values; tune them from your own logger data (README, "Tuning").
#pragma once

// ---- state machine ----
// Absence required before arming (and before a sighting can count as arrival)
#define AWAY_MIN_MS            (10u * 60u * 1000u)
// Opening requires near-ramp reception. The recorded upper-parking arrival
// peaked at -88; the garage approach reached a four-packet median of -72.
// -80 is an experimental separation, with a timed median gate below.
#if DETECT_BLE
#define RSSI_TRIGGER_DBM       (-80)
#else
#define RSSI_TRIGGER_DBM       (-80)
#endif
// Strict arrival = approach signature: >= this many sightings ...
#define APPROACH_MIN_SIGHTINGS 8
// ... with total RSSI rise (max - first) of at least this many dB
#define APPROACH_MIN_RISE_DB   6
// ... measured between the median of the first and of the last N sightings,
// not single samples: consecutive samples at rest spread up to 8 dB, and on
// 2026-09-06 10:42 three samples inside one second (-92 -84 -88) fired the
// rule on a car that had not moved. The ramp must also last at least this long
#define APPROACH_MEDIAN_N      4
#define APPROACH_MIN_MS        (2u * 1000u)
// Near medians and the latest sample must remain above the gate for 500 ms.
// A weak sample or gap over 1500 ms resets this evidence; a burst is not enough.
#define APPROACH_NEAR_MIN_MS   500u
#define APPROACH_NEAR_MAX_GAP_MS 1500u
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
// Also applies after a separately confirmed departure, even during lockout.
// After an encounter that ended WITHOUT a verdict (car lingered in range, then
// left or went quiet), re-arm after this much silence instead of AWAY_MIN_MS.
// 2026-09-06 00:38: the car waited 2 min at the ramp top (no ramp, no verdict),
// went silent 4.5 min, then came down the ramp into a scanner still disarmed
// for another 6 min. A verdict still enters the full lockout, and a sighting at
// or above WAKE_STRONG_DBM while disarmed (the car came in and parked) cancels
// the short re-arm, so the parked-car door-open blip keeps its 10 min cover.
#define REARM_NOVERDICT_MS     (2u * 60u * 1000u)

// ---- inferred garage presence and duplicate-command suppression ----
// Weak upper-property activity does not establish garage occupancy. A command
// sets a separate duplicate latch, and location stays UNKNOWN until a parked
// tail is observed. Only a separate credible departure clears that latch.
// Time-bucket medians establish a passage; sparse fading reception and quiet
// confirm departure. Equal/short head-tail traces remain ambiguous.
// Boot is UNKNOWN and blocks automatic pulses until movement establishes state.
#define PRESENCE_QUIET_MS      (60u * 1000u)
#define PRESENCE_PARKED_MIN_MS (15u * 1000u)
#define PRESENCE_DEPARTURE_HEAD_MS (15u * 1000u)
#define PRESENCE_DIRECTION_MARGIN_MS (10u * 1000u)
#define PRESENCE_FADED_DBM     (-85)
#define JOURNEY_NEAR_GAP_MS    3000u
#define JOURNEY_NEAR_PACKETS   4u
// A departure passage is corroborated by direction/fade/quiet. Its bucket
// median may be slightly below the opening gate (recorded departure: -81).
#define JOURNEY_PASS_DBM       (-82)
#define JOURNEY_FADE_MIN_MS    2000u
#define JOURNEY_FADE_MAX_PPS   2u
#define JOURNEY_FADE_GAP_MS    2500u
#define JOURNEY_ENDING_WINDOW_MS 10000u
#define JOURNEY_FAST_FADE_MS   10000u
#define JOURNEY_FADE_DROP_DB   10

// Reception timestamps survive the callback queue. Old evidence can update
// location but must never issue a delayed opening command.
#define RADIO_MAX_ACTUATION_AGE_MS 2000u
#define RADIO_EVENTS_PER_LOOP  8u
#define RADIO_QUEUE_SIZE       64u

// Optional corroboration from the existing Classic-BT head unit. A single
// 1.28s inquiry during a weak, stationary beacon session, at most once/10min.
// No response is UNKNOWN, never departure. BLE remains the arrival signal.
#ifndef UCONNECT_HINT_ENABLED
#define UCONNECT_HINT_ENABLED  1
#endif
#define UCONNECT_SETTLE_MS     15000u
#define UCONNECT_INTERVAL_MS   (10u * 60u * 1000u)
#define UCONNECT_SCAN_UNITS    1
#define UCONNECT_TIMEOUT_MS    4000u
#define UCONNECT_NEAR_DBM      (-80)

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
#define SIGNAL_BUCKET_MS      1000u // all states: count, median, range, first/last
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
