// Bluedoor — arrival detection + (v1) remote actuation. One codebase, two
// build targets (platformio.ini):
//
//   logger (PULSE_ENABLED=0): Phase 0.5 one-week "would-open" logger. Full
//     arrival pipeline from PLAN.md (classic BT inquiry + RSSI filter,
//     boot-disarm, away threshold, approach signature, lockout) but the only
//     output is a log: flash file + status web page. No GPIO is ever driven.
//
//   v1 (PULSE_ENABLED=1): production. Same pipeline; a strict verdict can
//     additionally pulse GPIO 26 -> PC817 optocoupler -> sacrificial remote's
//     button (COMPONENTS.md wiring). Run modes DISABLED / DRY-RUN / LIVE,
//     persisted; boots into whatever was set, first boot = DRY-RUN (PLAN.md
//     Phase 2: dry-run week first). Web controls are token-gated.
//
// Verdicts logged:
//   VERDICT WOULD-OPEN (strict)  — the primary rule: absence >= AWAY_MIN, then
//                                  an approach ramp crossing RSSI_TRIGGER_DBM
//   RELAXED would-open           — the no-ramp fallback rule, logged in
//                                  parallel so the week's data can compare them
//   REFUSE wake-in-place         — strong+flat encounter after absence (car
//                                  door opened while parked): latches the whole
//                                  encounter non-fireable
// Pass criteria for the week: every real arrival -> strict WOULD-OPEN,
// zero strict WOULD-OPENs from anything else.

#include <Arduino.h>
#if __has_include("config.h")
#include "config.h"
#else
#include "config.example.h"
#endif
#include "tunables.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <atomic>
#include <time.h>

#include "esp32-hal-bt.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_core_dump.h"
#include "esp_sntp.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif
#ifndef PULSE_ENABLED
#define PULSE_ENABLED 0
#endif
// Phase 0.5 result (2026-08-27): the car's own head unit cannot support arrival
// detection. It answers inquiry only while parked with someone inside — never
// while driving — and reads ~35 dB weaker than a phone in the same seat, so it
// straddles the trigger threshold even parked inside the garage. DETECT_BLE
// swaps the radio layer to watch a BLE beacon carried in the car instead: a
// strong, always-advertising, fixed-MAC signal the ramp logic can actually use.
#ifndef DETECT_BLE
#define DETECT_BLE 0
#endif
#if DETECT_BLE
#include "esp_gap_ble_api.h"
#ifndef BEACON_BLE_MAC          // add the real one to config.h once you own it
#define BEACON_BLE_MAC "AA:BB:CC:DD:EE:FF"
#endif
#define TARGET_MAC BEACON_BLE_MAC
#define TARGET_KIND "beacon"
#else
#define TARGET_MAC CAR_BT_MAC
#define TARGET_KIND "car"
#endif
#if PULSE_ENABLED
#define FW_ROLE "v1"
#else
#define FW_ROLE "logger"
#endif

// ---------------------------------------------------------------- utilities

static const char *LOG_CUR = "/log.txt";
static const char *LOG_OLD = "/log.old.txt";

static Preferences prefs;
static WebServer server(80);

static uint32_t bootCount = 0;
static bool timeSynced = false;
// filled in at boot by logCoreDump() / the abort wrapper, shown on the page
static String coreDumpStatus = "none stored";
static String lastAbortMsg = "none";

// Alternating radio-quiet / WiFi-up windows, logged with the RADIO census so
// the two are directly comparable. Boot always starts in the On phase.
enum class WifiPhase : uint8_t { On, Off };
static WifiPhase wifiPhase = WifiPhase::On;
static uint32_t wifiPhaseSinceMs = 0, lastRadioStatsMs = 0;

// Panic forensics (2 panics on day 1, no USB attached for a backtrace):
// RTC noinit RAM survives crash resets, so the next boot can log where the
// main loop last checkpointed and the lifetime-min free heap. A crash in
// another task still shows loop's last crumb, but paired with min-heap it
// separates OOM-abort from a stack/driver fault.
RTC_NOINIT_ATTR static uint32_t bcMagic;
RTC_NOINIT_ATTR static uint32_t bcStep;
RTC_NOINIT_ATTR static uint32_t bcUptimeS;
RTC_NOINIT_ATTR static uint32_t bcMinHeap;
// Salted with the build timestamp: RTC-noinit variables move between builds,
// and the first boot after an OTA otherwise validates stale bytes from the
// old layout as evidence (seen live at boot #30 — a garbage "abort message"
// logged right after reflashing). A different build's leftovers never match.
static constexpr uint32_t bcMagicHash(const char *s, uint32_t h) {
  return *s ? bcMagicHash(s + 1, h * 31u + (uint8_t)*s) : h;
}
#define BC_MAGIC bcMagicHash(__DATE__ " " __TIME__, 0xB1DE0011u)
// 'ota' used to be the last crumb set before delay(10), so it covered nearly
// all of the loop's wall time: every panic pointed there whatever the cause.
// 'idle' now absorbs the delay, so 'ota' means ArduinoOTA.handle() for real and
// 'idle' means the fault came from another task while the loop was parked.
static const char *BC_NAMES[] = {"boot", "bt-queue", "machine-tick", "scan-tick",
                                 "wifi-tick", "web-client", "ota", "heartbeat",
                                 "idle"};
#define BC_COUNT (sizeof(BC_NAMES) / sizeof(BC_NAMES[0]))
static inline void crumb(uint32_t step) {
  bcStep = step;
  bcUptimeS = millis() / 1000;
  uint32_t mh = ESP.getMinFreeHeap();
  if (mh < bcMinHeap) bcMinHeap = mh;
}

// The core dump named the crashing task ('wifi'), but exccause 0xFFFF with a
// one-entry corrupt backtrace means an abort(), not a CPU fault — and the
// message explaining it went to a serial console nobody is attached to. Every
// IDF abort path (failed assert(), ESP_ERROR_CHECK(), bare abort()) funnels
// through esp_system_abort(details), so wrapping it (-Wl,--wrap in
// platformio.ini) copies that message into RTC-noinit RAM, which survives the
// reset. This runs in panic context: copy the string and nothing else — no
// locks, no heap calls, since the abort may come from inside the allocator.
#define ABORT_MSG_SZ 160
RTC_NOINIT_ATTR static char abortMsg[ABORT_MSG_SZ];
RTC_NOINIT_ATTR static uint32_t abortMagic;
RTC_NOINIT_ATTR static uint32_t abortUptimeS;

extern "C" void __real_esp_system_abort(const char *details) __attribute__((noreturn));
extern "C" void __wrap_esp_system_abort(const char *details) {
  size_t n = 0;
  if (details)
    while (n < ABORT_MSG_SZ - 1 && details[n]) {
      abortMsg[n] = details[n];
      n++;
    }
  abortMsg[n] = 0;
  abortUptimeS = millis() / 1000;
  abortMagic = BC_MAGIC;
  __real_esp_system_abort(details);
}

// The task WDT's panic path should reach the abort wrapper above, but an
// int-wdt reset never calls esp_system_abort at all — it dies in an NMI.
// This weak IDF hook runs first thing in the task-WDT ISR, so after the
// reset-reason split a boot can read the watchdogs unambiguously: task-wdt
// reset + this breadcrumb + an abort message is the expected trio; a
// watchdog reset with neither means the int-wdt (coex/ISR stall) got us.
RTC_NOINIT_ATTR static uint32_t twdtMagic;
RTC_NOINIT_ATTR static uint32_t twdtUptimeS;
extern "C" void esp_task_wdt_isr_user_handler(void) {
  twdtMagic = BC_MAGIC;
  twdtUptimeS = millis() / 1000;
}

static char ring[RING_LINES][RING_LINE_LEN];
static int ringHead = 0, ringCount = 0;

static String tsNow() {
  time_t now = time(nullptr);
  if (now > 1700000000) {
    struct tm t;
    localtime_r(&now, &t);
    char b[24];
    strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &t);
    return String(b);
  }
  char b[20];
  snprintf(b, sizeof(b), "boot+%lus", (unsigned long)(millis() / 1000));
  return String(b);
}

static String fmtDur(uint32_t ms) {
  uint32_t s = ms / 1000;
  char b[24];
  if (s < 60) snprintf(b, sizeof(b), "%us", (unsigned)s);
  else if (s < 3600) snprintf(b, sizeof(b), "%um%02us", (unsigned)(s / 60), (unsigned)(s % 60));
  else snprintf(b, sizeof(b), "%uh%02um", (unsigned)(s / 3600), (unsigned)((s % 3600) / 60));
  return String(b);
}

// blank or placeholder OTA password = OTA never comes up (known credential
// would allow firmware replacement over the LAN)
static bool otaPasswordUsable() {
  return OTA_PASSWORD[0] != '\0' && strcmp(OTA_PASSWORD, "change-me") != 0;
}

static void logEvent(const char *tag, const String &msg) {
  char line[224];
  snprintf(line, sizeof(line), "%s [B%03u] %-7s %s", tsNow().c_str(),
           (unsigned)bootCount, tag, msg.c_str());
  Serial.println(line);

  strlcpy(ring[ringHead], line, RING_LINE_LEN);
  ringHead = (ringHead + 1) % RING_LINES;
  if (ringCount < RING_LINES) ringCount++;

  File f = LittleFS.open(LOG_CUR, FILE_APPEND);
  if (!f) return;
  f.println(line);
  size_t sz = f.size();
  f.close();
  if (sz > LOG_ROTATE_BYTES) {
    LittleFS.remove(LOG_OLD);
    LittleFS.rename(LOG_CUR, LOG_OLD);
    File nf = LittleFS.open(LOG_CUR, FILE_WRITE);
    if (nf) {
      nf.printf("%s [B%03u] LOG     rotated (previous %u bytes in log.old.txt)\n",
                tsNow().c_str(), (unsigned)bootCount, (unsigned)sz);
      nf.close();
    }
  }
}

// ------------------------------------------------------------- BT plumbing

enum class BtEvKind : uint8_t { Sighting, InquiryDone, ProbeResult };

struct BtEv {
  BtEvKind kind;
  int8_t rssi;
  bool ok;
  char name[33];
};

static QueueHandle_t btQueue;
static esp_bd_addr_t carAddr;
static bool carMacIsPlaceholder = false;

// diagnostics updated from the BT callback task
static std::atomic<uint32_t> otherDevReports{0};
static std::atomic<int> otherBestRssi{-127};
static std::atomic<uint32_t> carNoRssiReports{0};

// Radio census, reset each RADIO line. Counts every inquiry response the
// controller delivers — the car and anonymous neighbours alike — so a quiet
// log can be read as "heard nothing" rather than "was not listening".
static std::atomic<uint32_t> radReports{0};
static std::atomic<uint32_t> radCarReports{0};
static std::atomic<int> radBestRssi{-127};
static uint32_t radCycles = 0;   // loop context only

enum class ScanMode : uint8_t { Idle, Inquiry, Probe };
static ScanMode scanMode = ScanMode::Idle;
static uint32_t probeStartMs = 0, lastProbeMs = 0, lastInqDoneMs = 0;
static uint32_t inqCycles = 0;
static bool otaActive = false;

static bool parseMac(const char *s, esp_bd_addr_t out) {
  unsigned b[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
    return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[i];
  return true;
}

static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
      bool hasRssi = false;
      int8_t rssi = 127;
      char name[33] = {0};
      for (int i = 0; i < param->disc_res.num_prop; i++) {
        esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
        if (p->type == ESP_BT_GAP_DEV_PROP_RSSI) {
          rssi = *(int8_t *)p->val;
          hasRssi = true;
        } else if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
          size_t n = p->len < 32 ? p->len : 32;
          memcpy(name, p->val, n);
        } else if (p->type == ESP_BT_GAP_DEV_PROP_EIR && name[0] == 0) {
          uint8_t len = 0;
          uint8_t *d = esp_bt_gap_resolve_eir_data((uint8_t *)p->val,
                                                   ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
          if (!d)
            d = esp_bt_gap_resolve_eir_data((uint8_t *)p->val,
                                            ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
          if (d) {
            size_t n = len < 32 ? len : 32;
            memcpy(name, d, n);
          }
        }
      }
      radReports++;
      if (hasRssi && rssi > radBestRssi) radBestRssi = rssi;
      if (memcmp(param->disc_res.bda, carAddr, 6) == 0) {
        radCarReports++;
        // GAP delivers name-discovery results here too, without an RSSI prop;
        // those must not enter the RSSI pipeline as the +127 init value
        if (!hasRssi) {
          carNoRssiReports++;
          break;
        }
        BtEv ev{BtEvKind::Sighting, rssi, true, {0}};
        strlcpy(ev.name, name, sizeof(ev.name));
        xQueueSend(btQueue, &ev, 0);
      } else {
        // neighbors' devices: counted for diagnostics, never identified/logged
        otherDevReports++;
        if (hasRssi && rssi > otherBestRssi) otherBestRssi = rssi;
      }
      break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
      if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        BtEv ev{BtEvKind::InquiryDone, 0, true, {0}};
        xQueueSend(btQueue, &ev, 0);
      }
      break;
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
      BtEv ev{BtEvKind::ProbeResult, 0,
              param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS, {0}};
      if (ev.ok) strlcpy(ev.name, (const char *)param->read_rmt_name.rmt_name, sizeof(ev.name));
      xQueueSend(btQueue, &ev, 0);
      break;
    }
    default:
      break;
  }
}

#if DETECT_BLE
static void startInquiry() {
  esp_ble_gap_stop_scanning();
  esp_ble_gap_start_scanning(0);
  scanMode = ScanMode::Inquiry;
}
#else
static void startInquiry() {
  esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                             INQ_LEN_UNITS, 0);
  if (err == ESP_OK) {
    scanMode = ScanMode::Inquiry;
  } else {
    scanMode = ScanMode::Idle;
    logEvent("ERR", "start_discovery failed: " + String(esp_err_to_name(err)));
  }
}
#endif  // DETECT_BLE

#if DETECT_BLE
// Passive scan: beacons broadcast, so there is nothing to scan-request. Duplicate
// filtering off — every advertisement is an RSSI sample, and the ramp logic wants
// samples, not unique devices.
static esp_ble_scan_params_t bleScanParams = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,   // 50 ms, in 0.625 ms units
    .scan_window = 0x30,     // 30 ms listening out of each 50
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

#if BLE_SURVEY
// Commissioning aid: the beacon's address is unknown until you own one, so log
// the strongest advertiser seen each interval — hold the beacon against the box
// and its MAC appears in the log. This is the ONE place this firmware records
// somebody else's address, so it is deliberate and opt-out: set BLE_SURVEY to 0
// once BEACON_BLE_MAC is in config.h. (Phones randomise their BLE address.)
static portMUX_TYPE surveyMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t surveyBda[6];
static int surveyRssi = -127;
static char surveyName[32];
static uint32_t lastSurveyMs = 0;
#endif

static void bleGapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      esp_ble_gap_start_scanning(0);   // 0 = until explicitly stopped
      break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) break;
      int8_t rssi = (int8_t)param->scan_rst.rssi;
      char name[33] = {0};
      uint8_t nlen = 0;
      uint8_t *n = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                            ESP_BLE_AD_TYPE_NAME_CMPL, &nlen);
      if (!n) n = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                           ESP_BLE_AD_TYPE_NAME_SHORT, &nlen);
      if (n && nlen) memcpy(name, n, nlen < 32 ? nlen : 32);

      radReports++;
      if (rssi > radBestRssi) radBestRssi = rssi;
      if (memcmp(param->scan_rst.bda, carAddr, 6) == 0) {
        radCarReports++;
        BtEv ev{BtEvKind::Sighting, rssi, true, {0}};
        strlcpy(ev.name, name, sizeof(ev.name));
        xQueueSend(btQueue, &ev, 0);
      } else {
        otherDevReports++;
        if (rssi > otherBestRssi) otherBestRssi = rssi;
      }
#if BLE_SURVEY
      portENTER_CRITICAL(&surveyMux);
      if (rssi > surveyRssi) {
        surveyRssi = rssi;
        memcpy(surveyBda, param->scan_rst.bda, 6);
        strlcpy(surveyName, name, sizeof(surveyName));
      }
      portEXIT_CRITICAL(&surveyMux);
#endif
      break;
    }
    default:
      break;
  }
}

static bool initBt() {
  // BLE-only: hand the classic BT controller's RAM back (tens of KB on a heap
  // that idles near 38 KB) — the car's classic radio is no longer watched.
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  cfg.mode = ESP_BT_MODE_BLE;
  if (esp_bt_controller_init(&cfg) != ESP_OK) return false;
  if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK) return false;
  if (esp_bluedroid_init() != ESP_OK) return false;
  if (esp_bluedroid_enable() != ESP_OK) return false;
  if (esp_ble_gap_register_callback(bleGapCallback) != ESP_OK) return false;
  return esp_ble_gap_set_scan_params(&bleScanParams) == ESP_OK;   // starts the scan
}
#else
static bool initBt() {
  if (!btStart()) return false;
  if (esp_bluedroid_init() != ESP_OK) return false;
  if (esp_bluedroid_enable() != ESP_OK) return false;
  esp_bt_gap_register_callback(gapCallback);
  // scanner only: stay invisible and unconnectable ourselves
  esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
  return true;
}
#endif  // DETECT_BLE

// ---------------------------------------------------------- state machine

enum class SysState : uint8_t { DisarmedBoot, DisarmedSeen, DisarmedLockout, Armed };

static SysState sysState = SysState::DisarmedBoot;
static uint32_t stateSinceMs = 0;
static uint32_t lastEvidenceMs = 0;   // any car evidence: inquiry sighting or page answer
static uint32_t lastSightMs = 0;      // inquiry sightings only (these carry RSSI)
static int8_t lastSightRssi = 127;
static char lastSightName[33] = {0};
static time_t lastSightWall = 0;

// counters (persisted where noted)
static uint32_t totalSightings = 0;
static uint32_t firedCount = 0;       // NVS
static uint32_t relaxedCount = 0;     // NVS
static uint32_t refusalCount = 0;     // NVS
static String lastVerdict = "none yet";

// encounter = sighting cluster while Armed
struct Sight { uint32_t ms; int8_t rssi; };
static struct {
  bool active = false;
  uint32_t startMs = 0, lastMs = 0;
  int8_t firstRssi = 0, lastRssi = 0, minRssi = 0, maxRssi = 0;
  uint16_t count = 0;
  bool relaxedFlagged = false, wakeFlagged = false;
  Sight recent[8];
  uint8_t rIdx = 0;
} enc;

// sighting aggregation while disarmed (keeps the log small when the car
// sits awake in the garage)
static struct {
  bool active = false;
  uint32_t startMs = 0, lastMs = 0;
  uint16_t count = 0;
  int8_t firstRssi = 0, lastRssi = 0, minRssi = 0, maxRssi = 0;
} agg;

// RSSI trajectory label: separates a drive-away (receding) from a
// parked-awake session (steady) and an approach at a glance in the log
static const char *trendWord(int8_t first, int8_t last) {
  if ((int)last - (int)first >= TREND_MIN_DB) return "approaching";
  if ((int)first - (int)last >= TREND_MIN_DB) return "receding";
  return "steady";
}

static const char *stateName(SysState s) {
  switch (s) {
    case SysState::DisarmedBoot: return "DISARMED(boot)";
    case SysState::DisarmedSeen: return "DISARMED(seen)";
    case SysState::DisarmedLockout: return "DISARMED(lockout)";
    case SysState::Armed: return "ARMED";
  }
  return "?";
}

static void setState(SysState s, const String &why) {
  if (s == sysState) return;
  logEvent("STATE", String(stateName(sysState)) + " -> " + stateName(s) + " (" + why + ")");
  sysState = s;
  stateSinceMs = millis();
}

static void flushAgg(const char *why) {
  if (!agg.active) return;
  logEvent("SIGHT", "car visible (" + String(why) + "): " + String(agg.count) +
                        " sightings in " + fmtDur(agg.lastMs - agg.startMs) +
                        ", rssi " + String(agg.firstRssi) + "->" + String(agg.lastRssi) +
                        " (min " + String(agg.minRssi) + " max " + String(agg.maxRssi) +
                        ") " + trendWord(agg.firstRssi, agg.lastRssi));
  agg.active = false;
}

static String encSummary() {
  return String("n=") + enc.count + " dur=" + fmtDur(enc.lastMs - enc.startMs) +
         " rssi first=" + enc.firstRssi + " min=" + enc.minRssi +
         " max=" + enc.maxRssi + " last=" + enc.lastRssi +
         " trend=" + trendWord(enc.firstRssi, enc.lastRssi);
}

static void endEncounter(const String &why) {
  if (!enc.active) return;
  logEvent("ENC", "encounter ended, no verdict (" + why + "): " + encSummary());
  enc.active = false;
}

// ------------------------------------------------------------- actuation
#if PULSE_ENABLED

enum class RunMode : uint8_t { Disabled = 0, DryRun = 1, Live = 2 };
static RunMode runMode = RunMode::DryRun;
static uint32_t pulseCount = 0;  // NVS
static bool pulseActive = false;
static uint32_t pulseOffAtMs = 0;
static String lastPulseInfo = "never";

static const char *modeName(RunMode m) {
  switch (m) {
    case RunMode::Disabled: return "DISABLED";
    case RunMode::DryRun: return "DRY-RUN";
    case RunMode::Live: return "LIVE";
  }
  return "?";
}

static void doPulse(const char *source) {
#if REED_ENABLED
  if (digitalRead(PIN_REED) != LOW) {
    logEvent("PULSE", String("REFUSED (") + source + "): reed says door not closed");
    return;
  }
#endif
  pulseCount++;
  prefs.putUInt("pulses", pulseCount);
  lastPulseInfo = tsNow() + " (" + source + ")";
  logEvent("PULSE", String("*** PULSING REMOTE *** (") + source + ", " +
                        String((unsigned)PULSE_MS) + "ms on GPIO " + String(PIN_PULSE) + ")");
  digitalWrite(PIN_PULSE, HIGH);
  pulseActive = true;
  pulseOffAtMs = millis() + PULSE_MS;
}

static void pulseTick() {
  if (pulseActive && (int32_t)(millis() - pulseOffAtMs) >= 0) {
    digitalWrite(PIN_PULSE, LOW);
    pulseActive = false;
    logEvent("PULSE", "pulse complete, output LOW");
  }
}

static void actuateOnVerdict() {
  switch (runMode) {
    case RunMode::Live:
      doPulse("auto arrival");
      break;
    case RunMode::DryRun:
      logEvent("PULSE", "DRY-RUN: arrival verdict, pulse suppressed (set LIVE via web "
                        "once the dry-run log is clean)");
      break;
    case RunMode::Disabled:
      logEvent("PULSE", "DISABLED: arrival verdict, pulse suppressed");
      break;
  }
}
#endif  // PULSE_ENABLED

static void fireWouldOpen() {
  firedCount++;
  prefs.putUInt("fired", firedCount);
  lastVerdict = tsNow() + "  WOULD-OPEN (strict): " + encSummary();
  logEvent("VERDICT", "*** WOULD OPEN *** (strict rule) " + encSummary());
#if PULSE_ENABLED
  actuateOnVerdict();
#endif
  logEvent("VERDICT", "entering lockout: no further verdicts until a full away period");
  enc.active = false;
  setState(SysState::DisarmedLockout, "would-open fired");
}

static void onCarSighting(int8_t rssi, const char *name) {
  uint32_t now = millis();
  totalSightings++;
  lastEvidenceMs = now;
  lastSightMs = now;
  lastSightRssi = rssi;
  lastSightWall = time(nullptr);
  if (name[0]) strlcpy(lastSightName, name, sizeof(lastSightName));

  if (sysState == SysState::Armed) {
    if (!enc.active) {
      enc.active = true;
      enc.startMs = now;
      enc.firstRssi = enc.minRssi = enc.maxRssi = rssi;
      enc.count = 0;
      enc.relaxedFlagged = enc.wakeFlagged = false;
      enc.rIdx = 0;
      memset(enc.recent, 0, sizeof(enc.recent));
      logEvent("ENC", "encounter started: first sighting after away period, rssi " +
                          String(rssi));
    }
    enc.lastMs = now;
    enc.lastRssi = rssi;
    if (rssi < enc.minRssi) enc.minRssi = rssi;
    if (rssi > enc.maxRssi) enc.maxRssi = rssi;
    if (enc.count < 65535) enc.count++;
    enc.recent[enc.rIdx] = {now, rssi};
    enc.rIdx = (enc.rIdx + 1) % 8;
    logEvent("SIGHT", "armed sighting #" + String(enc.count) + " rssi " + String(rssi));

    // relaxed rule (comparison data only): N recent sightings above threshold
    if (!enc.relaxedFlagged) {
      int hits = 0;
      for (int i = 0; i < 8; i++)
        if (enc.recent[i].ms && now - enc.recent[i].ms <= RELAXED_WINDOW_MS &&
            enc.recent[i].rssi >= RSSI_TRIGGER_DBM)
          hits++;
      if (hits >= RELAXED_MIN_SIGHTINGS) {
        enc.relaxedFlagged = true;
        relaxedCount++;
        prefs.putUInt("relaxed", relaxedCount);
        logEvent("RELAXED", "relaxed (no-ramp) rule would fire here: " + encSummary());
      }
    }

    // wake-in-place: strong from the first sighting, flat ever since. Latches
    // the whole encounter non-fireable — RSSI drift later in a parked-car wake
    // must never become a verdict
    if (!enc.wakeFlagged && enc.count >= 3 && enc.firstRssi >= WAKE_STRONG_DBM &&
        (enc.maxRssi - enc.minRssi) <= WAKE_FLAT_DB) {
      enc.wakeFlagged = true;
      refusalCount++;
      prefs.putUInt("refused", refusalCount);
      logEvent("REFUSE", "wake-in-place signature (strong+flat), encounter latched "
                         "non-fireable: " + encSummary());
    }

    // strict rule: the real verdict (never from a wake-latched encounter)
    if (!enc.wakeFlagged &&
        enc.count >= APPROACH_MIN_SIGHTINGS && enc.lastRssi >= RSSI_TRIGGER_DBM &&
        (enc.maxRssi - enc.firstRssi) >= APPROACH_MIN_RISE_DB &&
        enc.lastRssi > enc.firstRssi) {
      fireWouldOpen();
      return;
    }
  } else {
    // disarmed: aggregate to keep the log compact
    if (!agg.active) {
      logEvent("SIGHT", "car sighted rssi " + String(rssi) +
                            (name[0] ? String(" name='") + name + "'" : String("")) +
                            " [" + stateName(sysState) + "]");
      agg.active = true;
      agg.startMs = now;
      agg.count = 0;
      agg.firstRssi = agg.minRssi = agg.maxRssi = rssi;
    }
    agg.lastMs = now;
    agg.lastRssi = rssi;
    agg.count++;
    if (rssi < agg.minRssi) agg.minRssi = rssi;
    if (rssi > agg.maxRssi) agg.maxRssi = rssi;
    if (now - agg.startMs >= SIGHT_AGG_WINDOW_MS) flushAgg("period");
  }
}

static bool lastProbeOk = false, probeEverRan = false;
static uint32_t lastProbeOkMs = 0;

static void onProbeResult(bool ok, const char *name) {
  uint32_t now = millis();
  if (ok) {
    lastEvidenceMs = now;
    lastProbeOkMs = now;
    if (!probeEverRan || !lastProbeOk)
      logEvent("PROBE", String("car answered page (presence w/o RSSI)") +
                            (name[0] ? String(" name='") + name + "'" : String("")));
    if (sysState == SysState::Armed) {
      // pageable but not answering inquiry — presence only, never a verdict
      logEvent("PROBE", "page answered while ARMED: car present but not seen by "
                        "inquiry — check discoverability in the log");
      endEncounter("page-presence");
      setState(SysState::DisarmedSeen, "car answered page");
    }
  } else {
    if (probeEverRan && lastProbeOk)
      logEvent("PROBE", "car stopped answering pages");
  }
  probeEverRan = true;
  lastProbeOk = ok;
}

static void machineTick() {
  uint32_t now = millis();

  if (enc.active) {
    if (now - enc.lastMs > ENCOUNTER_QUIET_MS) {
      endEncounter("went quiet");
      setState(SysState::DisarmedSeen, "encounter without verdict");
    } else if (now - enc.startMs > ENCOUNTER_MAX_MS) {
      endEncounter("window expired, car lingering");
      setState(SysState::DisarmedSeen, "encounter without verdict");
    }
  }

  // flush early when sightings stop, so departures / head-unit sleep get a
  // timestamp instead of dissolving into the next period flush
  if (agg.active && now - agg.lastMs > SIGHT_AGG_QUIET_MS) flushAgg("went quiet");

  if (sysState != SysState::Armed && now - lastEvidenceMs >= AWAY_MIN_MS) {
    flushAgg("state change");
    setState(SysState::Armed, "no car evidence for " + fmtDur(now - lastEvidenceMs));
  }
}

// -------------------------------------------------------------- scan loop

#if DETECT_BLE
// BLE scanning is continuous, so there is no per-cycle completion event: restart
// it periodically as a liveness watchdog, and keep the RADIO census's "cycles"
// meaningful as scan restarts rather than inquiry sweeps.
static void scanTick() {
  if (otaActive) return;
  uint32_t now = millis();
  if (now - lastInqDoneMs >= BLE_SCAN_RESTART_MS) {
    lastInqDoneMs = now;
    inqCycles++;
    radCycles++;
    startInquiry();
  }
#if BLE_SURVEY
  if (now - lastSurveyMs >= BLE_SURVEY_MS) {
    lastSurveyMs = now;
    uint8_t b[6];
    int r;
    char n[32];
    portENTER_CRITICAL(&surveyMux);
    memcpy(b, surveyBda, 6);
    r = surveyRssi;
    strlcpy(n, surveyName, sizeof(n));
    surveyRssi = -127;
    portEXIT_CRITICAL(&surveyMux);
    if (r > -127) {
      char mac[18];
      snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X", b[0], b[1], b[2], b[3],
               b[4], b[5]);
      logEvent("SURVEY", String("strongest BLE advertiser: ") + mac + " rssi " + String(r) +
                             (n[0] ? String(" '") + n + "'" : String("")));
    }
  }
#endif
}
#else
static void scanTick() {
  if (otaActive) return;
  uint32_t now = millis();

  if (scanMode == ScanMode::Probe && now - probeStartMs > PROBE_TIMEOUT_MS) {
    logEvent("ERR", "probe timed out without result event; resuming inquiry");
    onProbeResult(false, "");
    startInquiry();
    return;
  }
  if (scanMode == ScanMode::Inquiry && now - lastInqDoneMs > SCAN_STUCK_MS) {
    logEvent("ERR", "inquiry watchdog: no completion event, restarting discovery");
    esp_bt_gap_cancel_discovery();
    lastInqDoneMs = now;  // give the cancel a full window before re-triggering
    return;               // DISC_STOPPED event will restart inquiry
  }
  if (scanMode == ScanMode::Idle) startInquiry();
}
#endif  // DETECT_BLE

static void onInquiryDone() {
  uint32_t now = millis();
  inqCycles++;
  radCycles++;
  lastInqDoneMs = now;
  if (otaActive) {
    scanMode = ScanMode::Idle;
    return;
  }
  bool probeDue = (now - lastProbeMs >= PROBE_INTERVAL_MS) &&
                  (now - lastSightMs >= PROBE_SKIP_IF_SEEN_MS) && !carMacIsPlaceholder;
  if (probeDue) {
    lastProbeMs = now;
    probeStartMs = now;
    if (esp_bt_gap_read_remote_name(carAddr) == ESP_OK) {
      scanMode = ScanMode::Probe;
    } else {
      startInquiry();
    }
  } else {
    startInquiry();
  }
}

// -------------------------------------------------------------------- web

static String htmlEscape(const String &s) {
  String o;
  o.reserve(s.length());
  for (char c : s) {
    if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else if (c == '&') o += "&amp;";
    else o += c;
  }
  return o;
}

// Flush the page out in ~1 KB pieces. Building it whole meant a single
// ~20 KB allocation (chrome + 120 escaped tail lines) on a heap that idles at
// ~38 KB free with a largest free block of only ~18 KB — the page was by far
// the biggest thing this firmware ever asked for, and heartbeats show min-heap
// dropping to 15 KB whenever someone loads it. Chunked, it costs ~1 KB.
static void chunk(String &h, bool force = false) {
  if (h.length() >= 1024 || (force && h.length())) {
    server.sendContent(h);
    h = "";
  }
}

static void handleRoot() {
  uint32_t now = millis();
  String h;
  h.reserve(1600);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  h += F("<!doctype html><html><head><meta charset=utf-8>"
         "<meta name=viewport content='width=device-width,initial-scale=1'>"
         "<title>bluedoor 0.5</title><style>"
         "body{font-family:system-ui,sans-serif;margin:1em;background:#111;color:#ddd;max-width:60em}"
         "h1{font-size:1.3em}table{border-collapse:collapse;margin:.5em 0}"
         "td{padding:.15em .6em;border-bottom:1px solid #333}td:first-child{color:#888}"
         ".st{display:inline-block;padding:.2em .6em;border-radius:.3em;font-weight:700}"
         ".armed{background:#173}.dis{background:#555}.live{background:#a33}.dry{background:#a70}"
         "fieldset{border:1px solid #444;margin:.5em 0}form{margin:.3em 0}"
         ".fired{color:#f66;font-weight:700}pre{background:#000;padding:.6em;overflow-x:auto;"
         "font-size:.78em;line-height:1.35}a{color:#7af}input,button{font-size:1em;padding:.3em}"
         "</style></head><body><h1>bluedoor &mdash; " FW_ROLE
#if PULSE_ENABLED
         " (production)"
#else
         " (phase 0.5 would-open logger)"
#endif
         "</h1>");

  bool armed = sysState == SysState::Armed;
  h += "<p><span class='st " + String(armed ? "armed" : "dis") + "'>" + stateName(sysState) +
       "</span> for " + fmtDur(now - stateSinceMs);
#if PULSE_ENABLED
  h += " &nbsp; mode <span class='st " +
       String(runMode == RunMode::Live ? "live" : (runMode == RunMode::DryRun ? "dry" : "dis")) +
       "'>" + modeName(runMode) + "</span>";
  if (pulseActive) h += " &nbsp; <b class=fired>PULSING NOW</b>";
#endif
  h += "</p>";
#if PULSE_ENABLED
  if (strcmp(CONTROL_TOKEN, "change-me-too") == 0)
    h += F("<p class=fired>⚠ CONTROL_TOKEN is still the placeholder — web controls "
           "are refused until config.h gets a real token.</p>");
#endif

  if (carMacIsPlaceholder)
    h += F("<p class=fired>⚠ config.h still has the placeholder car MAC — "
           "this build can never see the car!</p>");
  if (!otaPasswordUsable())
    h += F("<p class=fired>⚠ OTA_PASSWORD is blank or the placeholder — OTA is "
           "disabled until config.h gets a real one (USB reflash only).</p>");

  h += F("<table>");
  h += "<tr><td>would-open (strict)</td><td class=fired>" + String(firedCount) + "</td></tr>";
#if PULSE_ENABLED
  h += "<tr><td>pulses fired</td><td class=fired>" + String(pulseCount) + " (last: " +
       htmlEscape(lastPulseInfo) + ")</td></tr>";
  h += "<tr><td>door sensor</td><td>";
#if REED_ENABLED
  h += String("reed on GPIO ") + PIN_REED + ": door " +
       (digitalRead(PIN_REED) == LOW ? "CLOSED" : "NOT closed");
#else
  h += "not fitted (v1 — reed interlock is the v2 upgrade)";
#endif
  h += "</td></tr>";
#endif
  h += "<tr><td>relaxed rule would have fired</td><td>" + String(relaxedCount) + "</td></tr>";
  h += "<tr><td>wake-in-place refusals</td><td>" + String(refusalCount) + "</td></tr>";
  h += "<tr><td>last verdict</td><td>" + htmlEscape(lastVerdict) + "</td></tr>";
  h += "<tr><td>car last sighted</td><td>";
  if (lastSightMs == 0 && lastSightRssi == 127) {
    h += "never (this boot)";
  } else {
    h += fmtDur(now - lastSightMs) + " ago, rssi " + String(lastSightRssi);
    if (lastSightName[0]) h += String(" ('") + lastSightName + "')";
  }
  h += "</td></tr>";
  h += "<tr><td>sightings (this boot)</td><td>" + String(totalSightings);
  if (uint32_t noR = carNoRssiReports.load())
    h += " (+" + String((unsigned)noR) + " car reports w/o RSSI, ignored)";
  h += "</td></tr>";
  h += "<tr><td>page probe</td><td>";
  if (!probeEverRan) h += "not run yet";
  else h += String(lastProbeOk ? "answering" : "no answer") +
            (lastProbeOkMs ? " (last ok " + fmtDur(now - lastProbeOkMs) + " ago)" : "");
  h += "</td></tr>";
  h += "<tr><td>inquiry cycles</td><td>" + String(inqCycles) + " (other-device reports: " +
       String((unsigned)otherDevReports.load()) + ")</td></tr>";
#if WIFI_DUTY_TEST
  h += "<tr><td>coexistence test</td><td>WiFi up; going quiet for " +
       fmtDur(WIFI_TEST_OFF_MS) + " in " +
       fmtDur(WIFI_TEST_ON_MS - (now - wifiPhaseSinceMs)) +
       " (detection keeps running; page returns after)</td></tr>";
#endif
  h += "<tr><td>boot / uptime</td><td>#" + String(bootCount) + " / " + fmtDur(now) +
       (timeSynced ? "" : " — clock not NTP-synced yet") + "</td></tr>";
  h += "<tr><td>wifi / heap</td><td>" + String(WiFi.RSSI()) + " dBm / " +
       String(ESP.getFreeHeap() / 1024) + " KB free</td></tr>";
  h += "<tr><td>fw</td><td>" FW_VERSION " built " __DATE__ " " __TIME__ "</td></tr>";
  h += "<tr><td>last crash</td><td>" + htmlEscape(coreDumpStatus) + "</td></tr>";
  h += "<tr><td>abort message</td><td>" + htmlEscape(lastAbortMsg) + "</td></tr>";
  h += "<tr><td>tunables</td><td>away=" + String(AWAY_MIN_MS / 60000) +
       "min trigger=" + String(RSSI_TRIGGER_DBM) + "dBm ramp&ge;" +
       String(APPROACH_MIN_RISE_DB) + "dB sightings&ge;" + String(APPROACH_MIN_SIGHTINGS) +
       " inq=" + String(INQ_LEN_UNITS * 128 / 100) + "." + String(INQ_LEN_UNITS * 128 % 100) +
       "s</td></tr>";
  h += F("</table>");

#if PULSE_ENABLED
  h += F("<fieldset><legend>controls (token required)</legend>"
         "<form method=POST action=/mode>token <input type=password name=token size=10> "
         "<select name=mode>");
  h += String("<option value=disabled") + (runMode == RunMode::Disabled ? " selected" : "") +
       ">DISABLED</option><option value=dryrun" + (runMode == RunMode::DryRun ? " selected" : "") +
       ">DRY-RUN</option><option value=live" + (runMode == RunMode::Live ? " selected" : "") +
       ">LIVE</option>";
  h += F("</select> <button>set mode</button></form>"
         "<form method=POST action=/pulse onsubmit=\"return confirm('Pulse the remote — "
         "the door WILL move. Sure?')\">token <input type=password name=token size=10> "
         "<button>manual pulse (commissioning)</button></form></fieldset>");
#endif
  h += F("<form method=POST action=/mark><input name=note placeholder='e.g. real arrival now' "
         "maxlength=80> <button>Mark in log</button></form>"
         "<p><a href=/log>download log</a> &middot; <a href=/log.old>previous log</a> &middot; "
         "<a href=/tail>plain tail</a></p><pre>");
  chunk(h, true);
  for (int i = 0; i < ringCount; i++) {
    int idx = (ringHead - 1 - i + RING_LINES) % RING_LINES;
    h += htmlEscape(ring[idx]);
    h += '\n';
    chunk(h);
  }
  // auto-refresh, but never while someone is typing in a form field
  h += F("</pre><script>var ff=document.querySelectorAll('input');"
         "setInterval(function(){for(var k=0;k<ff.length;k++){var e=ff[k];"
         "if(document.activeElement===e||e.value)return;}location.reload();},10000);"
         "</script></body></html>");
  chunk(h, true);
  server.sendContent("");   // terminates the chunked response
}

static void handleMark() {
  String note = server.arg("note");
  String clean;
  for (char c : note)
    if (isPrintable(c) && clean.length() < 80) clean += c;
  logEvent("MARK", clean.length() ? clean : String("(unlabeled mark)"));
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleLogFile(const char *path) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "no such log yet");
    return;
  }
  server.streamFile(f, "text/plain");
  f.close();
}

static void handleTail() {
  String o;
  o.reserve(1600);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/plain", "");
  for (int i = 0; i < ringCount; i++) {
    int idx = (ringHead - ringCount + i + RING_LINES) % RING_LINES;
    o += ring[idx];
    o += '\n';
    chunk(o);
  }
  chunk(o, true);
  server.sendContent("");
}

#if PULSE_ENABLED
static bool tokenOk() {
  if (strcmp(CONTROL_TOKEN, "change-me-too") == 0) {
    logEvent("ERR", "control refused: CONTROL_TOKEN still placeholder in config.h");
    return false;
  }
  if (server.arg("token") != CONTROL_TOKEN) {
    logEvent("ERR", "control refused: bad token from " +
                        server.client().remoteIP().toString());
    return false;
  }
  return true;
}

static void handleMode() {
  if (!tokenOk()) {
    server.send(403, "text/plain", "bad token");
    return;
  }
  String m = server.arg("mode");
  RunMode nm;
  if (m == "disabled") nm = RunMode::Disabled;
  else if (m == "dryrun") nm = RunMode::DryRun;
  else if (m == "live") nm = RunMode::Live;
  else {
    server.send(400, "text/plain", "mode?");
    return;
  }
  if (nm != runMode) {
    logEvent("MODE", String("run mode ") + modeName(runMode) + " -> " + modeName(nm) +
                         " (web, from " + server.client().remoteIP().toString() + ")");
    runMode = nm;
    prefs.putUChar("mode", (uint8_t)nm);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleManualPulse() {
  if (!tokenOk()) {
    server.send(403, "text/plain", "bad token");
    return;
  }
  if (runMode == RunMode::Disabled) {
    logEvent("PULSE", "manual pulse refused: mode is DISABLED");
  } else {
    logEvent("PULSE", "manual pulse requested via web (commissioning, from " +
                          server.client().remoteIP().toString() + ")");
    doPulse("manual/web");
    // door state is unknown after any pulse — same lockout as an auto fire
    endEncounter("manual pulse");
    setState(SysState::DisarmedLockout, "manual pulse");
  }
  server.sendHeader("Location", "/");
  server.send(303);
}
#endif  // PULSE_ENABLED

// ------------------------------------------------------------ wifi & ota

static bool wifiWasConnected = false, otaReady = false, mdnsReady = false;
static uint32_t lastWifiKickMs = 0;

// wifiTick's polled edge sees "down" but never why. The WiFi event task
// records the driver's reason code here (no logEvent — that is loop-only)
// and counts every drop, including churn faster than the 500ms poll.
static volatile uint8_t wifiLastDiscReason = 0;
static std::atomic<uint32_t> wifiDropCount{0};

// names for the codes that decide between the stability hypotheses; anything
// else stays numeric and looks up in esp_wifi_types.h
static const char *wifiDiscReasonStr(uint8_t r) {
  switch (r) {
    case WIFI_REASON_AUTH_EXPIRE: return " auth-expire";
    case WIFI_REASON_ASSOC_LEAVE: return " assoc-leave";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return " 4way-handshake-timeout";
    case WIFI_REASON_BEACON_TIMEOUT: return " beacon-timeout(weak link)";
    case WIFI_REASON_NO_AP_FOUND: return " no-ap-found";
    case WIFI_REASON_AUTH_FAIL: return " auth-fail";
    case WIFI_REASON_ASSOC_FAIL: return " assoc-fail";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return " handshake-timeout";
    case WIFI_REASON_CONNECTION_FAIL: return " connection-fail";
    default: return "";
  }
}

// Heap-pressure hypothesis, caught in the act instead of inferred: the heap
// calls this on any failed malloc, from whatever task made it — counters
// only here, the heartbeat does the reporting.
static std::atomic<uint32_t> allocFailCount{0};
static std::atomic<uint32_t> allocFailLastSize{0};
static void onAllocFailed(size_t size, uint32_t caps, const char *fn) {
  (void)caps; (void)fn;
  allocFailCount++;
  allocFailLastSize = (uint32_t)size;
}

// every web hit, counted at registration below (plus 404s via onNotFound) —
// correlates page traffic with resets and heap dips
static std::atomic<uint32_t> httpReqCount{0};

// Set from the SNTP task; logEvent() stays loop-only, so the loop does the
// logging. bootWall* snapshot the clock the reset carried over, which lets the
// first real sync report how far off it was.
static volatile bool ntpFired = false;
static int64_t bootWallSec = 0;
static uint32_t bootWallMs = 0;
static void onNtpSync(struct timeval *tv) { (void)tv; ntpFired = true; }

static void radioStats(const char *why) {
  uint32_t reports = radReports.exchange(0);
  uint32_t carRep = radCarReports.exchange(0);
  int best = radBestRssi.exchange(-127);
  uint32_t cyc = radCycles;
  radCycles = 0;
  lastRadioStatsMs = millis();
  logEvent("RADIO", String(why) + ": wifi=" +
                        (wifiPhase == WifiPhase::Off ? "off" : "on") +
                        " cycles=" + String(cyc) + " reports=" + String(reports) +
                        " car=" + String(carRep) + " best=" +
                        (best > -127 ? String(best) + "dBm" : String("nothing heard")) +
                        " temp=" + String((int)temperatureRead()) + "C");
}

static void setupOtaOnce() {
  if (otaReady || !otaPasswordUsable()) return;
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    otaActive = true;
    esp_bt_gap_cancel_discovery();
    logEvent("BOOT", "OTA update starting");
  });
  ArduinoOTA.onError([](ota_error_t e) {
    otaActive = false;
    logEvent("ERR", "OTA failed: " + String((int)e));
    scanMode = ScanMode::Idle;  // scanTick restarts inquiry
  });
  ArduinoOTA.begin();
  otaReady = true;
}

#if WIFI_DUTY_TEST
static void wifiPhaseTick() {
  uint32_t now = millis();
  if (wifiPhase == WifiPhase::On && now - wifiPhaseSinceMs >= WIFI_TEST_ON_MS) {
    radioStats("wifi-up window ended");
    if (otaReady) {
      ArduinoOTA.end();
      otaReady = false;
    }
    if (mdnsReady) {
      MDNS.end();
      mdnsReady = false;
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiWasConnected = false;
    wifiPhase = WifiPhase::Off;
    wifiPhaseSinceMs = now;
    logEvent("RADIO", "coexistence test: WiFi OFF for " + fmtDur(WIFI_TEST_OFF_MS) +
                          " — detection continues, page/OTA unreachable until it returns");
  } else if (wifiPhase == WifiPhase::Off && now - wifiPhaseSinceMs >= WIFI_TEST_OFF_MS) {
    radioStats("radio-quiet window ended");
    wifiPhase = WifiPhase::On;
    wifiPhaseSinceMs = now;
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(DEVICE_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    logEvent("RADIO", "coexistence test: WiFi back ON");
  }
}
#endif

static void wifiTick() {
#if WIFI_DUTY_TEST
  wifiPhaseTick();
  if (wifiPhase == WifiPhase::Off) {
    if (millis() - lastRadioStatsMs >= RADIO_STATS_MS) radioStats("periodic");
    return;   // deliberately down: no reconnect kicks, no NTP
  }
#endif
  bool up = WiFi.status() == WL_CONNECTED;
  uint32_t now = millis();
  if (now - lastRadioStatsMs >= RADIO_STATS_MS) radioStats("periodic");
  if (up && !wifiWasConnected) {
    logEvent("WIFI", "connected, ip " + WiFi.localIP().toString() + " rssi " +
                         String(WiFi.RSSI()));
    configTzTime(TZ_INFO, NTP_SERVER);
    if (!mdnsReady && MDNS.begin(DEVICE_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      mdnsReady = true;
    }
    setupOtaOnce();
  } else if (!up && wifiWasConnected) {
    logEvent("WIFI", "disconnected, reason " + String((unsigned)wifiLastDiscReason) +
                         wifiDiscReasonStr(wifiLastDiscReason) +
                         " — detection continues, page/NTP unavailable");
  }
  wifiWasConnected = up;

  if (!up && now - lastWifiKickMs > 30000) {
    lastWifiKickMs = now;
    WiFi.reconnect();
  }

  // A warm reset (panic, watchdog, OTA) leaves the RTC running, so
  // time(nullptr) is already plausible inside setup() and the old check fired
  // before WiFi was even up: every crash boot logged an "NTP synced" line
  // carrying the pre-reset clock, which had drifted minutes during the run
  // that crashed. Wait for a real SNTP update, and report the correction so
  // the timestamps on the crash-boot lines above can be read with it.
  if (!timeSynced && ntpFired) {
    timeSynced = true;
    String extra;
    if (bootWallSec) {
      int64_t expect = bootWallSec + (int64_t)((millis() - bootWallMs) / 1000);
      long skew = (long)((int64_t)time(nullptr) - expect);
      extra = ", clock carried over the reset was off by " + String(skew) + "s";
    }
    logEvent("TIME", String("NTP synced: ") + tsNow() +
                         (bootWallSec ? "" : " (earlier stamps are boot-relative)") + extra);
  }
}

// ------------------------------------------------------------------ setup

static uint32_t lastHeartbeatMs = 0, lastTickMs = 0;
static uint32_t hbSightings = 0, hbCycles = 0, hbOther = 0;

// The Arduino SDK ships with core-dump-to-flash enabled and min_spiffs.csv
// already carries a 64 KB coredump partition, so every panic since day one has
// written a full ELF dump there — nothing ever read it back. Summarise it into
// the log at boot: crashing task, exception cause and the backtrace PCs, which
// decode offline against the firmware.elf of the build that crashed:
//   xtensa-esp32-elf-addr2line -pfiaC -e firmware.elf <pc> <pc> ...
// The raw dump is deliberately NOT served over HTTP: it is a RAM image and
// would hand out WiFi/OTA credentials to anyone on the LAN.
static void logCoreDump() {
  size_t addr = 0, size = 0;
  if (esp_core_dump_image_get(&addr, &size) != ESP_OK || size == 0) return;
  coreDumpStatus = String((unsigned)size) + " byte dump in flash";
  if (esp_core_dump_image_check() != ESP_OK) {
    coreDumpStatus += " (bad checksum)";
    logEvent("ERR", "core dump present (" + String((unsigned)size) +
                        " bytes) but failed its checksum");
    return;
  }
  bool reported = false;
  esp_core_dump_summary_t *sum =
      (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
  if (!sum) return;
  if (esp_core_dump_get_summary(sum) == ESP_OK) {
    String bt;
    for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16; i++)
      bt += (i ? " 0x" : "0x") + String(sum->exc_bt_info.bt[i], HEX);
    coreDumpStatus = String("task '") + sum->exc_task + "' pc=0x" +
                     String(sum->exc_pc, HEX) + " cause=" +
                     String(sum->ex_info.exc_cause);
    reported = true;
    logEvent("ERR", String("core dump: task '") + sum->exc_task + "' pc=0x" +
                        String(sum->exc_pc, HEX) + " exccause=" +
                        String(sum->ex_info.exc_cause) + " excvaddr=0x" +
                        String(sum->ex_info.exc_vaddr, HEX) +
                        (sum->exc_bt_info.corrupted ? " (backtrace corrupt)" : "") +
                        " backtrace: " + bt);
  }
  free(sum);
  // erase once reported, so a "core dump:" line always means a fresh crash
  // rather than the same old dump re-announced at every boot
  if (reported && esp_core_dump_image_erase() != ESP_OK)
    logEvent("ERR", "core dump erase failed — the next boot will re-report this same dump");
}

static const char *resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    // The three watchdogs indict different suspects, so never merge them:
    // int-wdt = interrupts stayed off >300ms (ISR/critical section — the
    // classic WiFi+BT coexistence failure), task-wdt = IDLE0 starved >5s by
    // a runaway task, rtc-wdt = a hang before the app even ran.
    case ESP_RST_INT_WDT: return "int-wdt";
    case ESP_RST_TASK_WDT: return "task-wdt";
    case ESP_RST_WDT: return "rtc-wdt";
    case ESP_RST_BROWNOUT: return "brownout";
    default: return "other";
  }
}

void setup() {
#if PULSE_ENABLED
  // Opto drive LOW before anything else runs; the 10k hardware pulldown
  // covers the boot-ROM window before this line (COMPONENTS.md — NOT optional).
  digitalWrite(PIN_PULSE, LOW);
  pinMode(PIN_PULSE, OUTPUT);
  digitalWrite(PIN_PULSE, LOW);
#if REED_ENABLED
  pinMode(PIN_REED, INPUT_PULLUP);
#endif
#endif
  Serial.begin(115200);
  delay(100);

  // TZ from the first timestamp: system time survives soft resets (panic,
  // OTA), and without this those boots stamp UTC until WiFi re-syncs
  setenv("TZ", TZ_INFO, 1);
  tzset();
  if (time(nullptr) > 1700000000) {   // a warm reset carried the clock over
    bootWallSec = (int64_t)time(nullptr);
    bootWallMs = millis();
  }
  sntp_set_time_sync_notification_cb(onNtpSync);

  prefs.begin("bluedoor", false);
  bootCount = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", bootCount);
  firedCount = prefs.getUInt("fired", 0);
  relaxedCount = prefs.getUInt("relaxed", 0);
  refusalCount = prefs.getUInt("refused", 0);
#if PULSE_ENABLED
  pulseCount = prefs.getUInt("pulses", 0);
  runMode = (RunMode)prefs.getUChar("mode", (uint8_t)RunMode::DryRun);
  if (runMode > RunMode::Live) runMode = RunMode::DryRun;
#endif

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed!");

  logEvent("BOOT", String("bluedoor ") + FW_VERSION + " (" + FW_ROLE + ") built " + __DATE__ +
                       " " + __TIME__ + ", reset: " + resetReasonStr() + ", boot #" +
                       String(bootCount));
  // Watchdog resets carry the same breadcrumbs a panic does — the crumb says
  // what loop was doing (or that it was parked in 'idle') when the WDT hit.
  esp_reset_reason_t bootRR = esp_reset_reason();
  bool crashBoot = bootRR == ESP_RST_PANIC || bootRR == ESP_RST_INT_WDT ||
                   bootRR == ESP_RST_TASK_WDT || bootRR == ESP_RST_WDT;
  if (crashBoot && bcMagic == BC_MAGIC)
    logEvent("ERR", "crash forensics: last loop checkpoint '" +
                        String(bcStep < BC_COUNT ? BC_NAMES[bcStep] : "?") + "' at uptime " +
                        String(bcUptimeS) + "s, min free heap " +
                        String(bcMinHeap / 1024) + "K");
  if (abortMagic == BC_MAGIC) {
    abortMsg[ABORT_MSG_SZ - 1] = 0;
    lastAbortMsg = String(abortMsg);
    logEvent("ERR", "abort message from the crash: " + lastAbortMsg + " (at uptime " +
                        String(abortUptimeS) + "s)");
  }
  abortMagic = 0;   // never report a stale message against a later reset
  if (twdtMagic == BC_MAGIC)
    logEvent("ERR", "task watchdog fired at uptime " + String(twdtUptimeS) +
                        "s in the run before this boot");
  twdtMagic = 0;
  logCoreDump();
  bcMagic = BC_MAGIC;
  bcStep = 0;
  bcUptimeS = 0;
  bcMinHeap = UINT32_MAX;
#if PULSE_ENABLED
  logEvent("MODE", String("run mode: ") + modeName(runMode) +
                       " (persisted; first-ever boot defaults to DRY-RUN)");
#endif
  logEvent("STATE", String("boot-disarmed: first sighting after boot is never an arrival; "
                           "arming needs ") + fmtDur(AWAY_MIN_MS) + " with no car evidence");

  if (!parseMac(TARGET_MAC, carAddr)) {
    carMacIsPlaceholder = true;
    logEvent("ERR", TARGET_KIND " MAC unparseable — fix config.h");
  } else if (strcasecmp(TARGET_MAC, "AA:BB:CC:DD:EE:FF") == 0) {
    carMacIsPlaceholder = true;
    logEvent("ERR", "config.h has the placeholder " TARGET_KIND " MAC — fill it in "
                    "(see config.example.h)");
  }
  if (strcmp(WIFI_SSID, "your-wifi-ssid") == 0)
    logEvent("ERR", "config.h has placeholder WiFi credentials — web page will not come up");
  if (!otaPasswordUsable())
    logEvent("ERR", "OTA_PASSWORD blank or placeholder in config.h — OTA stays disabled, "
                    "reflash over USB only");

  btQueue = xQueueCreate(32, sizeof(BtEv));
  stateSinceMs = lastEvidenceMs = lastInqDoneMs = lastProbeMs = millis();
  wifiPhaseSinceMs = lastRadioStatsMs = millis();

  if (initBt()) {
    logEvent("BT", String("classic BT up, watching ") +
                       (carMacIsPlaceholder ? "(placeholder MAC!)" : TARGET_MAC) +
                       ", inquiry cycle " + String(INQ_LEN_UNITS * 1.28f, 2) + "s continuous");
    startInquiry();
  } else {
    logEvent("ERR", "BT init failed — logger is blind! check build/config");
  }

  heap_caps_register_failed_alloc_callback(onAllocFailed);

  // registered before begin() so even the very first association failure
  // leaves its reason code
  WiFi.onEvent(
      [](WiFiEvent_t, WiFiEventInfo_t info) {
        wifiLastDiscReason = info.wifi_sta_disconnected.reason;
        wifiDropCount++;
      },
      ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  server.on("/", HTTP_GET, []() { httpReqCount++; handleRoot(); });
  server.on("/mark", HTTP_POST, []() { httpReqCount++; handleMark(); });
#if PULSE_ENABLED
  server.on("/mode", HTTP_POST, []() { httpReqCount++; handleMode(); });
  server.on("/pulse", HTTP_POST, []() { httpReqCount++; handleManualPulse(); });
#endif
  server.on("/log", HTTP_GET, []() { httpReqCount++; handleLogFile(LOG_CUR); });
  server.on("/log.old", HTTP_GET, []() { httpReqCount++; handleLogFile(LOG_OLD); });
  server.on("/tail", HTTP_GET, []() { httpReqCount++; handleTail(); });
  server.onNotFound([]() {
    httpReqCount++;
    server.send(404, "text/plain", "not found");
  });
  server.begin();

  lastHeartbeatMs = millis();
}

void loop() {
  BtEv ev;
  crumb(1);
  while (xQueueReceive(btQueue, &ev, 0) == pdTRUE) {
    switch (ev.kind) {
      case BtEvKind::Sighting:
        hbSightings++;
        onCarSighting(ev.rssi, ev.name);
        break;
      case BtEvKind::InquiryDone:
        hbCycles++;
        onInquiryDone();
        break;
      case BtEvKind::ProbeResult:
        onProbeResult(ev.ok, ev.name);
        if (scanMode == ScanMode::Probe) startInquiry();
        break;
    }
  }

  uint32_t now = millis();
#if PULSE_ENABLED
  pulseTick();
#endif
  if (now - lastTickMs >= 500) {
    lastTickMs = now;
    crumb(2);
    machineTick();
    crumb(3);
    scanTick();
    crumb(4);
    wifiTick();
    if (agg.active && now - agg.startMs >= SIGHT_AGG_WINDOW_MS) flushAgg("period");
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    crumb(7);
    uint32_t other = otherDevReports.exchange(0);
    uint32_t http = httpReqCount.exchange(0);
    uint32_t wdrop = wifiDropCount.exchange(0);
    uint32_t afail = allocFailCount.load();   // cumulative: rare and alarming
    logEvent("HB", String(stateName(sysState)) + " sightings=" + String(hbSightings) +
                       " cycles=" + String(hbCycles) + " otherdevs=" + String(other) +
                       " heap=" + String(ESP.getFreeHeap() / 1024) + "K min=" +
                       String(ESP.getMinFreeHeap() / 1024) + "K big=" +
                       String(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024) +
                       "K wifi=" +
                       String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) +
                       // die temp (internal sensor: offset unreliable, trend real)
                       // — the board came back from the garage noticeably hot
                       " temp=" + String((int)temperatureRead()) + "C" +
                       " http=" + String(http) + " wifidrop=" + String(wdrop) +
                       (afail ? " ALLOCFAIL=" + String(afail) + " last=" +
                                    String(allocFailLastSize.load()) + "b"
                              : ""));
    hbSightings = hbCycles = 0;
  }

  crumb(5);
  server.handleClient();
  crumb(6);
  if (otaReady) ArduinoOTA.handle();
  crumb(8);
  delay(10);
}
