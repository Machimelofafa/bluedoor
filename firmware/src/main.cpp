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
//                                  door opened while parked): must NOT fire
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

#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif
#ifndef PULSE_ENABLED
#define PULSE_ENABLED 0
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
      int8_t rssi = 127;
      char name[33] = {0};
      for (int i = 0; i < param->disc_res.num_prop; i++) {
        esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
        if (p->type == ESP_BT_GAP_DEV_PROP_RSSI) {
          rssi = *(int8_t *)p->val;
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
      if (memcmp(param->disc_res.bda, carAddr, 6) == 0) {
        BtEv ev{BtEvKind::Sighting, rssi, true, {0}};
        strlcpy(ev.name, name, sizeof(ev.name));
        xQueueSend(btQueue, &ev, 0);
      } else {
        // neighbors' devices: counted for diagnostics, never identified/logged
        otherDevReports++;
        if (rssi != 127 && rssi > otherBestRssi) otherBestRssi = rssi;
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

static bool initBt() {
  if (!btStart()) return false;
  if (esp_bluedroid_init() != ESP_OK) return false;
  if (esp_bluedroid_enable() != ESP_OK) return false;
  esp_bt_gap_register_callback(gapCallback);
  // scanner only: stay invisible and unconnectable ourselves
  esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
  return true;
}

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
  uint32_t startMs = 0;
  uint16_t count = 0;
  int8_t minRssi = 0, maxRssi = 0;
} agg;

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
                        " sightings in " + fmtDur(millis() - agg.startMs) +
                        ", rssi " + String(agg.minRssi) + ".." + String(agg.maxRssi));
  agg.active = false;
}

static String encSummary() {
  return String("n=") + enc.count + " dur=" + fmtDur(enc.lastMs - enc.startMs) +
         " rssi first=" + enc.firstRssi + " min=" + enc.minRssi +
         " max=" + enc.maxRssi + " last=" + enc.lastRssi;
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

    // strict rule: the real verdict
    if (enc.count >= APPROACH_MIN_SIGHTINGS && enc.lastRssi >= RSSI_TRIGGER_DBM &&
        (enc.maxRssi - enc.firstRssi) >= APPROACH_MIN_RISE_DB &&
        enc.lastRssi > enc.firstRssi) {
      fireWouldOpen();
      return;
    }

    // wake-in-place: strong from the first sighting, flat ever since
    if (!enc.wakeFlagged && enc.count >= 3 && enc.firstRssi >= WAKE_STRONG_DBM &&
        (enc.maxRssi - enc.minRssi) <= WAKE_FLAT_DB) {
      enc.wakeFlagged = true;
      refusalCount++;
      prefs.putUInt("refused", refusalCount);
      logEvent("REFUSE", "wake-in-place signature (strong+flat), NOT a would-open: " +
                             encSummary());
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
      agg.minRssi = agg.maxRssi = rssi;
    }
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

  if (sysState != SysState::Armed && now - lastEvidenceMs >= AWAY_MIN_MS) {
    flushAgg("state change");
    setState(SysState::Armed, "no car evidence for " + fmtDur(now - lastEvidenceMs));
  }
}

// -------------------------------------------------------------- scan loop

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

static void onInquiryDone() {
  uint32_t now = millis();
  inqCycles++;
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

static void handleRoot() {
  uint32_t now = millis();
  String h;
  h.reserve(9000);
  h += F("<!doctype html><html><head><meta charset=utf-8>"
         "<meta http-equiv=refresh content=10>"
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
  h += "<tr><td>sightings (this boot)</td><td>" + String(totalSightings) + "</td></tr>";
  h += "<tr><td>page probe</td><td>";
  if (!probeEverRan) h += "not run yet";
  else h += String(lastProbeOk ? "answering" : "no answer") +
            (lastProbeOkMs ? " (last ok " + fmtDur(now - lastProbeOkMs) + " ago)" : "");
  h += "</td></tr>";
  h += "<tr><td>inquiry cycles</td><td>" + String(inqCycles) + " (other-device reports: " +
       String((unsigned)otherDevReports.load()) + ")</td></tr>";
  h += "<tr><td>boot / uptime</td><td>#" + String(bootCount) + " / " + fmtDur(now) +
       (timeSynced ? "" : " — clock not NTP-synced yet") + "</td></tr>";
  h += "<tr><td>wifi / heap</td><td>" + String(WiFi.RSSI()) + " dBm / " +
       String(ESP.getFreeHeap() / 1024) + " KB free</td></tr>";
  h += "<tr><td>fw</td><td>" FW_VERSION " built " __DATE__ " " __TIME__ "</td></tr>";
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
  for (int i = 0; i < ringCount; i++) {
    int idx = (ringHead - 1 - i + RING_LINES) % RING_LINES;
    h += htmlEscape(ring[idx]);
    h += '\n';
  }
  h += F("</pre></body></html>");
  server.send(200, "text/html", h);
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
  o.reserve(ringCount * RING_LINE_LEN / 2);
  for (int i = 0; i < ringCount; i++) {
    int idx = (ringHead - ringCount + i + RING_LINES) % RING_LINES;
    o += ring[idx];
    o += '\n';
  }
  server.send(200, "text/plain", o);
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

static void setupOtaOnce() {
  if (otaReady) return;
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
  if (strlen(OTA_PASSWORD)) ArduinoOTA.setPassword(OTA_PASSWORD);
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

static void wifiTick() {
  bool up = WiFi.status() == WL_CONNECTED;
  uint32_t now = millis();
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
    logEvent("WIFI", "disconnected — detection continues, page/NTP unavailable");
  }
  wifiWasConnected = up;

  if (!up && now - lastWifiKickMs > 30000) {
    lastWifiKickMs = now;
    WiFi.reconnect();
  }

  if (!timeSynced && time(nullptr) > 1700000000) {
    timeSynced = true;
    logEvent("TIME", "NTP synced: " + tsNow() + " (earlier stamps are boot-relative)");
  }
}

// ------------------------------------------------------------------ setup

static uint32_t lastHeartbeatMs = 0, lastTickMs = 0;
static uint32_t hbSightings = 0, hbCycles = 0, hbOther = 0;

static const char *resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
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
#if PULSE_ENABLED
  logEvent("MODE", String("run mode: ") + modeName(runMode) +
                       " (persisted; first-ever boot defaults to DRY-RUN)");
#endif
  logEvent("STATE", String("boot-disarmed: first sighting after boot is never an arrival; "
                           "arming needs ") + fmtDur(AWAY_MIN_MS) + " with no car evidence");

  if (!parseMac(CAR_BT_MAC, carAddr)) {
    carMacIsPlaceholder = true;
    logEvent("ERR", "CAR_BT_MAC unparseable — fix config.h");
  } else if (strcasecmp(CAR_BT_MAC, "AA:BB:CC:DD:EE:FF") == 0) {
    carMacIsPlaceholder = true;
    logEvent("ERR", "config.h has the placeholder car MAC — copy config.example.h to "
                    "config.h and fill it in");
  }
  if (strcmp(WIFI_SSID, "your-wifi-ssid") == 0)
    logEvent("ERR", "config.h has placeholder WiFi credentials — web page will not come up");

  btQueue = xQueueCreate(32, sizeof(BtEv));
  stateSinceMs = lastEvidenceMs = lastInqDoneMs = lastProbeMs = millis();

  if (initBt()) {
    logEvent("BT", String("classic BT up, watching ") +
                       (carMacIsPlaceholder ? "(placeholder MAC!)" : CAR_BT_MAC) +
                       ", inquiry cycle " + String(INQ_LEN_UNITS * 1.28f, 2) + "s continuous");
    startInquiry();
  } else {
    logEvent("ERR", "BT init failed — logger is blind! check build/config");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/mark", HTTP_POST, handleMark);
#if PULSE_ENABLED
  server.on("/mode", HTTP_POST, handleMode);
  server.on("/pulse", HTTP_POST, handleManualPulse);
#endif
  server.on("/log", HTTP_GET, []() { handleLogFile(LOG_CUR); });
  server.on("/log.old", HTTP_GET, []() { handleLogFile(LOG_OLD); });
  server.on("/tail", HTTP_GET, handleTail);
  server.begin();

  lastHeartbeatMs = millis();
}

void loop() {
  BtEv ev;
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
    machineTick();
    scanTick();
    wifiTick();
    if (agg.active && now - agg.startMs >= SIGHT_AGG_WINDOW_MS) flushAgg("period");
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    uint32_t other = otherDevReports.exchange(0);
    logEvent("HB", String(stateName(sysState)) + " sightings=" + String(hbSightings) +
                       " cycles=" + String(hbCycles) + " otherdevs=" + String(other) +
                       " heap=" + String(ESP.getFreeHeap() / 1024) + "K wifi=" +
                       String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0));
    hbSightings = hbCycles = 0;
  }

  server.handleClient();
  if (otaReady) ArduinoOTA.handle();
  delay(10);
}
