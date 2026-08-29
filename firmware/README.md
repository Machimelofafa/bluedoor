# Bluedoor firmware

One codebase ([src/main.cpp](src/main.cpp)), two build targets
([platformio.ini](platformio.ini)):

| Target | What it is | Output |
|---|---|---|
| `logger` | **Phase 0.5** one-week "would-open" logger. Full arrival pipeline from [PLAN.md](../PLAN.md), but the only output is the log — no GPIO is ever driven, nothing is wired. | verdicts in the log only |
| `v1` | **Production.** Same pipeline; a strict arrival verdict can additionally pulse GPIO 26 → PC817 optocoupler → the sacrificial remote's button ([COMPONENTS.md](../COMPONENTS.md) wiring). | log + real button press |
| `logger-ble` | Same logger, but the radio layer watches a **BLE beacon carried in the car** instead of the car's own head unit. Needs `BEACON_BLE_MAC` in config.h. | verdicts in the log only |

### Why `logger-ble` exists (Phase 0.5 result, 2026-08-27)

The car's own Bluetooth cannot drive arrival detection, and no tuning fixes it:

- It answers inquiry **only while parked with someone inside** — never while
  driving. A controlled test (a phone riding in the car as a positive control)
  logged 523 phone responses and **zero** from the car across a drive out and
  back; the car last answered three minutes before the car pulled out.
- When it does answer it reads **~35 dB weaker than a phone in the same seat**
  (−75 dBm vs −40 dBm), so it straddles the −80 dBm trigger even parked inside
  the garage. No headroom is left to detect an approach.
- So the car is visible only in the state that must *never* fire
  (wake-in-place) and invisible in the one that must (arrival) — an exact
  inversion. The `REFUSE` guard at −60 dBm never fired and, at these levels,
  never can.

A beacon replaces it with a strong, always-advertising, fixed-address signal
the existing ramp logic can use — and because a beacon stays visible while the
car is home, "away ≥ 10 min" becomes meaningful and wake-in-place stops being
a special case.

Board: Freenove ESP32-WROOM (FNK0090).

## Before flashing

1. Edit [include/config.h](include/config.h) (gitignored — car MAC, WiFi
   credentials and tokens never enter git): fill in `WIFI_SSID`, `WIFI_PASS`,
   `OTA_PASSWORD`, and for v1 a real `CONTROL_TOKEN`. The car MAC is already
   set. (Fresh checkout? Copy `config.example.h` to `config.h` first.)
2. Detection thresholds and actuation parameters live in
   [include/tunables.h](include/tunables.h) — start values are hypotheses,
   tuned from logger-week data.

## Build / flash / watch

```bash
pio run -e logger              # build the Phase 0.5 logger (also plain `pio run`)
```
```bash
pio run -e v1                  # build production v1
```
```bash
pio run -e logger -t upload    # flash over USB (auto-detects the port)
```
```bash
pio device monitor             # serial log at 115200
```

After the first USB flash, reflash over the air without touching the device
(this is how the logger box becomes the v1 box without leaving its mounting
spot). OTA only comes up with a real `OTA_PASSWORD` in config.h — blank or
the placeholder keeps it disabled:

```bash
pio run -e v1 -t upload --upload-port bluedoor.local
```

If USB upload fails with a permission error, add yourself to `dialout`
(`sudo usermod -aG dialout $USER`, relog).

## Phase 0.5: using the logger week

- Status page: **http://bluedoor.local** (auto-refreshes; state, counters,
  last sighting, tunables, event tail).
- **Mark button**: when you actually arrive by car (or test wake-in-place by
  opening the parked car's door), tap "Mark in log" with a note — ground
  truth to correlate against verdicts.
- Full log: `/log` (rotates at 48 KB into `/log.old`); download both when
  visiting. Serial mirrors everything. WiFi loss never stops detection.
- Every boot starts DISARMED until a 10-min car-free period; verdict counters
  persist across reboots (NVS).

**Pass criteria (PLAN.md):** every real arrival (`MARK`) has a matching
strict `VERDICT`; zero strict `VERDICT`s without one — wake-in-place must
show as `REFUSE`, reboots covered by boot-disarm, neighbors by the MAC
filter. Only then buy Stage 2.

## v1: architecture

```
BT sighting ─▶ state machine ─▶ strict verdict ─▶ run-mode gate ─▶ pulse 250 ms
                    │                                  │              GPIO 26 ─▶ PC817 ─▶ remote
                    │                             DISABLED / DRY-RUN: log only
                    └─ post-pulse lockout: no re-fire until a full away period
```

- **Run modes**, persisted in NVS, switchable on the web page (token
  required): `DISABLED` (never pulses, even manually), `DRY-RUN` (default on
  first boot — verdicts log "pulse suppressed"), `LIVE`. A reboot never
  escalates the mode.
- **Boot safety:** GPIO 26 is non-strapping and driven LOW first thing in
  `setup()`; the 10 kΩ hardware pulldown covers the boot-ROM window. A
  boot/reset/flash cycle can never press the button.
- **Manual pulse** (commissioning): token-gated button on the page, refused
  in DISABLED mode. Any pulse — auto or manual — enters the same lockout,
  since the door state is unknown afterwards.
- **Reed interlock (v2)** is compiled in but stubbed: `REED_ENABLED 0` in
  tunables.h. Fitting the sensor later = wire GPIO 27, flip to 1, rebuild.
- Everything the logger logs, v1 still logs (PLAN.md: "log everything").

### Commissioning sequence (maps to PLAN.md Phases 1–2)

1. Solder per COMPONENTS.md (new remote enrolled first — old one is the
   programming key), flash `v1`.
2. Device boots in **DRY-RUN**. Supervised session: use **manual pulse** to
   confirm the door opens on command.
3. Leave in DRY-RUN for several days; check every `DRY-RUN: arrival verdict`
   line against reality.
4. Only when the dry-run log is clean, set **LIVE** from the web page.

### Hypotheses baked in (update after the logger week / commissioning)

| # | Hypothesis | Where | Falsified if… |
|---|---|---|---|
| 1 | Strict ramp rule (≥3 sightings, ≥6 dB rise, crossing −80 dBm) catches every real arrival | `tunables.h` | logger week shows real arrivals with `RELAXED` but no strict `VERDICT` → relax ramp, lean on interlocks (PLAN.md) |
| 2 | −80 dBm is the right proximity threshold from the chosen mounting spot | `RSSI_TRIGGER_DBM` | logged approach RSSI curves peak lower/higher |
| 3 | 250 ms reads as one clean button press on the MITTO 12V-UP | `PULSE_MS` | commissioning: door ignores it (lengthen) or double-triggers (shorten) |
| 4 | 5.12 s inquiry cycles are fast enough to catch a driving approach | `INQ_LEN_UNITS` | arrivals appear as 1–2 sightings only → shorten cycles |
| 5 | Lockout-until-full-away is an acceptable re-arm policy | state machine | legitimate same-hour second arrivals get eaten → add time-based cooldown path |
| 6 | The sleeping head unit does NOT answer page probes (so probes can't guard wake-in-place) | `PROBE_*` | logger shows `PROBE` answers while parked → add page-presence guard before arming |

## Reading the log

| Tag | Meaning |
|---|---|
| `VERDICT WOULD OPEN (strict)` | The firing rule fired. In v1 this is followed by a `PULSE` line (LIVE) or a suppression line (DRY-RUN/DISABLED). |
| `PULSE` | v1: actual/suppressed/refused/manual pulses, and pulse completion. |
| `MODE` | v1: run-mode changes (web) and the mode restored at boot. |
| `RELAXED` | The no-ramp fallback rule would have fired here (comparison data). |
| `REFUSE` | Wake-in-place signature (strong+flat after absence) — the whole encounter is latched non-fireable. |
| `ENC` | Encounter (sighting cluster while armed) started/ended, with RSSI stats and a `trend=` label (approaching / receding / steady). |
| `SIGHT` | Car sightings; aggregated per minute while disarmed, each line with first→last RSSI and a trend label — `receding` = drove away, `steady` = parked awake (e.g. someone opened the car), `approaching` = drove up. A `went quiet` flush timestamps the moment sightings stopped. |
| `PROBE` | Targeted page to the car — presence without RSSI, never fires anything. |
| `STATE` | DISARMED(boot/seen/lockout) ↔ ARMED transitions with reasons. |
| `HB` | 30-min heartbeat (liveness); gap = crash/power loss. Includes `big=` (largest free block) — allocation failures are about fragmentation, which free heap alone hides — plus `http=` (web requests served this interval), `wifidrop=` (WiFi disconnects this interval), and `ALLOCFAIL=` (cumulative failed mallocs with the last requested size — should never appear). |
| `RADIO` | 5-min census of what the inquiry radio actually heard: completed cycles, every device report (car and anonymous neighbours), and the best RSSI of anything. Reads a quiet log as "heard nothing" rather than "was not listening". Also marks the WiFi/BT coexistence test windows (`WIFI_DUTY_TEST` in tunables.h): WiFi alternates 10 min up / 20 min quiet, so the two can be compared inside one run. Every boot starts with WiFi up, so OTA is always reachable within 10 min of a reset. |
| `MARK` | Your ground-truth annotations. |
| `ERR` | Init problems, watchdog restarts, refused control attempts (with source IP). Boot-time crash forensics also land here: the `BOOT` line's reset reason distinguishes `int-wdt` (ISR/critical-section stall >300ms — the classic WiFi+BT coexistence signature) from `task-wdt` (IDLE0 starved >5s) and `rtc-wdt`; a task-wdt reset additionally logs its RTC-RAM breadcrumb and an abort message, so a watchdog reset with *neither* points at the int-wdt/coex path. |

## Design notes

- **Stability verdict (overnight soak, 2026-08-28):** continuous classic
  inquiry + WiFi crashes the prebuilt BT controller every 1–3.5 h — a
  controller assert (`ASSERT_PARAM rwbt.c:393`) and repeated int-wdt stalls
  inside its level-4 interrupt (`hli_vectors.S`), with the app loop innocent
  every time (heap 28–32 K min, temp flat, http=0). Desk conditions (strong
  WiFi, cool, clean power) reproduced it, ruling out environment. Retiring
  continuous inquiry is the fix; the BLE build does exactly that. Note: the
  classic-RAM release is NOT possible on Arduino 2.0.17's prebuilt BTDM libs
  (`esp_bt_controller_init` rejects both a BLE cfg.mode and a post-release
  default cfg), so the BLE build runs the controller dual-mode with classic
  simply unused. **Confirmed 2026-08-29: the BLE build ran 25 h clean under
  identical desk conditions** (vs. 4 crashes in 9.5 h for classic), heap
  fragmentation flat (`big=` 27 K throughout, vs. 18–19 K on classic) —
  retiring inquiry fixed it.
- Classic BT (BR/EDR) inquiry via ESP-IDF GAP, RSSI from
  `ESP_BT_GAP_DISC_RES_EVT`, continuous cycles. BLE is useless here — the car
  is classic-only (Phase 0).
- The ESP32 itself is non-connectable and non-discoverable; other devices'
  MACs are never logged (privacy) — just anonymous counts.
- WiFi + classic BT share the radio (coexistence enabled in the stock Arduino
  core); a slightly sluggish web page while inquiry runs is normal. Detection
  never depends on WiFi.
- Partition scheme `min_spiffs`: OTA stays possible, a 128 KB LittleFS holds
  ~2 weeks of logs across the two rotation files.
- Security posture (PLAN.md): convenience-lock grade. Controls are LAN-only +
  token; a spoofed MAC still has to fake an approach ramp after a real away
  period. Physical remotes and key remain the fallback.
