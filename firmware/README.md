# Bluedoor firmware

One codebase ([src/main.cpp](src/main.cpp)), two build targets
([platformio.ini](platformio.ini)):

| Target | What it is | Output |
|---|---|---|
| `logger` | **Phase 0.5** one-week "would-open" logger. Full arrival pipeline from [PLAN.md](../PLAN.md), but the only output is the log — no GPIO is ever driven, nothing is wired. | verdicts in the log only |
| `v1` | **Production.** Same pipeline; a strict arrival verdict can additionally pulse GPIO 26 → PC817 optocoupler → the sacrificial remote's button ([COMPONENTS.md](../COMPONENTS.md) wiring). | log + real button press |

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
spot):

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
- Full log: `/log` (rotates at 80 KB into `/log.old`); download both when
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
| `REFUSE` | Wake-in-place signature (strong+flat after absence) — correctly not fired. |
| `ENC` | Encounter (sighting cluster while armed) started/ended, with RSSI stats. |
| `SIGHT` | Car sightings; aggregated per minute while disarmed. |
| `PROBE` | Targeted page to the car — presence without RSSI, never fires anything. |
| `STATE` | DISARMED(boot/seen/lockout) ↔ ARMED transitions with reasons. |
| `HB` | 30-min heartbeat (liveness); gap = crash/power loss. |
| `MARK` | Your ground-truth annotations. |
| `ERR` | Init problems, watchdog restarts, refused control attempts (with source IP). |

## Design notes

- Classic BT (BR/EDR) inquiry via ESP-IDF GAP, RSSI from
  `ESP_BT_GAP_DISC_RES_EVT`, continuous cycles. BLE is useless here — the car
  is classic-only (Phase 0).
- The ESP32 itself is non-connectable and non-discoverable; other devices'
  MACs are never logged (privacy) — just anonymous counts.
- WiFi + classic BT share the radio (coexistence enabled in the stock Arduino
  core); a slightly sluggish web page while inquiry runs is normal. Detection
  never depends on WiFi.
- Partition scheme `min_spiffs`: OTA stays possible, ~190 KB LittleFS holds
  ~2 weeks of logs across the two rotation files.
- Security posture (PLAN.md): convenience-lock grade. Controls are LAN-only +
  token; a spoofed MAC still has to fake an approach ramp after a real away
  period. Physical remotes and key remain the fallback.
