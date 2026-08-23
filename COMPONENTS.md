# Bluedoor — electronics bill of materials & wiring map

Companion to [PLAN.md](PLAN.md). Board: Freenove ESP32-WROOM (FNK0090),
ordered 2026-08-22. **Build target: minimal v1 — ESP32 + old remote only, no
door sensor** (decision 2026-08-22; reed interlock deferred to v2).

## Wiring map — v1

```
                         ESP32 (Freenove FNK0090)
                         ────────────────────────
  GPIO 26 ──[R1 330 Ω]──▶ PC817 pin 1 (LED anode)
                          PC817 pin 2 (LED cathode) ──▶ GND
  GPIO 26 ──[R2 10 kΩ]──▶ GND   (pulldown: a boot/reset can never
                                 press the button — NOT optional)

                          PC817 pin 4 (collector) ──▶ remote button pad (+)
                          PC817 pin 3 (emitter)  ──▶ remote button pad (−)
                          (measure pad polarity FIRST — output is directional)

  5V/USB  ◀── phone charger + USB cable (owned)
```

GPIO 26 is a non-strapping pin — safe on boot. Avoid 0, 2, 5, 12, 15.

## v1 shopping list (everything needed to solder)

| # | Part | Qty | Notes | ~€ |
|---|---|---|---|---|
| 1 | PC817 optocoupler, DIP-4 | 2 | 1 + spare; EL817/LTV-817 equivalents fine | 1 |
| 2 | Resistor 330 Ω, 1/4 W | 2 | opto LED series (≈6 mA from 3.3 V GPIO); 220–470 Ω all fine | — |
| 3 | Resistor 10 kΩ, 1/4 W | 2 | GPIO pulldown + spare | — |
| 4 | Thin stranded hookup wire, AWG 26–28 | ~1 m | the two flying leads to the remote's button pads | 2 |
| 5 | Heat-shrink tubing, small sizes | few pcs | insulate the pad leads | 1 |
| 6 | Dupont jumper wires (M-F) | small set | connect opto/resistors to the ESP32 headers without soldering the board | 2 |
| 7 | Any small box | 1 | houses ESP32 + remote; drill a hole for the USB cable | 0–3 |

Total: **under €10.** Resistors: cheapest as a small E12 assortment box (~€6)
if none on hand. No perfboard needed for v1 — opto + two resistors can be
soldered "dead-bug" onto the flying leads, heat-shrunk, and jumpered to the
ESP32's female headers. Never solder the ESP32 board itself.

## Tools (check what you already have)

| Item | Why |
|---|---|
| Soldering iron with a fine tip + 0.8 mm rosin-core solder | the remote's pads are small |
| Flux pen & desoldering braid | clean joints; braid is mistake insurance on the remote PCB |
| **Multimeter** | **mandatory before soldering** — measure button-pad voltage, polarity, current |
| Wire strippers, tweezers, tape | general assembly |
| Isopropyl alcohol | clean flux residue off the 15-year-old remote board |

## v2 upgrade kit — OPTIONAL, only if v1's no-sensor behavior annoys

Buy nothing here unless the door regularly gets closed-on-arrival because it
was left open (see PLAN.md architecture note).

| Part | Qty | Notes | ~€ |
|---|---|---|---|
| Wired magnetic door contact (alarm type, screw terminals) | 1 | closed when magnet adjacent → door-closed = GPIO LOW | 3–5 |
| 2-core alarm/bell wire | 5–10 m | reed → ESP32 run; length per mounting spot | 3 |
| 100 nF ceramic capacitor | 2 | noise filter at the ESP32 end | — |
| 10 kΩ resistor (pullup) | 1 | reed input (GPIO 27, kept stubbed in firmware) | — |
| Small toggle switch (physical arm/disable) | 1 | optional nicety | 1 |
| Perfboard ~5×7 cm + female header strips | 1+2 | tidier permanent assembly | 2 |

## Not on this list (tracked in PLAN.md)

- Freenove ESP32-WROOM board — ordered 2026-08-22
- BFT MITTO COOL C2 remote — Stage 2, after the logger week (becomes daily
  carry; the old MITTO 12V-UP gets the soldering)
- Old remote's spare battery — type TBD, note it when the remote is opened
- 5 V USB charger + cable — owned

## Order of operations reminder (from PLAN.md)

1. ESP32 arrives → logger firmware, no soldering yet.
2. Logger week decides the mounting spot (power + BT reach + remote-RF reach).
3. New remote bought & enrolled (old one is the programming key) → only then
   open and solder the old remote.
4. Jumper-wire everything first; make it permanent once proven.
