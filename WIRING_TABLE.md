# RPT Winch Tension Subsystem — Wiring Table

**Board:** Arduino Mega 2560
**Subsystem:** Cable-tension loop (load cell → HX711 → Arduino → MD25HV → winch motor)
**Logic level:** 5 V (HX711 powered from Arduino 5 V; all logic is 5 V)

Pin assignments are taken directly from the firmware
(`mastercontrolfile_quarter_powered_DCTS` / `WinchTensionPID_test`):
`Winch_PWM = D3`, `Winch_DIR = D29`, `DT_PIN = D17`, `SCK_PIN = D18`.

---

## 1. Load Cell (QLLG-25) → HX711

The QLLG-25 is a 4-wire full-bridge load cell (+ shield). Colors are from the
sensor's own wiring diagram.

| Load cell wire | Color | Connects to (HX711 pad) | Notes |
|----------------|-------|-------------------------|-------|
| EXC+ | Red   | **E+** | Bridge excitation + |
| EXC− | Black | **E−** | Bridge excitation − |
| SIG+ | Green | **A+** | Signal +, channel A |
| SIG− | White | **A−** | Signal −, channel A |
| Shield | Bare/Drain | **GND** (HX711) or chassis ground | Cable shield; ground at one end only to avoid ground loops |

> Channel A is used because the firmware calls `scale.set_gain(32)`, which is a
> channel-A gain setting on the HX711. Wire the bridge to **A+/A−**, not B+/B−.

---

## 2. HX711 → Arduino Mega

| HX711 pin | Connects to (Arduino) | Firmware name | Notes |
|-----------|------------------------|---------------|-------|
| VCC | **5V** | — | 5 V logic/supply (chosen for best signal headroom on the Mega) |
| GND | **GND** | — | Common ground with Arduino |
| DT (DOUT) | **D17** | `DT_PIN` | Serial data out from HX711 |
| SCK (PD_SCK) | **D18** | `SCK_PIN` | Serial clock from Arduino |

> The HX711's analog supply (its load-cell bridge excitation) comes from the same
> VCC, so a clean/quiet 5 V improves tension resolution. Keep load-cell wires away
> from the motor/power wiring.

---

## 3. MD25HV Winch Driver — Logic side → Arduino Mega

The MD25HV control bank is `GND / 5Vo / PWM / DIR` (per the supplied wiring diagram).

| MD25HV logic pin | Connects to (Arduino) | Firmware name | Notes |
|------------------|------------------------|---------------|-------|
| PWM | **D3** | `Winch_PWM` | Speed input. Driven by Timer 5 (CTC ISR toggles D3); duty = `OCR5B` |
| DIR | **D29** | `Winch_DIR` | Direction input. HIGH = "too-tight" correction, LOW = "too-loose" (per DCTS/PID convention) |
| GND | **GND** | — | **Required** common ground between driver logic and Arduino |
| 5Vo | *(leave unconnected, or use only as a 5 V reference)* | — | This is a **5 V OUTPUT from the driver** — do **NOT** wire Arduino 5 V into it. Only connect if you intend to power external logic from the driver |

> ⚠️ The single most important wire here is **GND**: the Arduino and the MD25HV
> must share a common ground or the PWM/DIR signals have no reference and the
> winch will behave erratically.

---

## 4. MD25HV Winch Driver — Power / Motor side

This is the high-current side. Keep it physically separate from the load-cell wiring.

| MD25HV power pin | Connects to | Notes |
|------------------|-------------|-------|
| VB+ | **Motor supply +** (battery/PSU positive) | Main power for the winch motor. Match the motor's rated voltage; the HV in MD25HV denotes the high-voltage variant — verify your pack voltage is within the driver's rating |
| VB− | **Motor supply −** (battery/PSU negative) | Power ground. Tie to system ground (same reference as Arduino GND) |
| MA | **Winch motor terminal A** | One motor lead |
| MB | **Winch motor terminal B** | Other motor lead. Swap MA/MB if the winch spools the wrong way |

> The winch is a **brushed DC motor** driven by the MD25HV (an H-bridge PWM+DIR
> controller). The "AC motor" label was a mix-up — the MD25HV cannot drive AC, and
> your firmware's PWM+DIR scheme confirms DC. No code change needed.

---

## 5. Grounding & power summary

| Net | Members |
|-----|---------|
| **Common GND** (single reference) | Arduino GND, HX711 GND, MD25HV logic GND, MD25HV VB− / motor supply − |
| **5 V logic** | Arduino 5V → HX711 VCC |
| **Motor power** | Motor PSU+ → MD25HV VB+ ; Motor PSU− → MD25HV VB− (and into common GND) |

**Wiring order to avoid mistakes**
1. Establish the common ground star first (all GNDs to one point).
2. Power the HX711 from Arduino 5 V; wire the load-cell bridge (red/black/green/white + shield).
3. Wire MD25HV logic: D3→PWM, D29→DIR, Arduino GND→driver GND.
4. Wire MD25HV power last: VB+/VB− to the motor PSU, MA/MB to the winch motor.

---

## Pins still to confirm on hardware

- **Motor supply voltage** vs. MD25HV-HV rating and winch motor rating — not specified in code; set from your actual hardware.
- **Shield grounding point** — ground the load-cell shield at one end only (recommended: the HX711/Arduino end) to prevent ground loops.
- **MA/MB polarity** — determines spool direction; swap if the winch tightens when it should release.
