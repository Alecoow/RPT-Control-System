# RPT Control System — Full Code Documentation

**Project:** Robotic Palm Trimmer (RPT) Control System
**Platform:** Arduino Mega 2560 (both the on-machine controller and the handheld remote)
**Documented files:**

| File | Role | Version header |
|------|------|----------------|
| `mastercontrolfile_quarter_powered_DCTS_04_28_2026.ino` | On-machine "master" controller — runs all motors, reads sensors, executes the tension system | v1.1, 4-28-2026 |
| `WirelessController_pot_inc_4_28_2026.ino` | Handheld remote transmitter — reads buttons/joysticks/pot, drives an LCD, sends packets | v1.1, 4-28-2026 |

> **Scope note:** This document only *describes* the existing code. No code is modified. Line numbers refer to the source files as uploaded. Where the code's own comments are factually wrong (e.g. timer prescaler values), this document states the actual hardware behavior and flags the discrepancy.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [How the Two Boards Talk (the RF link)](#2-how-the-two-boards-talk-the-rf-link)
3. [The Shared `ControllerPacket` Structure](#3-the-shared-controllerpacket-structure)
4. [Master Control File — Detailed Walkthrough](#4-master-control-file--detailed-walkthrough)
5. [The Dynamic Cable Tension System (DCTS)](#5-the-dynamic-cable-tension-system-dcts)
6. [The Timer / PWM Subsystem](#6-the-timer--pwm-subsystem)
7. [Wireless Controller File — Detailed Walkthrough](#7-wireless-controller-file--detailed-walkthrough)
8. [Glossary — Master Control File](#8-glossary--master-control-file)
9. [Glossary — Wireless Controller File](#9-glossary--wireless-controller-file)
10. [Known Issues, Quirks, and Things to Verify](#10-known-issues-quirks-and-things-to-verify)

---

## 1. System Overview

The RPT is a machine that climbs/positions itself on a palm tree and trims it. There are **two separate Arduino Mega boards**:

- **The machine ("master control")** sits on the robot. It drives every motor, reads the load cell and ultrasonic sensor, and runs the automatic tension logic. It is the **receiver (RX)** on the radio link.
- **The remote ("wireless controller")** is held by the operator. It reads joysticks, buttons, and a potentiometer, shows status on a 16×2 LCD, and is the **transmitter (TX)** on the radio link.

The operator's inputs are bundled into a single struct (`ControllerPacket`) and sent over a 2.4 GHz nRF24L01 radio. The machine acts on that packet every loop. The machine sends two bytes back (current load and current range) piggybacked on the radio's automatic acknowledgement ("ack payload"), which the remote displays on its LCD.

There are **five actuator groups** on the machine:

| Actuator | Purpose | Driven by |
|----------|---------|-----------|
| **Winch** ("tension") | Pulls/releases the cable to set cable tension | Timer 5 PWM + `Winch_DIR` |
| **Circumferential motor** | Moves the trimmer around the trunk | Timer 4 PWM + `Circumferential_DIR` |
| **Traction wheels** (×3) | Climb/descend the trunk | Timer 3 PWM + 3 direction pins |
| **Stepper ("radial")** | Moves the chainsaw in/out toward the trunk | Timer 1 toggle ISR + `Stepper_DIR` |
| **Chainsaw** | Cutting tool | Single on/off enable pin |

Two sensors feed the machine:

| Sensor | Measures | Read by |
|--------|----------|---------|
| **HX711 + S-type load cell** | Cable tension (as a raw ADC count called "quanta") | `scale.read()` |
| **HC-SR04-style ultrasonic** | Radial distance to the trunk (cm) | `ultraSonic()` via `pulseIn` |

The **tension system** is the only closed-loop behavior in the current code. It is named **DCTS** (Dynamic Cable Tension System) and lives in the `DCTS()` function. Everything else is open-loop manual control driven directly by the operator's joysticks.

---

## 2. How the Two Boards Talk (the RF link)

Both boards use the **nRF24L01** radio (the `RF24` library) configured identically so they can pair:

| Setting | Value (both boards) | Where (master / remote) |
|---------|---------------------|--------------------------|
| Data rate | `RF24_250KBPS` | master L176 / remote L116 |
| Power amplifier level | `RF24_PA_LOW` | master L177 / remote L117 |
| Channel | `76` | master L178 / remote L121 |
| Address (5 bytes) | `{'T','E','S','T','1'}` | master L111 / remote L70 |
| Dynamic payloads | enabled | master L180 / remote L118 |
| Ack payloads | enabled | master L181 / remote L119 |
| Auto-ack | `true` | master L182 / remote L120 |

The master calls `openReadingPipe(1, address)` and `startListening()` (it receives). The remote calls `openWritingPipe(address)` (it transmits). Both use **CE pin 49** and **CSN pin 53**.

**The acknowledgement (ack) payload** is the clever part of the link: when the remote sends a packet, the master's radio hardware automatically replies with a 2-byte array (`ackArr`). The master loads that array with `ackArr[0] = load (lbs)` and `ackArr[1] = range (cm)`. The remote reads it back inside `SendPacket()` and displays it on the LCD. This gives two-way data over a one-way `write()` call.

> ⚠️ **3.3 V warning** (remote L68): the nRF24L01+PA+LNA module runs on **3.3 V**, not 5 V. Connecting it to 5 V destroys it.

---

## 3. The Shared `ControllerPacket` Structure

Both files define the **same** struct so the bytes line up on each end. (Master L90–102, remote L75–87.)

```cpp
struct ControllerPacket {
  char    buttonID;      // which button event this packet represents ('1'..'8', '0' = none, '9' = idle default)
  bool    b_EMERGENCY;   // emergency flag
  uint8_t winch_spd;     // tension/winch speed   (from LEFT joystick X)
  bool    winch_dir;     // tension/winch direction
  uint8_t traction_spd;  // traction speed        (from LEFT joystick Y)
  bool    traction_dir;  // traction direction
  uint8_t circum_spd;    // circumferential speed (from RIGHT joystick X)
  bool    circum_dir;    // circumferential direction
  uint8_t radial_spd;    // radial/stepper speed  (from RIGHT joystick Y)
  bool    radial_dir;    // radial direction
  uint8_t tensionSet;    // operator's desired tension setpoint (lbs), chosen via potentiometer
};
```

**Critical mapping to understand:** the field names do *not* always match the physical control that feeds them. The remote's `#define` comments claim one mapping (X=tension, Y=elevator), but the **actual call arguments are swapped** (see [section 7.5](#75-function-by-function) and [Issue #9](#10-known-issues-quirks-and-things-to-verify)). The mapping below reflects what the code *actually does* given the swapped argument order in `ReadJoystick()`:

- `winch_spd` / `winch_dir` ← **Left joystick Y (vertical)** → controls cable tension (the winch). In the master, this is the *manual* tension command. *(The `#define` comment says X drives this, but the code passes Y.)*
- `traction_spd` / `traction_dir` ← **Left joystick X (horizontal)** → controls the traction (climbing) wheels.
- `circum_spd` / `circum_dir` ← **Right joystick Y (vertical)** → circumferential motor.
- `radial_spd` / `radial_dir` ← **Right joystick X (horizontal)** → radial stepper (chainsaw in/out).
- `tensionSet` ← **potentiometer**, set via the LCD "Set Load" flow. Default value `20`.

> ⚠️ **Verify on hardware.** Because the argument order is swapped relative to the `#define` comments, the *physical* axis that moves each motor may not be what the labels suggest. The mapping above is derived strictly from the code's argument order; confirm against the real joysticks.

The remote initializes the packet as `{ '9', 0, 0, 0, 0, 0, 0, 0, 0, 0, 20 }` (remote L89): buttonID `'9'`, no emergency, all speeds/dirs zero, `tensionSet = 20`.

---

## 4. Master Control File — Detailed Walkthrough

File: `mastercontrolfile_quarter_powered_DCTS_04_28_2026.ino`

### 4.1 Header & includes (L1–8)

Version 1.1. The header comment summarizes the v1.1 change: *"Changed DCTS equations and limits to operate with 1/4th the power. Now allows sending of range data to controller. Allow for greater auto tension values from controller."* Includes: `SPI.h`, `nRF24L01.h`, `RF24.h` (radio), `HX711.h` (load cell amplifier), `printf.h` (lets the RF24 library pretty-print its config over Serial).

### 4.2 Pin definitions (L10–49)

All pins are fixed with `#define`. See the [pin map glossary](#8-glossary--master-control-file) for the complete list. Functional groups:

- **HX711 load cell:** `DT_PIN 17`, `SCK_PIN 18`. (Comments say "D2/D3" but the actual values are 17 and 18 — the comment is stale.)
- **Traction wheels (×3):** PWM on pins 6, 5, 4; direction on pins 23, 25, 27.
- **Winch (the "Circumferential Motor Pins" comment is mislabeled):** `Winch_PWM 3`, `Winch_DIR 29`.
- **Circumferential (the "Winch Motor Pins" comment is mislabeled):** `Circumferential_PWM 45`, `Circumferential_DIR 31`.
- **Stepper:** `Stepper_PWM 37`, `Stepper_DIR 35` (comment warns these "may be flipped").
- **Chainsaw:** `Chainsaw_EN 39` (simple on/off).
- **Load-cell deadband:** `PLMI 5` (± padding around the tension target, in lbs).
- **Ultrasonic:** `Ultrasonic_ECHO 20`, `Ultrasonic_TRIG 19`.
- **Radio:** `CE_PIN 49`, `CSN_PIN 53`.

> ⚠️ **Two comment/label swaps to be aware of:** the block commented "Circumferential Motor Pins" actually defines the **Winch** pins, and the block commented "Winch Motor Pins" actually defines the **Circumferential** pins. The `#define` *names* are correct and used consistently in code; only the comments are swapped. Trust the names, not the comments.

### 4.3 Global variables (L51–124)

Grouped by purpose (each is fully defined in the [glossary](#8-glossary--master-control-file)):

- **Connection timing:** `lastReceivedTime`, `connectionTimeout = 200 ms`.
- **Tension setpoint:** `Target_Weight = 30` (lbs, adjustable). Converted to load-cell quanta: `TW_Adj = Target_Weight * 10430` and `PLMI_Adj = PLMI * 10430` (L73–74). The constant **10430 quanta ≈ 1 lb** is the load cell's calibration slope.
- **Ultrasonic:** `ultra_dur` (echo pulse width, µs), `ultra_dis` (computed distance, cm).
- **Load cell / DCTS state:** `scale` (HX711 object), `DCTS_EN` (auto-tension enable flag), `reading` (raw load-cell value), `counter`, `duty_percent`, and three PWM duty temporaries `temp_OCR3B/4B/5B`.
- **Radio:** `radio` object, `address`, `rxPkt` (the live received packet), `tempPkt` (staging packet before validation), `radioSent` (true when a packet has recently arrived), `ackArr[2]` (the 2 bytes sent back to the remote).
- **Direction dampening (declared but effectively unused):** `currDIR`, `dirCTR`.
- **Debug:** `startTimer`, `testchar` (used by the serial-driven `manualController()`).

### 4.4 Function-by-function

**`loadCellInit()` (L126–134)** — Starts the HX711 on `DT_PIN`/`SCK_PIN`, sets gain to **32**, waits `delay(500)` for stabilization, then `scale.tare(20)` (averages 20 readings to zero the scale). Uses a blocking `delay()`.

**`loadCellDebug()` (L136–148)** — Entirely commented out. A no-op stub left for debugging the raw/converted load readings. (Mentions the conversions: ÷10430 → lbs, ÷22998 → kg.)

**`pinInit()` (L150–172)** — Sets `pinMode(..., OUTPUT)` on every motor PWM pin, every direction pin, the chainsaw enable, and the ultrasonic TRIG. Also sets `Ultrasonic_ECHO` as OUTPUT then drives TRIG LOW. *(Note: ECHO is configured OUTPUT here even though it's read as an input by `pulseIn`; see [Known Issues](#10-known-issues-quirks-and-things-to-verify).)*

**`radioInit()` (L174–184)** — Configures the radio for **receiving** (see [section 2](#2-how-the-two-boards-talk-the-rf-link)): begin, 250 kbps, PA low, channel 76, open reading pipe 1 on the shared address, dynamic payloads, ack payloads, auto-ack on, then `startListening()`.

**`emergency()` (L186–199)** — Hard stop. Calls all four manual control functions with zero speed (circum, traction, tension, radial), then **enters an infinite `while(1)` loop** that does nothing but call `radioRX()` until the operator presses the emergency button *and* button 7's slot is satisfied — specifically it breaks only when `rxPkt.winch_spd > 100 && rxPkt.buttonID == '8'`. Until then the machine is locked. *(This is a blocking, intentionally trapping state.)*

**`PWMTimerInit()` (L201–266)** — Configures the AVR hardware timers that generate motor PWM. Detailed in [section 6](#6-the-timer--pwm-subsystem). Ends with `sei()` to enable global interrupts.

**`radioRX()` (L268–279)** — If a packet is available: set `radioSent = 1`, stamp `lastReceivedTime = millis()`, read into the staging `tempPkt`, validate with `isValid()` (currently always true), copy to the live `rxPkt`, and queue the ack payload (`ackArr`) back to the remote.

**`isValid()` (L104–107)** — Packet validator. **Currently a stub that always returns `true`** (marked "WIP").

**`rampMotorSpeed()` (L281–296)** — A helper to gradually ramp a PWM register (`OCRx`) from a current speed to a target speed in steps, with `delayMicroseconds()` between steps. Includes underflow protection on ramp-down. **Not called anywhere in the active code path** (a commented call exists in `manualTractControl`).

**`debugRX()` (L298–325)** — All Serial prints commented out; a no-op stub for dumping every received field.

**`manDelay()` (L327–332)** — A busy-wait "delay" that just increments a counter `del` times. A crude blocking spin. Used historically for the ultrasonic timing (now commented out).

**`DCTS()` (L334–358)** — **The tension control algorithm.** Fully covered in [section 5](#5-the-dynamic-cable-tension-system-dcts).

**`manualCircumControl(speed, dir)` (L360–368)** — Sets the circumferential motor PWM: `OCR4B = constrain(speed, 0, 180)` and sets `Circumferential_DIR` (LOW if `dir`, HIGH otherwise).

**`manualTensionControl(speed, dir)` (L369–377)** — **The manual counterpart to DCTS.** Sets `OCR5B = constrain(speed, 0, 150)` and sets `Winch_DIR` (HIGH if `dir`, LOW otherwise). This and `DCTS()` both own the winch outputs and are mutually exclusive (see the `DCTS_EN` gate in `mainControl`).

**`manualTractControl(speed, dir)` (L379–417)** — Drives all three traction wheels together. A large commented-out block at the top shows an abandoned direction-change dampening scheme (the `currDIR`/`dirCTR` idea). The **active** code: `OCR3B = constrain(speed, 0, 193)` and sets all three traction direction pins together (LOW if `dir`, HIGH otherwise).

**`manualRadialControl(on, dir)` (L419–430)** — Controls the stepper. If `on`: enable the Timer 1 compare interrupt (`TIMSK1 |= (1<<OCIE1A)`) so the stepper pulses, and set `Stepper_DIR`. If `!on`: disable that interrupt (stepper stops). Note this is an on/off + direction control — the *step rate* is fixed by Timer 1's `OCR1A`, not by a speed value. **(This is the radial system you asked to leave alone.)**

**`manualController()` (L432–564)** — A **serial-keyboard debug controller**. A big `switch(testchar)` mapping single letters to actions (E=emergency, O/P/I=traction, F/R=tension, Q/W=radial, T/Y=circum, S=stop all, C/N=chainsaw on/off, Z/X=unused). The line that would read `testchar` from Serial is commented out, so in normal operation `testchar` never changes and this does nothing. Kept for bench testing with just the machine and a laptop.

**`readDebug()` (L567–577)** — Commented out entirely. Was used to inject mock load/distance values from the serial monitor into the ack array.

**`buttonSel()` (L579–633)** — Acts on `rxPkt.buttonID` (the button reported by the remote):
- `'1'` → `DCTS_EN = 1` (turn the auto-tension system **ON**)
- `'2'` → `DCTS_EN = 0` (turn it **OFF**)
- `'5'` → chainsaw ON (`Chainsaw_EN HIGH`)
- `'6'` → chainsaw OFF (`Chainsaw_EN LOW`)
- `'3'`, `'4'`, `'7'`, `'8'` → currently empty / no action.

**`printControllerDebug()` (L635–658)** — Calls `radioRX()` then (all Serial prints commented out) would dump every joystick value. Effectively just receives a packet.

**`ultraSonic()` (L660–672)** — Fires the ultrasonic sensor: TRIG low→high→low, then `pulseIn(Ultrasonic_ECHO, HIGH, 30000)` reads the echo width into `ultra_dur` (30 ms timeout). Distance `ultra_dis = ultra_dur * 0.0343 / 2` (speed of sound 343 m/s → 0.0343 cm/µs, halved for round trip). The fixed timing delays around TRIG are commented out.

**`setup()` (L674–684)** — Runs once: Serial at 115200, `radioInit()`, print "Done Initializing.", `pinInit()`, `loadCellInit()`, `PWMTimerInit()`, then `printf_begin()` and `radio.printPrettyDetails()` to dump the radio config.

**`mainControl()` (L686–739)** — **The heart of the loop.** Sequence each iteration:
1. `radioRX()` — get the latest packet.
2. If `millis() - lastReceivedTime > connectionTimeout` (200 ms) → `radioSent = 0` (connection considered lost).
3. If `scale.is_ready()`: `reading = scale.read()`; `ackArr[0] = reading / 10430.0` (load in lbs); call `ultraSonic()`; `ackArr[1] = ultra_dis` (range in cm). *(So both ack bytes are refreshed only when the load cell is ready.)*
4. **If `radioSent` (link is alive):**
   - If `rxPkt.b_EMERGENCY` → `emergency()` (lock down).
   - If `rxPkt.tensionSet != Target_Weight && rxPkt.tensionSet < 176` → adopt the new setpoint: `Target_Weight = constrain(rxPkt.tensionSet, 10, 175)`, recompute `TW_Adj`.
   - **The tension mode switch:** if `DCTS_EN` → run `DCTS()` (auto); else → `manualTensionControl(rxPkt.winch_spd, rxPkt.winch_dir)` (manual).
   - `buttonSel()` — handle the reported button.
   - Manual drive of the other actuators: `manualCircumControl(...)`, `manualTractControl(...)`, `manualRadialControl((rxPkt.radial_spd > 20), rxPkt.radial_dir)` (stepper turns on only past a speed threshold of 20).
5. **Else (link lost):** zero everything — circum, traction, tension, radial all set to 0. This is the failsafe when packets stop arriving.

**`loop()` (L741–751)** — Calls only `mainControl()`. Everything else (`manualController`, debug, standalone ultrasonic) is commented out. The comment confirms: *"This is the only function that needs to be in here."*

**The ISRs (L754–789)** — Detailed in [section 6](#6-the-timer--pwm-subsystem).

---

## 5. The Dynamic Cable Tension System (DCTS)

This is the system you intend to replace with a PID loop. Here is exactly what it is and what depends on it.

### 5.1 The algorithm (`DCTS()`, L334–358)

```cpp
void DCTS() {
  if (reading >= (TW_Adj + PLMI_Adj)) {        // above target + deadband
    temp_OCR5B = (reading - TW_Adj) / 10500;   // proportional-ish speed
    OCR5B = constrain((uint8_t)temp_OCR5B, 12, 30);
    digitalWrite(Winch_DIR, HIGH);             // retract
  } else if (reading <= (TW_Adj - PLMI_Adj)) { // below target - deadband
    temp_OCR5B = (TW_Adj - reading) / 10500;
    OCR5B = constrain((uint8_t)temp_OCR5B, 12, 30);
    digitalWrite(Winch_DIR, LOW);              // release
  } else {                                     // inside deadband
    temp_OCR5B = 0;
    OCR5B = temp_OCR5B;                         // brake (no drive)
  }
}
```

**What it does, in words:** Compare the measured load (`reading`, in quanta) against the target (`TW_Adj`) with a deadband of `±PLMI_Adj`. If too tight, drive the winch to release; if too loose, drive it to retract; if within the deadband, stop. The drive *speed* is proportional to how far outside the deadband you are (`error / 10500`), but it is clamped to a narrow **12–30** PWM range (the "quarter power" limit added in v1.1).

**This is not a PID.** It is bang-bang (three-state: retract / release / brake) with a proportional speed scaling and a deadband. There is **no integral term, no derivative term, no filtering, and no millis()-based timing**. The error is computed inline; there is no stored error variable, no accumulator, and no previous-error memory.

**Key magic numbers:**
- `10430` — quanta per pound (calibration slope), used to build `TW_Adj`.
- `10500` — the divisor that scales tension error to PWM duty. The v1.1 comment calls it "1/7th the original slope," widening the range of error values the equation covers.
- `12, 30` — the constrained min/max winch PWM ("1/4th the original" limits — quarter power).

### 5.2 Inputs the DCTS consumes (the feedback path)

| Input | What it is | Set where |
|-------|-----------|-----------|
| `reading` | Raw load-cell value (tension feedback) | `mainControl` L693: `reading = scale.read()` |
| `TW_Adj` | Target tension in quanta (the setpoint) | Built from `Target_Weight`; recomputed L708 when a new `tensionSet` arrives |
| `PLMI_Adj` | Deadband padding in quanta | `PLMI * 10430` (L74) |

### 5.3 Outputs the DCTS produces (the actuator path)

| Output | What it is | Consumed by |
|--------|-----------|-------------|
| `OCR5B` | Winch PWM duty register | Timer 5 ISRs (`TIMER5_COMPA/B_vect`, L777–785) which toggle `Winch_PWM` (pin 3) |
| `Winch_DIR` (pin 29) | Winch direction | The motor driver hardware |

So the full winch chain is: **`DCTS()` writes `OCR5B` + `Winch_DIR` → Timer 5 ISRs pulse `Winch_PWM` → motor driver moves the winch.**

### 5.4 What gates and surrounds the DCTS (control-flow dependencies)

- **`DCTS_EN`** (L64) is the on/off switch. It is set by remote **button '1'** (on) / **button '2'** (off) in `buttonSel()`.
- In `mainControl()` (L712–716), `DCTS_EN` chooses between `DCTS()` (auto) and `manualTensionControl()` (manual). **They are mutually exclusive and both own `OCR5B` and `Winch_DIR`.**
- **`manualTensionControl()`** (L369–377) is the manual twin: same outputs, constrained 0–150 instead of 12–30, direction taken from the joystick.

### 5.5 Other code that writes the winch outputs (must keep winning over the PID)

These also set `OCR5B`/`Winch_DIR` and any replacement must coexist with them:
- `emergency()` → `manualTensionControl(0, 0)` (L190) — stop on e-stop.
- `mainControl()` link-lost branch (L736) — `manualTensionControl(0, 0)`.
- `manualController()` debug cases `'F'`/`'R'`/`'S'` (L463, L473, L519) — only if the serial debug path is re-enabled.

### 5.6 What to keep vs. replace (for the PID migration)

**Replace:** the body of `DCTS()` (the algorithm) — and conceptually the deadband/`PLMI` machinery if the PID handles small errors differently.

**Keep (the PID reuses these):**
- `reading` / `scale.read()` — same feedback signal.
- The `Target_Weight → TW_Adj` setpoint conversion (the 10430 slope).
- The Timer 5 PWM ISRs and `Winch_PWM` pin (the actuator hardware layer).
- The `DCTS_EN` vs. `manualTensionControl()` mode switch — the PID slots in exactly where `DCTS()` is called.
- The e-stop and link-lost zeroing of `OCR5B`.

---

## 6. The Timer / PWM Subsystem

The machine generates motor PWM with **AVR hardware timers in CTC (Clear Timer on Compare) mode**, manually toggling pins in compare-match ISRs instead of using `analogWrite`. Each motor timer uses two compare registers: `OCRnA` sets the PWM **period** and `OCRnB` sets the **duty** (on-time). The COMPA ISR turns the pin **on** at the start of a period; the COMPB ISR turns it **off** partway through. So `OCRnB` is effectively the speed command.

### 6.1 Timer setup (`PWMTimerInit`, L201–266)

| Timer | Drives | Period reg | Duty reg | `OCRnA` (period) | CS bits set in code | **Actual** prescaler |
|-------|--------|-----------|----------|------------------|---------------------|----------------------|
| **Timer 3** | Traction wheels (×3) | `OCR3A` | `OCR3B` | 194 | `CS32 | CS30` | **1024** |
| **Timer 4** | Circumferential motor | `OCR4A` | `OCR4B` | 194 | `CS42 | CS40` | **1024** |
| **Timer 5** | **Winch (tension)** | `OCR5A` | `OCR5B` | 194 | `CS51 | CS50` | **64** |
| **Timer 1** | Stepper (radial) | `OCR1A` | — | 200 | `CS12 | CS10` | **1024** |

All four are put in **CTC mode** (`WGMn2 = 1`). For Timers 3/4/5, both compare interrupts are enabled (`OCIEnA | OCIEnB`). For Timer 1, the compare interrupt is left **disabled** at init (`TIMSK1 &= ~(1<<OCIE1A)`) and is turned on only when the stepper should move (`manualRadialControl`). `sei()` enables global interrupts at the end.

> ⚠️ **The prescaler comments are wrong.** The code comments claim "Prescaler = 64" for Timers 3 and 4, but the bits actually set (`CS32+CS30`, `CS42+CS40` — i.e. CS bit 2 and CS bit 0) select **prescaler 1024** on AVR timers. For Timer 5 the comment says "1024" but the bits set (`CS51+CS50` — CS bit 1 and CS bit 0) select **prescaler 64**. The hardware does what the *bits* say, not what the comments say. This matters because it sets each motor's PWM frequency. Documented here exactly so there's no confusion; **no code is being changed.**

The resulting PWM frequency for a motor timer is `16,000,000 / (prescaler × (OCRnA + 1))`. With the actual prescalers above and `OCRnA = 194`, Timers 3/4 run at ≈ 80 Hz and Timer 5 (winch) at ≈ 1.28 kHz. (Provided for understanding; verify against scope if frequency matters for the drivers.)

### 6.2 The ISRs (L754–789)

| ISR | Action |
|-----|--------|
| `TIMER3_COMPA_vect` (L755–761) | If `OCR3B != 0`, drive all three traction PWM pins HIGH (start of period). |
| `TIMER3_COMPB_vect` (L762–766) | Drive all three traction PWM pins LOW (end of duty). |
| `TIMER4_COMPA_vect` (L767–771) | If `OCR4B != 0`, drive `Circumferential_PWM` HIGH. *(Comment says "Winch" but it's the circumferential pin — another stale comment.)* |
| `TIMER4_COMPB_vect` (L773–775) | Drive `Circumferential_PWM` LOW. |
| `TIMER5_COMPA_vect` (L777–781) | If `OCR5B != 0`, drive `Winch_PWM` HIGH. **(This is the winch / tension PWM.)** |
| `TIMER5_COMPB_vect` (L783–785) | Drive `Winch_PWM` LOW. |
| `TIMER1_COMPA_vect` (L787–789) | Toggle `Stepper_PWM` each compare match → generates the stepper step pulses. |

The `if (OCRnB != 0)` guard in each COMPA ISR means a duty of 0 leaves the pin LOW (motor off / braked), which is how a speed of 0 stops each motor.

---

## 7. Wireless Controller File — Detailed Walkthrough

File: `WirelessController_pot_inc_4_28_2026.ino`. This is the handheld remote (the **transmitter**).

### 7.1 Header & includes (L1–10)

Version 1.1. The v1.1 change: *"Allows for tension to be set to an upper limit of 175 lbs when using pot in dcts."* Includes the radio stack, plus `Wire.h` + `LiquidCrystal_I2C.h` (the I2C LCD) and `printf.h`.

### 7.2 Button pin definitions (L12–21)

Nine buttons, each on a digital pin with `INPUT_PULLUP` (reads LOW when pressed). Note the comments list a *different* "Pin" number than the `#define` value — trust the `#define`:

| Button (code name) | `#define` pin | Logical # |
|--------------------|--------------|-----------|
| `Tension_On_Button` | 23 | 1 |
| `Tension_Off_Button` | 25 | 2 |
| `Tension_Set_Point_Button` | 27 | 3 |
| `Tension_Indicated_Button` | 29 | 4 |
| `Chainsaw_On_Button` | 24 | 5 |
| `Chainsaw_Off_Button` | 26 | 6 |
| `Chainsaw_Distance_Set_Point_Button` | 28 | 7 |
| `Chainsaw_Distance_Indicated_Button` | 30 | 8 |
| `Emergency_Button` | 22 | 9 |

### 7.3 Joystick / pot / NRF / LCD definitions (L23–36)

- **Left joystick:** X on `A0`, Y on `A1`.
- **Right joystick:** X on `A8`, Y on `A9`.
- **Potentiometer:** `A4` (sets the tension/range value via the LCD flow).
- **Radio:** CE `49`, CSN `53` (matches the master).
- **LCD I2C:** SDA `20`, SCL `21` (defined for clarity; the I2C library uses the hardware pins regardless).

### 7.4 State variables (L38–92)

- **Debounce:** `debounceDelay = 25 ms`; per-button arrays `lastButtonState[8]`, `lastDebounceTime[8]`, `buttonPressed[8]`.
- **Modes:** `emergencyButtonState`, `setTens` (in "set tension" LCD mode), `setDist` (in "set distance" LCD mode), `conf` (confirmed flag).
- **Joystick raw values:** `leftJoystickxVal`, `leftJoystickyVal`, `rightJoystickxVal`, `rightJoystickyVal`; tunables `deadzone = 100` (raw-count center deadzone) and `joystickDeadzone = 15` (output-side deadzone).
- **Potentiometer:** `potVal` (raw 0–1023), `mapPotVal` (mapped value shown/sent), `potCTR` (declared, unused).
- **LCD:** `lcd` object at I2C address `0x27`, 16×2.
- **Radio:** `radio` object, shared `address`, plus an unused `message`/`receivedData` pair left over from testing.
- **Packet & ack:** `packet` (the outgoing `ControllerPacket`, default shown above) and `ackArr[2]` (the load/range bytes returned by the machine).

### 7.5 Function-by-function

**`setup()` (L94–130)** — Serial 115200; set all nine button pins to `INPUT_PULLUP` and the four joystick + pot pins to `INPUT`; configure the radio for **transmitting** (begin, 250 kbps, PA low, dynamic + ack payloads, auto-ack, channel 76, `openWritingPipe(address)`); `printf_begin()` + `printPrettyDetails()`; init the LCD, backlight on, and draw the first screen via `LCDScreen()`.

**`CheckButtonDebouncePressed(pin, i, name)` (L133–170)** — Standard debounce: if the reading changed, reset that button's timer; once stable longer than `debounceDelay`, update `buttonPressed[i]`. On a *press* (LOW):
- print the button name,
- if `i == 2` (Tension Set Point) → toggle `setTens`, clear `conf`,
- if `i == 6` (Chainsaw Distance Set Point) → toggle `setDist`, clear `conf`,
- set `packet.buttonID = '1' + i` (so index 0→'1', … 7→'8'),
- `SendPacket()`.

**`CheckEmergencyButton()` (L172–220)** — Two-state e-stop:
- **Normal → emergency:** if the emergency button reads LOW, set `emergencyButtonState = true`, send a packet with `b_EMERGENCY = 1` and `buttonID = '8'`, print the warning, and flash the LCD 5 times (using blocking `delay(200)`).
- **Emergency → recover:** if both the emergency button **and** button 8 read LOW, clear `emergencyButtonState`, send `b_EMERGENCY = 0`, show "SYSTEM RECOVERED", and `delay(2000)`.

**`ReadJoystick()` (L222–275)** — Reads all four analog joystick axes, then calls `JoystickLeftPWM()` and `JoystickRightPWM()` to convert them into packet fields. The commented block at the bottom is a serial-print map of each direction's meaning.

**`JoystickLeftPWM(joystickxVal, joystickyVal)` (L277–307)** — Converts the **left** joystick. ⚠️ The caller (`ReadJoystick`, L228) invokes it as `JoystickLeftPWM(leftJoystickyVal, leftJoystickxVal)` — **Y is passed into the parameter named `joystickxVal`, and X into `joystickyVal`.** Tracing the *actual* values:
- The first parameter (`joystickxVal`, which actually holds **left joystick Y**) → `winch_spd` / `winch_dir` (tension). Distance from center (512) ×2, mapped 0–1024 → 0–193; zeroed if under `joystickDeadzone`.
- The second parameter (`joystickyVal`, which actually holds **left joystick X**) → `traction_spd` / `traction_dir`.

  So in physical terms: **left joystick Y drives tension, left joystick X drives traction** — the opposite of what the parameter names and `#define` comments suggest. See [Issue #9](#10-known-issues-quirks-and-things-to-verify).

**`JoystickRightPWM(joystickxVal, joystickyVal)` (L309–339)** — Converts the **right** joystick, called the same swapped way (`JoystickRightPWM(rightJoystickyVal, rightJoystickxVal)`, L229). By the same tracing: the first parameter (actual **right Y**) → `circum_spd`/`circum_dir`; the second parameter (actual **right X**) → `radial_spd`/`radial_dir`. Physical: **right Y drives circumferential, right X drives radial.**

**`SendPacket()` (L341–373)** — `stopListening()`, `radio.write(&packet, sizeof(packet))`. If sent, print success and, if an ack payload is available, read the 2 bytes back into `ackArr` (the machine's load/range). Else print "Packet Sent Failed!".

**`LCDScreen()` (L414–483)** — The active LCD renderer (an older version is commented out above it, L375–412). Three display modes:
- **`setTens` mode:** if button '4' was pressed or already confirmed (`conf`) → show "Confirmed N lbs", latch `packet.tensionSet = mapPotVal`. Otherwise read the pot, `mapPotVal = map(potVal, 0,1023, 0,175)`, show "Set Load: N lbs". *(The 0–175 range is the v1.1 upper limit.)*
- **`setDist` mode:** if button '8' or `conf` → "Confirmed N cm". Otherwise read pot, `mapPotVal = map(potVal, 0,1023, 0,99)`, show "Set Range: N cm".
- **Default (neither):** show live telemetry from the machine — "Load: `ackArr[0]` lbs" (or "255+" if saturated) on line 1, "Range: `ackArr[1]` cm" (or "200+") on line 2.

**`loop()` (L485–516)** — Each iteration:
1. `CheckEmergencyButton()` first. If in emergency, `return` immediately (freeze — send nothing else).
2. `ReadJoystick()` → refresh joystick fields.
3. Set `packet.buttonID = '0'` (no button) and `SendPacket()` — sends the continuous joystick stream.
4. Run all eight `CheckButtonDebouncePressed(...)` checks (each may send its own packet on a press).
5. `LCDScreen()` to refresh the display.

So the remote sends a steady stream of joystick packets (buttonID '0') plus extra packets on button presses, and shows machine telemetry when not in a set-point mode.

---

## 8. Glossary — Master Control File

### 8.1 Pin definitions

| Name | Pin | Meaning |
|------|-----|---------|
| `DT_PIN` | 17 | HX711 data (DOUT) line |
| `SCK_PIN` | 18 | HX711 clock line |
| `TractionWheel1_PWM` | 6 | Traction wheel 1 PWM (via Timer 3) |
| `TractionWheel2_PWM` | 5 | Traction wheel 2 PWM |
| `TractionWheel3_PWM` | 4 | Traction wheel 3 PWM |
| `TractionWheel1_DIR` | 23 | Traction wheel 1 direction |
| `TractionWheel2_DIR` | 25 | Traction wheel 2 direction |
| `TractionWheel3_DIR` | 27 | Traction wheel 3 direction |
| `Winch_PWM` | 3 | Winch (tension) PWM (via Timer 5) |
| `Winch_DIR` | 29 | Winch direction (HIGH=retract, LOW=release in DCTS) |
| `Circumferential_PWM` | 45 | Circumferential motor PWM (via Timer 4) |
| `Circumferential_DIR` | 31 | Circumferential motor direction |
| `Stepper_PWM` | 37 | Stepper step pulse (toggled by Timer 1 ISR) |
| `Stepper_DIR` | 35 | Stepper direction |
| `Chainsaw_EN` | 39 | Chainsaw enable (HIGH=on, LOW=off) |
| `PLMI` | 5 | Tension deadband padding, in **lbs** (not a pin) |
| `Ultrasonic_ECHO` | 20 | Ultrasonic echo input |
| `Ultrasonic_TRIG` | 19 | Ultrasonic trigger output |
| `CE_PIN` | 49 | nRF24 chip-enable |
| `CSN_PIN` | 53 | nRF24 chip-select-not (SPI) |

### 8.2 Variables and objects

| Name | Type | Meaning |
|------|------|---------|
| `lastReceivedTime` | `unsigned long` | `millis()` timestamp of the last packet received |
| `connectionTimeout` | `const unsigned long` = 200 | ms with no packet before the link is "lost" |
| `Target_Weight` | `uint8_t` = 30 | Desired cable tension, **lbs** (the tension setpoint) |
| `ultra_dur` | `unsigned long` | Ultrasonic echo pulse width, µs |
| `ultra_dis` | `unsigned long` | Computed distance, cm |
| `scale` | `HX711` | Load-cell ADC object |
| `DCTS_EN` | `bool` = 0 | **Auto-tension enable.** True → run `DCTS()`; false → manual winch |
| `reading` | `long` = 0 | Raw load-cell value (tension feedback), "quanta" |
| `counter` | `short int` = 0 | Message-rate slowdown counter (declared; minimal use) |
| `duty_percent` | `uint8_t` = 0 | Duty-cycle percent (declared; not central to logic) |
| `temp_OCR3B` | `uint8_t` | Temp duty for traction before constrain → `OCR3B` |
| `temp_OCR4B` | `uint8_t` | Temp duty for circumferential → `OCR4B` |
| `temp_OCR5B` | `uint8_t` | Temp duty for winch → `OCR5B` |
| `TW_Adj` | `long int` | Target tension in **quanta** = `Target_Weight × 10430` (the setpoint the DCTS compares against) |
| `PLMI_Adj` | `long int` | Deadband in **quanta** = `PLMI × 10430` |
| `radio` | `RF24` | Radio object (CE 49, CSN 53) |
| `address` | `const byte[5]` | Shared RF address `{'T','E','S','T','1'}` |
| `rxPkt` | `ControllerPacket` | The **live** received packet acted on by the machine |
| `tempPkt` | `ControllerPacket` | Staging packet, validated before copying to `rxPkt` |
| `radioSent` | `bool` = 0 | True when a packet was recently received (link alive) |
| `ackArr` | `uint8_t[2]` | Bytes sent back to remote: `[0]`=load(lbs), `[1]`=range(cm) |
| `currDIR` | `bool` = 0 | Intended last-direction for traction dampening (largely unused) |
| `dirCTR` | `uint16_t` = 0 | Direction-change dwell counter (largely unused) |
| `startTimer` | `unsigned long` | Debug timing (declared) |
| `testchar` | `char` = '0' | Serial-debug command char for `manualController()` |

### 8.3 Key constants embedded in code

| Value | Meaning |
|-------|---------|
| `10430` | Load-cell calibration: **quanta per pound** (slope) |
| `22998` | Quanta per kilogram (used only in commented debug) |
| `10500` | DCTS error→PWM divisor ("1/7th slope" per v1.1 comment) |
| `12 … 30` | DCTS winch PWM clamp (quarter-power limit) |
| `0 … 150` | Manual winch PWM clamp (`manualTensionControl`) |
| `0 … 180` | Circumferential PWM clamp |
| `0 … 193` | Traction PWM clamp |
| `194` | `OCRnA` PWM period for Timers 3/4/5 |
| `200` | `OCR1A` stepper period (Timer 1) |
| `0.0343 / 2` | Ultrasonic µs→cm conversion (sound 0.0343 cm/µs, ÷2 round trip) |
| `30000` | `pulseIn` echo timeout, µs (≈ 5 m max range) |
| `10`, `175` | Allowed tension-setpoint range (lbs) when adopting a new `tensionSet` |
| `176` | Upper bound check on incoming `tensionSet` |
| `32` | HX711 gain setting |
| `20` | Tare sample count |

---

## 9. Glossary — Wireless Controller File

### 9.1 Pin definitions

| Name | Pin | Meaning |
|------|-----|---------|
| `Tension_On_Button` | 23 | Button 1 — turn auto-tension ON |
| `Tension_Off_Button` | 25 | Button 2 — turn auto-tension OFF |
| `Tension_Set_Point_Button` | 27 | Button 3 — enter "set tension" mode |
| `Tension_Indicated_Button` | 29 | Button 4 — confirm tension / indicate |
| `Chainsaw_On_Button` | 24 | Button 5 — chainsaw ON |
| `Chainsaw_Off_Button` | 26 | Button 6 — chainsaw OFF |
| `Chainsaw_Distance_Set_Point_Button` | 28 | Button 7 — enter "set distance" mode |
| `Chainsaw_Distance_Indicated_Button` | 30 | Button 8 — confirm distance / recover from e-stop |
| `Emergency_Button` | 22 | Button 9 — emergency stop |
| `Left_Joystick_Left_Right_X` | A0 | Left joystick X → **traction** (see Issue #9; comments say tension, code wires it to traction) |
| `Left_Joystick_Up_Down_Y` | A1 | Left joystick Y → **tension/winch** (see Issue #9) |
| `Right_Joystick_Left_Right_X` | A8 | Right joystick X → **radial** (see Issue #9) |
| `Right_Joystick_Up_Down_Y` | A9 | Right joystick Y → **circumferential** (see Issue #9) |
| `Potentiometer` | A4 | Sets tension/range value in the LCD set-point flow |
| `CE_PIN` | 49 | nRF24 chip-enable |
| `CSN_PIN` | 53 | nRF24 chip-select-not |
| `SDA_PIN` | 20 | I2C data (documented; LCD lib uses hardware I2C) |
| `SCL_PIN` | 21 | I2C clock (documented) |

### 9.2 Variables and objects

| Name | Type | Meaning |
|------|------|---------|
| `debounceDelay` | `const unsigned long` = 25 | Debounce stability window, ms |
| `lastButtonState[8]` | `int[]` | Last raw read of each button |
| `lastDebounceTime[8]` | `unsigned long[]` | Last time each button's state changed |
| `buttonPressed[8]` | `bool[]` | Current debounced pressed state per button |
| `emergencyButtonState` | `bool` | True while in emergency lockout |
| `setTens` | `bool` | True while in "set tension" LCD mode |
| `setDist` | `bool` | True while in "set distance" LCD mode |
| `conf` | `bool` | True once a set-point value is confirmed |
| `leftJoystickxVal` | `int` | Raw left-joystick X (0–1023) |
| `leftJoystickyVal` | `int` | Raw left-joystick Y |
| `rightJoystickxVal` | `int` | Raw right-joystick X |
| `rightJoystickyVal` | `int` | Raw right-joystick Y |
| `deadzone` | `int` = 100 | Raw-count center deadzone for direction decisions |
| `joystickDeadzone` | `int` = 15 | Output-side deadzone (zeroes small speeds) |
| `potVal` | `uint16_t` = 0 | Raw potentiometer (0–1023) |
| `mapPotVal` | `uint8_t` = 0 | Mapped pot value (lbs or cm depending on mode) |
| `potCTR` | `uint16_t` = 0 | Declared, unused |
| `lcd` | `LiquidCrystal_I2C` | LCD object, address 0x27, 16×2 |
| `radio` | `RF24` | Radio object (CE 49, CSN 53) |
| `address` | `const byte[5]` | Shared RF address (must match the machine) |
| `message` | `const char[]` | Leftover test string, unused in logic |
| `receivedData` | `char[32]` | Leftover receive buffer, unused |
| `packet` | `ControllerPacket` | The outgoing packet; default `{'9',0,0,0,0,0,0,0,0,0,20}` |
| `ackArr[2]` | `uint8_t[]` | Telemetry back from machine: `[0]`=load lbs, `[1]`=range cm |

### 9.3 Key constants

| Value | Meaning |
|-------|---------|
| `512` | Joystick center (10-bit ADC midpoint) |
| `1024` | Joystick range used in `map()` |
| `193` | Max mapped joystick output (matches motor PWM clamps) |
| `0 … 175` | "Set Load" pot mapping range (lbs) — v1.1 upper limit |
| `0 … 99` | "Set Range" pot mapping range (cm) |
| `255` | LCD "load saturated" sentinel (`ackArr[0]==255` → "255+") |
| `200` | LCD "range saturated" threshold (`ackArr[1]>200` → "200+") |
| `0x27` | LCD I2C address |

---

## 10. Known Issues, Quirks, and Things to Verify

These are factual observations from reading the code — **not** changes. Flagged so you fully understand the code's real behavior.

1. **Swapped comment labels (master, pin defs).** The block commented "Circumferential Motor Pins" defines the **Winch** pins; the block commented "Winch Motor Pins" defines the **Circumferential** pins. The `#define` *names* are correct and used correctly everywhere; only the comments are swapped.

2. **Timer prescaler comments are wrong (master, `PWMTimerInit`).** Comments say 64 for Timers 3/4 but the bits select **1024**; the comment says 1024 for Timer 5 but the bits select **64**. The hardware follows the bits. This affects each motor's PWM frequency. (Documented in [section 6](#6-the-timer--pwm-subsystem).)

3. **Stale pin comments (master).** `DT_PIN`/`SCK_PIN` comments say "D2/D3" but the values are 17/18. Several ISR comments say "Winch" where the pin is actually circumferential.

4. **Ultrasonic ECHO set as OUTPUT (master, `pinInit` L170).** `Ultrasonic_ECHO` is configured `OUTPUT`, yet `ultraSonic()` reads it with `pulseIn(..., HIGH, ...)`. On most boards `pulseIn` still works because the pin is driven by the sensor, but configuring an input pin as OUTPUT is unusual and worth verifying on hardware.

5. **`isValid()` is a stub** (master L104–107) — always returns `true`, so no packet validation actually happens despite the staging-buffer pattern in `radioRX()`.

6. **`emergency()` blocks the whole machine** (master L186–199) in a `while(1)` until a specific recovery combo (`winch_spd > 100 && buttonID == '8'`). While locked, only `radioRX()` runs; sensors and the main loop are frozen.

7. **Blocking delays exist** in `loadCellInit()` (`delay(500)`), `rampMotorSpeed()` (`delayMicroseconds`), and on the remote in `CheckEmergencyButton()` (`delay(200)` ×10, `delay(2000)`). These run outside the steady-state control path but are worth noting for the project's "non-blocking" goal.

8. **`OCR5B` is written non-atomically** while Timer 5 ISRs read it. Harmless at the current narrow bang-bang values, but a more active PID update may want an interrupt-safe write. (Relevant to the planned PID migration, not a current bug.)

9. **Swapped joystick axis arguments** (remote, `ReadJoystick` → `JoystickLeftPWM`/`JoystickRightPWM`). This is a real behavioral subtlety, not just naming. `ReadJoystick()` calls `JoystickLeftPWM(leftJoystickyVal, leftJoystickxVal)` and `JoystickRightPWM(rightJoystickyVal, rightJoystickxVal)` — the **Y value is passed first**, into a parameter named `joystickxVal`. Since the function's first parameter drives the "X-named" packet field, the result is:
   - Left joystick **Y** → `winch_spd` (tension); left **X** → `traction_spd`.
   - Right joystick **Y** → `circum_spd`; right **X** → `radial_spd`.

   This is the **opposite axis** from what both the parameter names and the `#define` comments (e.g. "Left Joystick X ... For Tension") state. The code's actual behavior is as traced here; the comments are misleading. Confirm against the physical joysticks before relying on any axis label.

10. **`tensionSet` bounds.** `mainControl()` adopts a new setpoint only if `rxPkt.tensionSet != Target_Weight && rxPkt.tensionSet < 176`, then clamps to `[10, 175]`. The remote's pot maps 0–175. So a pot value of 0–9 would be clamped up to 10 on the machine side.

---

*End of documentation. This document describes the code as uploaded; no source files were modified.*
