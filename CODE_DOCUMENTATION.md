# RPT Control System — Full Code Documentation

**Project:** Robotic Palm Trimmer (RPT) Control System
**Platform:** Arduino Mega 2560 (both the on-machine controller and the handheld remote)
**Documented files:**

| File | Role | Version header |
|------|------|----------------|
| `mastercontrolfile_quarter_powered_DCTS_04_28_2026.ino` | On-machine "master" controller — runs all motors, reads sensors, executes the tension system | v1.1, 4-28-2026 |
| `WinchTensionPID_test_06_19_2026.ino` | Standalone bench-test sketch — the proof-of-concept winch-tension PID the master file's loop was ported from (drives only the winch and the load cell; no radio, no other motors) | n/a (test harness) |
| `WirelessController_pot_inc_4_28_2026.ino` | Handheld remote transmitter — reads buttons/joysticks/pot, drives an LCD, sends packets | v1.1, 4-28-2026 |

> **Scope note:** This document *describes* the code in the current working tree — the master file **after** the winch-tension PID replaced the old bang-bang DCTS algorithm (see [section 5](#5-the-dynamic-cable-tension-system-dcts)). No further code is modified by this document. Line numbers refer to the working-tree source (the master file is now 929 lines); the master file's in-code version header still reads v1.1 because the PID change is not yet version-bumped. Where the code's own comments are factually wrong (e.g. timer prescaler values), this document states the actual hardware behavior and flags the discrepancy.

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

The **tension system** is the only closed-loop behavior in the current code. It is named **DCTS** (Dynamic Cable Tension System) and lives in the `DCTS()` function. In the current working tree that function is a **real PID loop** — proportional, integral, and derivative terms running once per fresh load-cell sample with real `millis()` timing — replacing the v1.1 bang-bang algorithm (see [section 5](#5-the-dynamic-cable-tension-system-dcts)). Everything else is open-loop manual control driven directly by the operator's joysticks.

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
- **Tension setpoint:** `Target_Weight = 30` (lbs, adjustable) — **now the PID setpoint directly, in lbs.** The quanta conversions `TW_Adj = Target_Weight * 10430` and `PLMI_Adj = PLMI * 10430` (L73–74) are **vestigial** after the PID migration: `TW_Adj` is still recomputed on setpoint change (L846) but no control path reads it, and `PLMI_Adj` is unused. The constant **10430 quanta ≈ 1 lb** is the load cell's calibration slope (now the named constant `QUANTA_PER_LB`, L342).
- **Ultrasonic:** `ultra_dur` (echo pulse width, µs), `ultra_dis` (computed distance, cm).
- **Load cell / DCTS state:** `scale` (HX711 object), `DCTS_EN` (auto-tension enable flag), `reading` (raw load-cell value), `counter`, `duty_percent`, and three PWM duty temporaries `temp_OCR3B/4B/5B`.
- **PID loop state:** declared in the PID block itself (L369–378), not in this globals section — `measured_lbs` (feedback in lbs), `sensorValid` (reading validity flag), `integral`, `prevError`, `derivFiltered`, `newSample` (fresh-read latch), `lastPIDTime` (PID `dt` clock), `overTensionStart` / `overTensionLatched` (over-tension relief timer and latch). See [section 5](#5-the-dynamic-cable-tension-system-dcts) and the [glossary](#8-glossary--master-control-file).
- **Radio:** `radio` object, `address`, `rxPkt` (the live received packet), `tempPkt` (staging packet before validation), `radioSent` (true when a packet has recently arrived), `ackArr[2]` (the 2 bytes sent back to the remote).
- **Direction dampening (declared but effectively unused):** `currDIR`, `dirCTR`.
- **Debug:** `startTimer`, `testchar` (used by the serial-driven `manualController()`).

### 4.4 Function-by-function

**`loadCellInit()` (L126–134)** — Starts the HX711 on `DT_PIN`/`SCK_PIN`, sets gain to **32**, waits `delay(500)` for stabilization, then `scale.tare(20)` (averages 20 readings to zero the scale). Uses a blocking `delay()`.

**`loadCellDebug()` (L136–148)** — Entirely commented out. A no-op stub left for debugging the raw/converted load readings. (Mentions the conversions: ÷10430 → lbs, ÷22998 → kg.)

**`pinInit()` (L150–173)** — Sets `pinMode(..., OUTPUT)` on every motor PWM pin, every direction pin, the chainsaw enable, and the ultrasonic TRIG. It explicitly drives `Chainsaw_EN` LOW so the cutting tool starts disabled. It also sets `Ultrasonic_ECHO` as OUTPUT then drives TRIG LOW. *(Note: ECHO is configured OUTPUT here even though it's read as an input by `pulseIn`; see [Known Issues](#10-known-issues-quirks-and-things-to-verify).)*

**`radioInit()` (L174–184)** — Configures the radio for **receiving** (see [section 2](#2-how-the-two-boards-talk-the-rf-link)): begin, 250 kbps, PA low, channel 76, open reading pipe 1 on the shared address, dynamic payloads, ack payloads, auto-ack on, then `startListening()`.

**`emergency()`** — Hard stop. Disables `DCTS_EN`, stops all four motion systems, and drives `Chainsaw_EN` LOW before remaining in a blocking receive loop while `rxPkt.b_EMERGENCY` is true. The controller repeatedly transmits zero-motion emergency packets, and the deliberate Emergency + Button 8 recovery gesture sends `b_EMERGENCY = 0`. The clear packet is therefore the only exit condition; stale joystick speed or button fields cannot release the lock. On the way out the master calls `resetPID()`, and automatic tension must be explicitly re-enabled by the operator.

**`PWMTimerInit()` (L201–266)** — Configures the AVR hardware timers that generate motor PWM. Detailed in [section 6](#6-the-timer--pwm-subsystem). Ends with `sei()` to enable global interrupts.

**`radioRX()` (L268–279)** — If a packet is available: set `radioSent = 1`, stamp `lastReceivedTime = millis()`, read into the staging `tempPkt`, validate with `isValid()` (currently always true), copy to the live `rxPkt`, and queue the ack payload (`ackArr`) back to the remote.

**`isValid()` (L104–107)** — Packet validator. **Currently a stub that always returns `true`** (marked "WIP").

**`rampMotorSpeed()` (L281–296)** — A helper to gradually ramp a PWM register (`OCRx`) from a current speed to a target speed in steps, with `delayMicroseconds()` between steps. Includes underflow protection on ramp-down. **Not called anywhere in the active code path** (a commented call exists in `manualTractControl`).

**`debugRX()` (L298–325)** — All Serial prints commented out; a no-op stub for dumping every received field.

**`manDelay()` (L327–332)** — A busy-wait "delay" that just increments a counter `del` times. A crude blocking spin. Used historically for the ultrasonic timing (now commented out).

**`winchStop()` (L380–382)** — Sets the winch duty to zero (`OCR5B = 0`); the Timer 5 ISRs then hold `Winch_PWM` LOW (motor off/braked). Does not touch `Winch_DIR`.

**`winchRelieve()` (L384–387)** — Drives `Winch_DIR` HIGH (the "too tight" correction direction) at the stall floor `WINCH_DUTY_MIN`. Used only by the over-tension branch of `DCTS()`, to creep back under the hard limit.

**`resetPID()` (L389–397)** — Clears the entire loop state: `integral`, `prevError`, `derivFiltered`, `newSample`, the over-tension timer and latch, and restarts the `dt` clock from `millis()`. Called every time ownership of the winch changes hands — see [5.4](#54-what-gates-and-surrounds-the-dcts-control-flow-dependencies).

**`driveWinch(effort)` (L399–415)** — Translates a signed PID effort into actuator state. The sign sets `Winch_DIR`: effort > 0 means measured tension is *below* setpoint (too loose) → `Winch_DIR` LOW; effort < 0 means too tight → HIGH. These are the same pin states v1.1 used, but which physical motion each one produces has never been verified (see Issue #13), so this document deliberately does not label either as "spool in" or "spool out". The magnitude is mapped linearly from `[0, EFFORT_FULL_SCALE]` onto the duty band `[12, 30]` and written to `OCR5B`; magnitudes below `EFFORT_DEADZONE` → `winchStop()`.

**`DCTS()` (L417–482)** — **The tension control algorithm — now a PID loop.** Part of the PID block (L335–482) that replaced the v1.1 bang-bang body. Fully covered in [section 5](#5-the-dynamic-cable-tension-system-dcts).

**`manualCircumControl(speed, dir)` (L484–492)** — Sets the circumferential motor PWM: `OCR4B = constrain(speed, 0, 180)` and sets `Circumferential_DIR` (LOW if `dir`, HIGH otherwise).

**`manualTensionControl(speed, dir)` (L493–501)** — **The manual counterpart to the PID.** Sets `OCR5B = constrain(speed, 0, 150)` and sets `Winch_DIR` (HIGH if `dir`, LOW otherwise). This and `DCTS()` both own the winch outputs and are mutually exclusive (see the `DCTS_EN` gate in `mainControl`).

**`manualTractControl(speed, dir)` (L503–541)** — Drives all three traction wheels together. A large commented-out block at the top shows an abandoned direction-change dampening scheme (the `currDIR`/`dirCTR` idea). The **active** code: `OCR3B = constrain(speed, 0, 193)` and sets all three traction direction pins together (LOW if `dir`, HIGH otherwise).

**`manualRadialControl(on, dir)` (L543–554)** — Controls the stepper. If `on`: enable the Timer 1 compare interrupt (`TIMSK1 |= (1<<OCIE1A)`) so the stepper pulses, and set `Stepper_DIR`. If `!on`: disable that interrupt (stepper stops). Note this is an on/off + direction control — the *step rate* is fixed by Timer 1's `OCR1A`, not by a speed value. **(This is the radial system you asked to leave alone.)**

**`manualController()` (L556–688)** — A **serial-keyboard debug controller**. A big `switch(testchar)` mapping single letters to actions (E=emergency, O/P/I=traction, F/R=tension, Q/W=radial, T/Y=circum, S=stop all, C/N=chainsaw on/off, Z/X=unused). The line that would read `testchar` from Serial is commented out, so in normal operation `testchar` never changes and this does nothing. Kept for bench testing with just the machine and a laptop.

**`readDebug()` (L691–701)** — Commented out entirely. Was used to inject mock load/distance values from the serial monitor into the ack array.

**`buttonSel()` (L703–761)** — Acts on `rxPkt.buttonID` (the button reported by the remote):
- `'1'` → `DCTS_EN = 1` (turn the auto-tension system **ON**). On a genuine off→on transition it first calls `resetPID()` (L705–711) so the loop never resumes with a stale integrator.
- `'2'` → `DCTS_EN = 0` (turn it **OFF**), then `resetPID()` (L713–718). This is also how an over-tension latch is cleared.
- `'5'` → chainsaw ON (`Chainsaw_EN HIGH`)
- `'6'` → chainsaw OFF (`Chainsaw_EN LOW`)
- `'3'`, `'4'`, `'7'`, `'8'` → currently empty / no action.

**`printControllerDebug()` (L763–786)** — Calls `radioRX()` then (all Serial prints commented out) would dump every joystick value. Effectively just receives a packet.

**`ultraSonic()` (L788–800)** — Fires the ultrasonic sensor: TRIG low→high→low, then `pulseIn(Ultrasonic_ECHO, HIGH, 30000)` reads the echo width into `ultra_dur` (30 ms timeout). Distance `ultra_dis = ultra_dur * 0.0343 / 2` (speed of sound 343 m/s → 0.0343 cm/µs, halved for round trip). The fixed timing delays around TRIG are commented out.

**`setup()` (L802–813)** — Runs once: Serial at 115200, `radioInit()`, print "Done Initializing.", `pinInit()`, `loadCellInit()`, `PWMTimerInit()`, then `resetPID()` (starts the PID's `dt` clock, L809), then `printf_begin()` and `radio.printPrettyDetails()` to dump the radio config.

**`mainControl()` (L815–879)** — **The heart of the loop.** Sequence each iteration:
1. `radioRX()` — get the latest packet.
2. If `millis() - lastReceivedTime > connectionTimeout` (200 ms) → `radioSent = 0` (connection considered lost).
3. If `scale.is_ready()` (L821–836): `reading = scale.read()` (L822); range-check it against `RAW_MIN_VALID`–`RAW_MAX_VALID`, setting `sensorValid` and, when valid, converting `measured_lbs = reading / QUANTA_PER_LB` (L823–828); set the PID's fresh-sample latch `newSample = true`, **but only when `DCTS_EN`** (L829–831), so the latch can't go stale during manual control; then `ackArr[0] = reading / 10430.0` (load in lbs), `ultraSonic()`, `ackArr[1] = ultra_dis` (range in cm). *(So both ack bytes are refreshed only when the load cell is ready.)*
4. **If `radioSent` (link is alive):**
   - If `rxPkt.b_EMERGENCY` → `emergency()` (lock down until an explicit clear packet, and disable automatic tension).
   - If `rxPkt.tensionSet != Target_Weight && rxPkt.tensionSet < 176` → adopt the new setpoint: `Target_Weight = constrain(rxPkt.tensionSet, 10, 175)` (L845) — this is now the PID's setpoint, in lbs — recompute `TW_Adj` (L846; vestigial, the PID no longer reads it), and `resetPID()` (L847) so the old integrator isn't carried into the new setpoint.
   - **The tension mode switch** (L851–855): if `DCTS_EN` → run `DCTS()` (the PID, auto); else → `manualTensionControl(rxPkt.winch_spd, rxPkt.winch_dir)` (manual).
   - `buttonSel()` — handle the reported button.
   - Manual drive of the other actuators: `manualCircumControl(...)`, `manualTractControl(...)`, `manualRadialControl((rxPkt.radial_spd > 20), rxPkt.radial_dir)` (stepper turns on only past a speed threshold of 20).
5. **Else (link lost):** zero every motion system, drive `Chainsaw_EN` LOW, and call `resetPID()` so the loop starts clean when the link returns. The cutting tool therefore cannot remain latched on after RF traffic stops.

**`loop()` (L881–891)** — Calls only `mainControl()`. Everything else (`manualController`, debug, standalone ultrasonic) is commented out. The comment confirms: *"This is the only function that needs to be in here."*

**The ISRs (L894–929)** — Detailed in [section 6](#6-the-timer--pwm-subsystem).

---

## 5. The Dynamic Cable Tension System (DCTS)

**Status: the DCTS algorithm is now a real winch-tension PID loop.** The v1.1 bang-bang controller (three-state, with a deadband) is gone; `DCTS()` now runs a proportional–integral–derivative controller on cable tension. The PID is the production port of the proof-of-concept from the standalone bench sketch `WinchTensionPID_test_06_19_2026.ino` (which drives only the winch and the load cell, no radio). The function name, the call site in `mainControl()`, and the Timer 5 actuator chain are all unchanged — but the port is not confined to `DCTS()`: `mainControl()` now also does the raw range check and lbs conversion, `setup()` starts the `dt` clock, and four call sites gained `resetPID()` (see [5.4](#54-what-gates-and-surrounds-the-dcts-control-flow-dependencies)).

### 5.1 The algorithm (PID block, L335–482)

The whole loop lives in one block: the section header (L335–340), calibration and limit constants (L342–348), gains (L350–356), the effort-mapping and over-tension constants (L358–367), PID state (L369–378), three actuator/state helpers (`winchStop()` L380–382, `winchRelieve()` L384–387, `resetPID()` L389–397), the effort-to-hardware mapping (`driveWinch()` L399–415), and the controller body `DCTS()` (L417–482):

```cpp
void driveWinch(float effort) {  // Turns a signed PID effort into a direction & duty
  float mag = (effort < 0.0f) ? -effort : effort;
  if (mag < EFFORT_DEADZONE) {
    winchStop();
    return;
  }

  if (effort > 0.0f) {
    digitalWrite(Winch_DIR, LOW);   // Correction direction for too loose
  } else {
    digitalWrite(Winch_DIR, HIGH);  // Correction direction for too tight
  }

  float frac = mag / EFFORT_FULL_SCALE;
  if (frac > 1.0f) frac = 1.0f;
  OCR5B = (uint8_t)(WINCH_DUTY_MIN + frac * (WINCH_DUTY_MAX - WINCH_DUTY_MIN) + 0.5f);
}

void DCTS() {  // Reads the load cell & performs the tension PID on it
  if (!newSample) {  // Only step on a fresh sample, the HX711 runs near 10 Hz
    return;
  }
  newSample = false;

  unsigned long now = millis();
  float dt = (float)(now - lastPIDTime) / 1000.0f;
  lastPIDTime = now;
  if (dt < 0.005f) dt = 0.005f;  // Guard against a zero span
  if (dt > 0.25f) dt = 0.25f;    // Guard against a long gap blowing up the I and D terms

  if (!sensorValid) {  // Don't drive on a reading we don't trust
    winchStop();
    integral = 0.0f;
    return;
  }

  if (measured_lbs >= TENSION_HARD_LIMIT_LBS) {  // Over tension, back off toward slack
    integral = 0.0f;
    if (overTensionLatched) {
      winchStop();
      return;
    }
    if (overTensionStart == 0) {
      overTensionStart = now;
    } else if (now - overTensionStart > OVERTENSION_RELIEF_MS) {
      overTensionLatched = true;  // Relief isn't working, stay stopped until DCTS is cycled
      winchStop();
      return;
    }
    winchRelieve();
    return;
  }
  overTensionStart = 0;  // Back under the limit

  float error = (float)Target_Weight - measured_lbs;  // Positive means too loose

  if (error >= -TENSION_DEADBAND_LBS && error <= TENSION_DEADBAND_LBS) {  // Close enough, brake
    winchStop();
    prevError = error;  // Keep the derivative's history current while braked
    derivFiltered = 0.0f;
    return;
  }

  float pTerm = Kp * error;

  integral += error * dt;
  float iContribution = Ki * integral;
  if (iContribution > INTEGRAL_CLAMP_DUTY) {  // Clamp the contribution, then back out the integral
    iContribution = INTEGRAL_CLAMP_DUTY;
    if (Ki != 0.0f) integral = INTEGRAL_CLAMP_DUTY / Ki;
  }
  if (iContribution < -INTEGRAL_CLAMP_DUTY) {
    iContribution = -INTEGRAL_CLAMP_DUTY;
    if (Ki != 0.0f) integral = -INTEGRAL_CLAMP_DUTY / Ki;
  }

  float rawDeriv = (error - prevError) / dt;
  derivFiltered = DERIV_ALPHA * rawDeriv + (1.0f - DERIV_ALPHA) * derivFiltered;
  float dTerm = Kd * derivFiltered;

  prevError = error;

  driveWinch(pTerm + iContribution + dTerm);
}
```

**What it does, step by step:**

1. **Sample gating.** `mainControl()` sets `newSample = true` each time the HX711 produces a fresh reading, but only while `DCTS_EN` is set (L829–831). `DCTS()` advances the PID only then — once per load-cell sample — instead of on every `loop()` iteration. The I and D terms use the real elapsed `dt` (clamped to `[0.005 s, 0.25 s]` so link/sensor pauses can't corrupt a step). *Note on the rate:* the HX711's output data rate is set by the module's **RATE pin** (10 SPS or 80 SPS), **not** by the gain setting — `set_gain(32)` selects input channel B. A stock breakout ties RATE low, so ~10 Hz is the expected rate, but it should be confirmed rather than inferred from the gain.
2. **Safety: untrusted reading.** If the latest raw value failed the range check (`!sensorValid`), the winch is braked and the integrator dumped. The loop never drives on a reading it doesn't trust.
3. **Safety: over-tension.** At or above `TENSION_HARD_LIMIT_LBS` (185 lbs) the loop does *not* simply brake — a brake would leave the cable parked above the limit with nothing able to bring it down. Instead it calls `winchRelieve()` to creep toward slack at the stall floor. The attempt is time-boxed to `OVERTENSION_RELIEF_MS` (1.5 s): if tension hasn't dropped back under the limit by then, `overTensionLatched` is set and the winch is held off until the operator cycles DCTS with buttons '2' then '1'. The time-box exists because the direction convention is unverified (Issue #13) — if `Winch_DIR` is wired the other way round, the "relief" would pull harder, and the latch caps how long that can go on.
4. **Deadband.** `|error| ≤ TENSION_DEADBAND_LBS (2 lbs)` → brake. `prevError` is refreshed and `derivFiltered` zeroed on the way out, so leaving the deadband after a long brake doesn't produce a derivative spike. The integrator is deliberately left alone. **Corollary worth knowing when tuning:** because this branch returns before any integral accumulation, the I-term can never reduce a residual error that sits inside the deadband — the loop is simply off in there.
5. **P/I/D.** `error = Target_Weight − measured_lbs` (positive = too loose, needs tightening). P term: `Kp·error`. I term: accumulates `error·dt`; anti-windup clamps the I *contribution* to `±INTEGRAL_CLAMP_DUTY` (30 duty units) and back-computes `integral` when clamped. D term: on error, low-pass filtered by `DERIV_ALPHA = 0.15` (the load cell is noisy).
6. **Actuation.** `driveWinch()` maps the signed total effort to hardware. The sign picks `Winch_DIR`: effort > 0 (too loose) → LOW, effort < 0 (too tight) → HIGH, the same pin states v1.1 used. The magnitude is mapped **linearly** from `[0, EFFORT_FULL_SCALE]` onto the duty band `[WINCH_DUTY_MIN, WINCH_DUTY_MAX]` = `[12, 30]`; magnitudes under `EFFORT_DEADZONE` brake instead.

**Why the effort mapping is a linear map and not a clamp.** This is the one place the port deviates from the bench sketch, and it matters. The sketch (and the first version of this port) rounded the effort and then *clamped* it into `[12, 30]`. But 12 is a stall floor, so clamping meant every effort below 12 came out as exactly 12: at `Kp = 1.5`, all errors from the deadband edge up to ~8 lbs produced identical output. That is the range the loop actually operates in at steady state, so the controller stayed effectively bang-bang exactly where proportional action was supposed to help. Mapping across the band instead spends all 18 available duty counts on the error range in use:

| Error (lbs) | Effort (`Kp·error`) | Old duty (clamped) | Current duty (mapped) |
|-------------|--------------------|--------------------|-----------------------|
| ≤ 2.0 | — | 0 (deadband) | 0 (deadband) |
| 2.5 | 3.8 | 12 | 14 |
| 4.0 | 6.0 | 12 | 16 |
| 8.0 | 12.0 | 12 | 19 |
| 13.0 | 19.5 | 20 | 24 |
| ≥ 20.0 | ≥ 30.0 | 30 | 30 |

`EFFORT_FULL_SCALE = 30.0f` was chosen so the saturation point (~20 lbs of error at `Kp = 1.5`) is where the old code saturated too. The trade-off is that mid-range errors now command noticeably more duty than before, so **re-verify the response on the bench** — reducing `Kp` or raising `EFFORT_FULL_SCALE` softens it.

**Tunables (all in the PID block, L342–367):**

| Constant / gain | Shipped value | Meaning |
|-----------------|---------------|---------|
| `Kp` | `1.5f` | Proportional gain — PWM duty per lb of error |
| `Ki` | `0.0f` | Integral gain — PWM duty per lb·s (off until tuned) |
| `Kd` | `0.0f` | Derivative gain — PWM duty per lb/s (off until tuned) |
| `DERIV_ALPHA` | `0.15f` | Derivative low-pass alpha (noise) |
| `INTEGRAL_CLAMP_DUTY` | `30.0f` | Max magnitude of the I contribution (anti-windup) |
| `EFFORT_FULL_SCALE` | `30.0f` | Effort magnitude that commands `WINCH_DUTY_MAX` |
| `EFFORT_DEADZONE` | `0.5f` | Effort below this → brake rather than creep at the floor |
| `TENSION_DEADBAND_LBS` | `2.0f` | \|error\| within this → brake |
| `TENSION_HARD_LIMIT_LBS` | `185.0f` | At/over this → relieve toward slack, never exceed |
| `OVERTENSION_RELIEF_MS` | `1500` | How long relief may run before latching the winch off |
| `WINCH_DUTY_MIN` / `WINCH_DUTY_MAX` | `12` / `30` | Winch duty window (stall floor / quarter-power ceiling) |
| `RAW_MIN_VALID` / `RAW_MAX_VALID` | `-50000L` / `3000000L` | Raw reading validity window (placeholder — see Issue #12) |
| `QUANTA_PER_LB` | `10430.0f` | Load-cell calibration: quanta per pound |

**As shipped the loop is proportional-only** (`Ki = Kd = 0`). Tune on the bench — ideally first in the standalone sketch `WinchTensionPID_test_06_19_2026.ino` — with the usual progression: P-only until stable, a little I to kill steady-state offset, a little D to damp overshoot. Two things to keep in mind while doing it. First, the deadband branch returns before the integral is touched, so I only has authority on errors *outside* ±2 lbs; if you want tighter steady-state holding, shrink `TENSION_DEADBAND_LBS` rather than reaching for `Ki`. Second, there is no runtime tuning path on the machine — `Kp`/`Ki`/`Kd` are plain globals and nothing reads them from Serial, so each change means a re-flash. The bench sketch does accept live setpoint changes over Serial, which is why it's the better place to tune.

**What it replaced (the v1.1 bang-bang algorithm, kept here for reference):**

```cpp
// v1.1 DCTS() — removed by the PID migration
if (reading >= (TW_Adj + PLMI_Adj)) {        // above target + deadband
  temp_OCR5B = (reading - TW_Adj) / 10500;   // proportional-ish speed
  OCR5B = constrain((uint8_t)temp_OCR5B, 12, 30);
  digitalWrite(Winch_DIR, HIGH);             // "too tight" correction direction
} else if (reading <= (TW_Adj - PLMI_Adj)) { // below target - deadband
  temp_OCR5B = (TW_Adj - reading) / 10500;
  OCR5B = constrain((uint8_t)temp_OCR5B, 12, 30);
  digitalWrite(Winch_DIR, LOW);              // "too loose" correction direction
} else {                                     // inside deadband
  temp_OCR5B = 0;
  OCR5B = temp_OCR5B;                         // brake (no drive)
}
```

That version compared raw quanta against `TW_Adj ± PLMI_Adj`, scaled the error by the magic divisor `10500` (no longer in the code), and clamped duty to 12–30. It had no integral term, no derivative term, no filtering, no `millis()` timing, and no sensor-validity check.

### 5.2 Inputs the DCTS consumes (the feedback path)

| Input | What it is | Set where |
|-------|-----------|-----------|
| `newSample` | Latch that a fresh load-cell reading is available | `mainControl` L829–831: set on `scale.is_ready()`, but only while `DCTS_EN` |
| `reading` | Raw load-cell value (tension feedback), "quanta" | `mainControl` L822: `reading = scale.read()` |
| `sensorValid` / `measured_lbs` | Validity flag, and tension in lbs = `reading / QUANTA_PER_LB` (kept at last value when invalid) | `mainControl` L823–828: range check against `RAW_MIN_VALID`–`RAW_MAX_VALID`, then conversion |
| `Target_Weight` | Setpoint in **lbs** — from the radio pot, adopted clamped to `[10, 175]` (default 30) | `mainControl` L844–845 when a new `tensionSet` arrives |
| `dt` | Real elapsed time since the last PID step, clamped to `[0.005, 0.25]` s | `DCTS()` L423–427 from `millis()` / `lastPIDTime` |

### 5.3 Outputs the DCTS produces (the actuator path)

| Output | What it is | Consumed by |
|--------|-----------|-------------|
| `OCR5B` | Winch PWM duty register — within the PID path, written only by `driveWinch()` (12–30), `winchRelieve()` (12) and `winchStop()` (0) | Timer 5 ISRs (`TIMER5_COMPA/B_vect`, L917–925) which toggle `Winch_PWM` (pin 3) |
| `Winch_DIR` (pin 29) | Winch direction: HIGH = "too tight" correction branch, LOW = "too loose" branch (inherited from the v1.1 DCTS — see Issue #13) | The motor driver hardware |
| `integral` (state) | Accumulated I-term; persists between samples, dumped on the safety branches, left untouched in the deadband, and cleared wholesale by `resetPID()` | The next PID step |
| `overTensionStart` / `overTensionLatched` (state) | Over-tension relief timer and its give-up latch | The next PID step; the latch clears only via `resetPID()` |

So the full winch chain is: **`mainControl()` acquires the reading → `DCTS()` advances the PID on each fresh sample → `driveWinch()`/`winchRelieve()`/`winchStop()` write `OCR5B` + `Winch_DIR` → Timer 5 ISRs pulse `Winch_PWM` → motor driver moves the winch.**

### 5.4 What gates and surrounds the DCTS (control-flow dependencies)

- **`DCTS_EN`** (L64) is the on/off switch. It is set by remote **button '1'** (on) / **button '2'** (off) in `buttonSel()`.
- In `mainControl()` (L851–855), `DCTS_EN` chooses between `DCTS()` (the PID, auto) and `manualTensionControl()` (manual). **They are mutually exclusive and both own `OCR5B` and `Winch_DIR`.**
- **`manualTensionControl()`** (L493–501) is the manual twin: same outputs, constrained 0–150 instead of 12–30, direction taken from the joystick.
- The PID's setpoint is `Target_Weight` itself (lbs), adopted at L844–845. The quanta-converted `TW_Adj` is still recomputed at L846 but is no longer read by any control path.
- **`resetPID()` is called at every boundary where the winch changes hands**, so the loop can never resume on stale memory. There are six call sites: `setup()` (initial `dt` clock), `buttonSel()` on a genuine DCTS off→on transition and on every off, the setpoint-change branch, `emergency()` on the way out of the e-stop lock, and the link-lost failsafe.

### 5.5 Other code that writes the winch outputs

These also set `OCR5B`/`Winch_DIR`:
- `emergency()` → `manualTensionControl(0, 0)` (L190) — stop on e-stop.
- `mainControl()` link-lost branch (L875) — `manualTensionControl(0, 0)`.
- `manualController()` debug cases `'F'`/`'R'` (L587, L597) and `'S'` (L643) — only if the serial debug path is re-enabled.

Both the emergency and link-lost paths also drive `Chainsaw_EN` LOW. Turning the chainsaw on again always requires a new Button 5 command after either safety event.

**They do not simply "win" over the PID — the ownership state matters.** `emergency()` clears `DCTS_EN` before stopping the outputs, so recovery falls through to manual tension control with the zero-speed clear packet rather than automatically restarting the PID. `resetPID()` also clears stale loop state. The operator must deliberately press Button 1 to re-enable automatic tension after recovery. The link-lost branch is the `else` of `if (radioSent)`, so `DCTS()` cannot run in the same iteration.

Note: `winchStop()` only clears `OCR5B` (duty 0 = motor off); it leaves `Winch_DIR` at its last value. That is harmless while the duty is zero, but it means the direction pin can sit stale between PID stops.

### 5.6 What changed vs. v1.1 (and what stayed)

**Replaced:** the bang-bang body of `DCTS()` and everything it implied — the quanta-domain comparison against `TW_Adj ± PLMI_Adj`, the `10500` error divisor (now gone from the code), and the implicit `PLMI` deadband (replaced by the explicit 2 lb `TENSION_DEADBAND_LBS`).

**Kept (the PID reuses these):**
- `reading` / `scale.read()` — the same feedback signal, now range-checked and converted to lbs.
- The `Target_Weight` setpoint flow from the radio (10–175 clamp) — now the PID setpoint directly, in lbs.
- The Timer 5 PWM ISRs and `Winch_PWM` pin (the actuator hardware layer).
- The `DCTS_EN` vs. `manualTensionControl()` mode switch — the PID slots in exactly where `DCTS()` is called.
- The e-stop and link-lost zeroing of `OCR5B`, and the 12–30 quarter-power duty window.

**Added:** sample-gated execution with a real `dt`, sensor-validity gating, the 185 lb hard limit with time-boxed relief, the PID state variables (L369–378), anti-windup, derivative low-pass filtering, linear effort-to-duty mapping across the band, and the `winchStop()` / `winchRelieve()` / `resetPID()` / `driveWinch()` helpers.

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

### 6.2 The ISRs (L894–929)

| ISR | Action |
|-----|--------|
| `TIMER3_COMPA_vect` (L895–901) | If `OCR3B != 0`, drive all three traction PWM pins HIGH (start of period). |
| `TIMER3_COMPB_vect` (L902–906) | Drive all three traction PWM pins LOW (end of duty). |
| `TIMER4_COMPA_vect` (L907–911) | If `OCR4B != 0`, drive `Circumferential_PWM` HIGH. *(Comment says "Winch" but it's the circumferential pin — another stale comment.)* |
| `TIMER4_COMPB_vect` (L913–915) | Drive `Circumferential_PWM` LOW. |
| `TIMER5_COMPA_vect` (L917–921) | If `OCR5B != 0`, drive `Winch_PWM` HIGH. **(This is the winch / tension PWM — driven by the PID via `OCR5B`.)** |
| `TIMER5_COMPB_vect` (L923–925) | Drive `Winch_PWM` LOW. |
| `TIMER1_COMPA_vect` (L927–929) | Toggle `Stepper_PWM` each compare match → generates the stepper step pulses. |

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

**`CheckEmergencyButton()`** — Two-state e-stop:
- **Normal → emergency:** if the emergency button reads LOW, set `emergencyButtonState = true`, zero every motion speed, and send `b_EMERGENCY = 1`. The LCD warning still flashes, but another emergency packet is sent after each blocking flash delay so a lost first packet cannot leave the master unaware.
- **Emergency → recover:** if both the emergency button **and** Button 8 read LOW, clear `emergencyButtonState`, keep every motion speed at zero, and send `b_EMERGENCY = 0` with `buttonID = '8'`. The master exits only on this cleared emergency state, shows "SYSTEM RECOVERED", and requires Button 1 before automatic tension can run again.

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

**`loop()`** — Each iteration:
1. `CheckEmergencyButton()` first. If in emergency, zero all motion commands, transmit another `b_EMERGENCY = 1` packet, and return without processing normal controls.
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
| `TW_Adj` | `long int` | Target tension in **quanta** = `Target_Weight × 10430`. **Vestigial after the PID:** still recomputed in `mainControl` (L846) when the setpoint changes, but the PID compares in lbs and no control path reads it |
| `PLMI_Adj` | `long int` | Deadband in **quanta** = `PLMI × 10430`. **No longer used** — the PID deadband is `TENSION_DEADBAND_LBS` (lbs) |
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
| `measured_lbs` | `float` | Last valid load-cell reading converted to lbs (`reading / QUANTA_PER_LB`) — the PID's feedback (PID block, L370) |
| `sensorValid` | `bool` | True while the latest reading passed the `RAW_MIN_VALID`–`RAW_MAX_VALID` range check; false → PID brakes (L371) |
| `integral` | `float` | Accumulated I-term, with anti-windup clamping (L372, L466–473) |
| `prevError` | `float` | Previous error sample, used by the D-term (L373) |
| `derivFiltered` | `float` | Low-pass-filtered derivative, alpha `DERIV_ALPHA` (L374) |
| `newSample` | `bool` | Latch set by `mainControl` on a fresh load-cell read **while `DCTS_EN`**; `DCTS()` advances the PID only when set (L375, L418–421) |
| `lastPIDTime` | `unsigned long` | `millis()` of the last PID step — the source of the real `dt` (L376, started via `resetPID()` in `setup` L809) |
| `overTensionStart` | `unsigned long` | `millis()` the hard limit was first crossed; 0 while under the limit (L377) |
| `overTensionLatched` | `bool` | Set when relief has run for `OVERTENSION_RELIEF_MS` without clearing the fault; holds the winch off until `resetPID()` (L378) |

### 8.3 Key constants embedded in code

| Value | Meaning |
|-------|---------|
| `10430.0` | Load-cell calibration: **quanta per pound** — now the named constant `QUANTA_PER_LB` (L342); the ack-load byte still uses the bare literal, `reading / 10430.0` (L832) |
| `22998` | Quanta per kilogram (used only in commented debug) |
| ~~`10500`~~ | ~~DCTS error→PWM divisor ("1/7th slope" per v1.1 comment)~~ — **removed with the bang-bang algorithm; no longer in code** |
| `12 … 30` | Winch duty band (quarter-power limit) — now `WINCH_DUTY_MIN` / `WINCH_DUTY_MAX` (L343–344); `driveWinch()` maps effort linearly across it rather than clamping into it |
| `0 … 150` | Manual winch PWM clamp (`manualTensionControl`) |
| `0 … 180` | Circumferential PWM clamp |
| `0 … 193` | Traction PWM clamp |
| `194` | `OCRnA` PWM period for Timers 3/4/5 |
| `200` | `OCR1A` stepper period (Timer 1) |
| `0.0343 / 2` | Ultrasonic µs→cm conversion (sound 0.0343 cm/µs, ÷2 round trip) |
| `30000` | `pulseIn` echo timeout, µs (≈ 5 m max range) |
| `10`, `175` | Allowed tension-setpoint range (lbs) when adopting a new `tensionSet` |
| `176` | Upper bound check on incoming `tensionSet` |
| `2.0` | `TENSION_DEADBAND_LBS` — \|error\| within this (lbs) → PID brakes (L345) |
| `185.0` | `TENSION_HARD_LIMIT_LBS` — measured tension at/over this (lbs) → PID relieves toward slack, never exceeds (L346) |
| `-50000 … 3000000` | `RAW_MIN_VALID` / `RAW_MAX_VALID` — raw reading validity window in quanta (placeholder, verify — see Issue #12) (L347–348) |
| `1.5 / 0.0 / 0.0` | `Kp` / `Ki` / `Kd` — PID gains: PWM duty per lb, per lb·s, per lb/s (placeholders, tune on the bench) (L351–353) |
| `0.15` | `DERIV_ALPHA` — derivative low-pass alpha (L355) |
| `30.0` | `INTEGRAL_CLAMP_DUTY` — max magnitude of the I contribution (anti-windup) (L356) |
| `30.0` | `EFFORT_FULL_SCALE` — effort magnitude that commands `WINCH_DUTY_MAX` (L361) |
| `0.5` | `EFFORT_DEADZONE` — effort below this brakes instead of creeping at the stall floor (L362) |
| `1500` | `OVERTENSION_RELIEF_MS` — relief window before the over-tension latch trips (L367) |
| `0.005 / 0.25` | `dt` clamps in `DCTS()` — min/max seconds between PID steps (L426–427) |
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

6. **`emergency()` intentionally blocks normal machine processing** while `b_EMERGENCY` remains true. During the lock only `radioRX()` runs; sensors and the main loop are frozen. The controller continually retries zero-motion emergency packets and clears the state only through Emergency + Button 8. The master disables `DCTS_EN` and resets the PID, so automatic tension cannot restart without a deliberate Button 1 command after recovery.

7. **Blocking delays exist** in `loadCellInit()` (`delay(500)`), `rampMotorSpeed()` (`delayMicroseconds`), and on the remote in `CheckEmergencyButton()` (`delay(200)` ×10, `delay(2000)`). These run outside the steady-state control path but are worth noting for the project's "non-blocking" goal.

8. **`OCR5B` is written by the PID while Timer 5 ISRs read it.** The PID updates `OCR5B` from the main loop on every fresh load-cell sample (~10 Hz) while `TIMER5_COMPA_vect` reads it. Note that **`OCR5B` is a 16-bit register**, not 8-bit: Timers 1/3/4/5 on the ATmega2560 are all 16-bit, and the file's own `rampMotorSpeed(volatile uint16_t* OCRx, …)` (L282) reflects that. So a `OCR5B = …` store is *not* a single instruction. It is nevertheless safe as currently written, for a specific reason worth recording: AVR routes 16-bit `OCRnx` **writes** through the shared TEMP register so that the pair commits on the low-byte store, and the ISRs here only ever *read* `OCR5B` (reads of `OCRnx` do not use TEMP, so there is nothing for them to clobber). That safety argument breaks if an ISR is ever given a 16-bit register write, or if the duty were made to exceed 255 — either would warrant an `ATOMIC_BLOCK` around the store.

9. **Swapped joystick axis arguments** (remote, `ReadJoystick` → `JoystickLeftPWM`/`JoystickRightPWM`). This is a real behavioral subtlety, not just naming. `ReadJoystick()` calls `JoystickLeftPWM(leftJoystickyVal, leftJoystickxVal)` and `JoystickRightPWM(rightJoystickyVal, rightJoystickxVal)` — the **Y value is passed first**, into a parameter named `joystickxVal`. Since the function's first parameter drives the "X-named" packet field, the result is:
   - Left joystick **Y** → `winch_spd` (tension); left **X** → `traction_spd`.
   - Right joystick **Y** → `circum_spd`; right **X** → `radial_spd`.

   This is the **opposite axis** from what both the parameter names and the `#define` comments (e.g. "Left Joystick X ... For Tension") state. The code's actual behavior is as traced here; the comments are misleading. Confirm against the physical joysticks before relying on any axis label.

10. **`tensionSet` bounds.** `mainControl()` adopts a new setpoint only if `rxPkt.tensionSet != Target_Weight && rxPkt.tensionSet < 176`, then clamps to `[10, 175]`. The remote's pot maps 0–175. So a pot value of 0–9 would be clamped up to 10 on the machine side.

11. **PID gains are untuned placeholders.** As shipped, `Kp = 1.5` with `Ki = Kd = 0` (L351–353) — the loop is a plain proportional controller with a 2 lb deadband over a 12–30 duty band. Gain units are PWM duty per lb (Kp), per lb·s (Ki), per lb/s (Kd). Tune on the bench, ideally first in the standalone sketch `WinchTensionPID_test_06_19_2026.ino`, which accepts live setpoint changes over Serial; the master file has **no runtime tuning path**, so every gain change there costs a re-flash. When you get to `Ki`, note that the deadband branch returns before the integral is touched, so I has no authority on errors inside ±2 lbs — shrink `TENSION_DEADBAND_LBS` if you need tighter steady-state holding.

12. **The raw-reading validity window is a placeholder.** `RAW_MIN_VALID = -50000L` / `RAW_MAX_VALID = 3000000L` (L347–348) are guessed bounds, not measured load-cell limits. Verify the HX711's actual offset and max-load quanta against your calibration; while a reading is out of range, `sensorValid` goes false, the PID brakes, and `measured_lbs` holds its last value.

13. **Winch direction convention is inherited, not re-verified — and one safety path now depends on it.** `driveWinch()` reproduces the old DCTS pin behavior: `Winch_DIR` LOW = the "too loose" branch (L407), HIGH = the "too tight" branch (L409). The test sketch marks this VERIFY-on-bench: which physical motion (spool-in vs spool-out) each pin state produces depends on the motor driver wiring. This is no longer only a cosmetic question, because `winchRelieve()` (L384–387) drives the HIGH branch *on purpose* to escape an over-tension fault — if HIGH actually tightens, the relief would make the fault worse. That is exactly what `OVERTENSION_RELIEF_MS` and `overTensionLatched` bound, but **confirm the direction on the bench before trusting the hard limit.**

14. **Vestigial DCTS variables remain.** `TW_Adj` is recomputed on every setpoint change (L846) but is no longer read by any control path, and `PLMI_Adj` (L74) plus `PLMI` (L41) are unused entirely — the PID deadband is `TENSION_DEADBAND_LBS` (lbs). Harmless, but candidates for removal in a cleanup pass.

15. **`ultraSonic()` sits inside the tension feedback path.** In `mainControl()` the ultrasonic ping is fired in the same `scale.is_ready()` block as the load-cell read (L833), *before* `DCTS()` runs. `pulseIn(Ultrasonic_ECHO, HIGH, 30000)` can block for up to 30 ms, so at a ~10 Hz sample rate the reading the PID acts on may already be up to ~30% of a sample period stale, and `dt` picks up the corresponding jitter. Not a correctness bug — `dt` is measured, not assumed — but moving the ping out of the tension path would tighten the loop.

16. **The load-cell slope is written twice.** `QUANTA_PER_LB` (L342) and the bare literal in `ackArr[0] = reading / 10430.0` (L832) are the same calibration constant in two places. If the rig is ever recalibrated, both must change. `ackArr[0]` is also a `uint8_t`, so the reported tension wraps above 255 lbs — only reachable in a fault, given the 175 lb setpoint ceiling, but it means the remote's display can't be trusted to show an over-range condition.

17. **The effort-to-duty mapping is new and needs bench confirmation.** `driveWinch()` maps effort linearly across `[12, 30]` rather than clamping into it, which fixed the loop being effectively bang-bang below ~8 lbs of error (see [5.1](#51-the-algorithm-pid-block-l335482)). The saturation point is unchanged, but **mid-range errors now command noticeably more duty than the v1.1 code did** — roughly 16–24 where the old code gave 12–20. Verify the response on the bench; lower `Kp` or raise `EFFORT_FULL_SCALE` to soften it.

---

*End of documentation. This document describes the working-tree code after the winch-tension PID migration; line numbers refer to the current master control file (929 lines). The wireless-controller file is unchanged and its references are as of its v1.1 upload.*
