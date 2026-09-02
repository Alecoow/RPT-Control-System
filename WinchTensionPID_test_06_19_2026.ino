// ============================================================================
//  Winch Tension PID — Standalone Test Sketch
//  Target: Arduino Mega 2560
//  Purpose: Isolate and bench-test the cable-tension control loop by itself.
//           Replaces the old bang-bang DCTS() with a real PID controller.
//
//  This sketch ONLY drives the winch motor and reads the HX711 load cell.
//  No radio, no other motors. This is dev-order step 6 (test winch PID alone).
//
//  Control summary:
//    Feedback : measured cable tension from the load cell (converted to lbs)
//    Setpoint : SETPOINT_LBS below (adjustable live over Serial — see loop notes)
//    Error    : setpoint - measured   (positive = too loose, need to tighten)
//    Output   : PID magnitude -> winch PWM duty (OCR5B), sign -> Winch_DIR
//
//  Structure (sections, in execution order each cycle):
//    1. Sensor reading
//    2. Input handling (serial setpoint changes)
//    3. PID calculation
//    4. Safety checks
//    5. Actuator output
//    6. Serial debug output
//
//  NON-BLOCKING: the control loop is timed with millis(). No delay() is used
//  inside the loop. (delay() appears only once, in load-cell init at startup.)
// ============================================================================

#include "HX711.h"

// ============================================================================
//  PIN DEFINITIONS  (must match mastercontrolfile_quarter_powered_DCTS)
// ============================================================================

// HX711 load-cell amplifier
#define DT_PIN   17   // HX711 data (DOUT)
#define SCK_PIN  18   // HX711 clock

// Winch motor (Timer 5 generates its PWM; see PWMTimerInit + ISRs at bottom)
#define Winch_PWM  3    // PWM output pin toggled by Timer 5 ISRs
// Direction pin convention is taken EXACTLY from the original DCTS():
//   When measured tension is ABOVE target (too tight) DCTS drove Winch_DIR HIGH.
//   When measured tension is BELOW target (too loose) DCTS drove Winch_DIR LOW.
// So in this sketch:  HIGH = the "too tight" correction direction,
//                     LOW  = the "too loose" correction direction.
// (Whether "too tight" physically means spool-out vs spool-in depends on your
//  winch wiring — this sketch simply reproduces DCTS's pin behavior so the
//  motor turns the same way it does today. VERIFY on the bench.)
#define Winch_DIR  29

// ============================================================================
//  CALIBRATION CONSTANTS
// ============================================================================

// Load-cell calibration: how many raw HX711 counts ("quanta") equal one pound.
// Taken from the existing master file (reading / 10430.0 -> lbs).
// >>> VERIFY against your own calibration before trusting absolute lbs. <<<
const float QUANTA_PER_LB = 10430.0f;

// ============================================================================
//  WINCH OUTPUT LIMITS  (PLACEHOLDER-ish — proven values from DCTS)
// ============================================================================
// The winch PWM duty (OCR5B) only produces motion within this band:
//   WINCH_DUTY_MIN = lowest duty that actually moves the winch under load
//                    (below this the motor just stalls/buzzes)
//   WINCH_DUTY_MAX = safe "quarter power" ceiling so the cable is never yanked
// These match the constrain(…, 12, 30) used in the original DCTS().
// Tune on the bench if needed; keep MAX conservative for safety.
const uint8_t WINCH_DUTY_MIN = 12;   // stall floor
const uint8_t WINCH_DUTY_MAX = 30;   // quarter-power ceiling

// Tension deadband (lbs): if |error| is within this, command zero (brake).
// Prevents the winch from chattering around the setpoint on load-cell noise.
const float TENSION_DEADBAND_LBS = 2.0f;

// ============================================================================
//  PID GAINS  (PLACEHOLDERS — start conservative, tune on the bench)
// ============================================================================
// Units: output is in "PWM duty per lb of error" before clamping to the band.
// Start with P only (Ki = Kd = 0), get a stable proportional response, then
// add Ki to kill steady-state offset, then a little Kd to damp overshoot.
float Kp = 1.5f;    // proportional gain  (PWM duty per lb)
float Ki = 0.0f;    // integral gain      (PWM duty per lb·s)
float Kd = 0.0f;    // derivative gain    (PWM duty per lb/s)

// ============================================================================
//  PID / LOOP STATE
// ============================================================================

// Setpoint — desired cable tension in pounds. Change live by typing a number
// in the Serial Monitor (see input handling section). Range-checked on entry.
float SETPOINT_LBS = 30.0f;
const float SETPOINT_MIN_LBS = 10.0f;   // matches master file's 10..175 clamp
const float SETPOINT_MAX_LBS = 175.0f;

// Fixed control period. The PID math assumes a constant dt, so we run the loop
// on a fixed schedule instead of "as fast as possible".
const unsigned long CONTROL_PERIOD_MS = 20;   // 50 Hz control loop
const float DT_S = CONTROL_PERIOD_MS / 1000.0f;
unsigned long lastControlTime = 0;

// Serial print throttle (don't spam every 20 ms)
const unsigned long PRINT_PERIOD_MS = 200;    // 5 Hz prints
unsigned long lastPrintTime = 0;

// HX711 object
HX711 scale;

// Measured values
long  reading       = 0;       // raw load-cell counts (quanta)
float measured_lbs  = 0.0f;    // converted tension in pounds
bool  sensorValid   = false;   // set false when a reading is rejected

// PID internal memory
float integral      = 0.0f;    // accumulated integral term (in lbs·s)
float prevError     = 0.0f;    // previous error, for the derivative term
float derivFiltered = 0.0f;    // low-pass-filtered derivative (noise control)

// Derivative low-pass filter coefficient (0..1).
// Smaller = more smoothing (more lag); larger = more responsive (more noise).
// The load cell is noisy, so we filter the D term fairly hard.
const float DERIV_ALPHA = 0.15f;

// Integral anti-windup: clamp the integral's *contribution* (Ki*integral) so a
// saturated output can't keep winding the integral up. Expressed in PWM duty.
const float INTEGRAL_CLAMP_DUTY = (float)WINCH_DUTY_MAX;

// Sensor validation bounds (raw quanta). A real reading should fall in a sane
// window; HX711 returns extreme/min/max-int values when disconnected or saturated.
// >>> Adjust these to your rig's real min/max once you've logged some data. <<<
const long RAW_MIN_VALID = -50000L;       // PLACEHOLDER — below this = reject
const long RAW_MAX_VALID =  3000000L;     // PLACEHOLDER — above this = reject

// Safety: maximum believable tension. If measured tension exceeds this, we stop
// the winch (do not keep pulling) — protects cable/hardware.
// >>> Set to your cable/mechanism's safe limit. <<<
const float TENSION_HARD_LIMIT_LBS = 185.0f;   // PLACEHOLDER

// Emergency-stop latch (no physical button in this isolated test; can be set by
// the safety checks or, on the real machine, by the e-stop packet). Once true,
// the winch is forced off until the sketch is reset.
bool emergencyStop = false;

// ============================================================================
//  SECTION 1 — SENSOR READING
// ============================================================================
void readSensor() {
  if (scale.is_ready()) {
    long raw = scale.read();

    // Sensor validation: reject obviously bad readings.
    if (raw >= RAW_MIN_VALID && raw <= RAW_MAX_VALID) {
      reading      = raw;
      measured_lbs = (float)raw / QUANTA_PER_LB;
      sensorValid  = true;
    } else {
      // Keep last good 'measured_lbs' but flag invalid so safety can react.
      sensorValid = false;
    }
  }
  // If not ready, we simply keep the previous values this cycle (non-blocking).
}

// ============================================================================
//  SECTION 2 — INPUT HANDLING
//  Allows changing the tension setpoint live by typing a number + Enter in the
//  Serial Monitor. Type 'z' to zero/reset the integrator. Type 's' to STOP
//  (latch emergency), 'r' to clear emergency.
// ============================================================================
void handleInput() {
  if (Serial.available() > 0) {
    // Peek first char to decide between command letters and a numeric setpoint.
    int c = Serial.peek();

    if (c == 's' || c == 'S') {
      Serial.read();
      emergencyStop = true;
      Serial.println(F(">> EMERGENCY STOP latched. Send 'r' to clear."));
    } else if (c == 'r' || c == 'R') {
      Serial.read();
      emergencyStop = false;
      integral = 0.0f;            // reset integrator on recovery
      Serial.println(F(">> Emergency cleared."));
    } else if (c == 'z' || c == 'Z') {
      Serial.read();
      integral = 0.0f;
      Serial.println(F(">> Integrator reset."));
    } else {
      // Treat the rest as a numeric setpoint in lbs.
      float val = Serial.parseFloat();
      if (val >= SETPOINT_MIN_LBS && val <= SETPOINT_MAX_LBS) {
        SETPOINT_LBS = val;
        integral = 0.0f;          // reset integrator on setpoint change (bumpless-ish)
        Serial.print(F(">> New setpoint (lbs): "));
        Serial.println(SETPOINT_LBS);
      } else {
        Serial.print(F(">> Setpoint out of range ["));
        Serial.print(SETPOINT_MIN_LBS); Serial.print(F(".."));
        Serial.print(SETPOINT_MAX_LBS); Serial.println(F("] — ignored."));
      }
    }
    // Flush any trailing newline/whitespace so it isn't parsed as 0 next loop.
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r' || Serial.peek() == ' ')) {
      Serial.read();
    }
  }
}

// ============================================================================
//  SECTION 3 — PID CALCULATION
//  Returns a SIGNED control effort in PWM-duty units:
//    positive => tighten (retract),  negative => release.
//  Error convention: error = setpoint - measured.
//    too loose (measured < setpoint) -> error positive -> tighten. Correct.
// ============================================================================
float computePID() {
  float error = SETPOINT_LBS - measured_lbs;

  // ---- Proportional ----
  float pTerm = Kp * error;

  // ---- Integral (with conditional anti-windup) ----
  // Tentatively accumulate, then clamp the integral *contribution*.
  integral += error * DT_S;
  float iContribution = Ki * integral;
  // Clamp the integral's effect so it can't wind past full output.
  if (iContribution >  INTEGRAL_CLAMP_DUTY) { iContribution =  INTEGRAL_CLAMP_DUTY; integral = (Ki != 0.0f) ?  INTEGRAL_CLAMP_DUTY / Ki : 0.0f; }
  if (iContribution < -INTEGRAL_CLAMP_DUTY) { iContribution = -INTEGRAL_CLAMP_DUTY; integral = (Ki != 0.0f) ? -INTEGRAL_CLAMP_DUTY / Ki : 0.0f; }

  // ---- Derivative (on error, low-pass filtered for noisy load cell) ----
  float rawDeriv = (error - prevError) / DT_S;
  derivFiltered  = DERIV_ALPHA * rawDeriv + (1.0f - DERIV_ALPHA) * derivFiltered;
  float dTerm    = Kd * derivFiltered;

  prevError = error;

  return pTerm + iContribution + dTerm;
}

// ============================================================================
//  SECTION 5 — ACTUATOR OUTPUT
//  Translate a signed PID effort into (direction pin) + (PWM duty in band).
//  Helper for both the PID path and the safety "stop" path.
// ============================================================================
void winchStop() {
  OCR5B = 0;                 // duty 0 -> Timer5 ISR holds Winch_PWM low (motor off)
}

void driveWinch(float effort) {
  // Deadband: tiny efforts -> stop (avoid chatter on noise).
  // Note: error deadband is applied in applyControl(); this also guards 0.
  if (effort == 0.0f) { winchStop(); return; }

  // Direction from sign — matched to original DCTS pin behavior.
  //   error = setpoint - measured:
  //     effort > 0  => too loose (measured < setpoint). DCTS used Winch_DIR LOW.
  //     effort < 0  => too tight (measured > setpoint). DCTS used Winch_DIR HIGH.
  if (effort > 0.0f) {
    digitalWrite(Winch_DIR, LOW);    // "too loose" correction (DCTS LOW branch)
  } else {
    digitalWrite(Winch_DIR, HIGH);   // "too tight" correction (DCTS HIGH branch)
    effort = -effort;                // use magnitude for duty
  }

  // Map magnitude into the usable duty band [WINCH_DUTY_MIN .. WINCH_DUTY_MAX].
  // Any nonzero command must be at least the stall floor to actually move.
  uint16_t duty = (uint16_t)(effort + 0.5f);   // round
  if (duty < WINCH_DUTY_MIN) duty = WINCH_DUTY_MIN;
  if (duty > WINCH_DUTY_MAX) duty = WINCH_DUTY_MAX;

  OCR5B = (uint8_t)duty;
}

// ============================================================================
//  SECTION 4 + 5 — SAFETY CHECKS then ACTUATOR
//  Decides the final winch command, with safety overriding the PID.
// ============================================================================
void applyControl() {
  // --- Safety check 1: emergency latch ---
  if (emergencyStop) { winchStop(); return; }

  // --- Safety check 2: invalid sensor -> don't drive blind ---
  if (!sensorValid) { winchStop(); return; }

  // --- Safety check 3: over-tension hard limit ---
  if (measured_lbs >= TENSION_HARD_LIMIT_LBS) {
    winchStop();                         // stop pulling; do not exceed limit
    integral = 0.0f;                     // dump integrator
    return;
  }

  // --- Error deadband: close enough -> brake, hold integrator ---
  float error = SETPOINT_LBS - measured_lbs;
  if (error >= -TENSION_DEADBAND_LBS && error <= TENSION_DEADBAND_LBS) {
    winchStop();
    return;
  }

  // --- Normal PID actuation ---
  float effort = computePID();
  driveWinch(effort);
}

// ============================================================================
//  SECTION 6 — SERIAL DEBUG OUTPUT (throttled, formatted for tuning)
// ============================================================================
void printDebug() {
  Serial.print(F("SET="));     Serial.print(SETPOINT_LBS, 1);
  Serial.print(F("  MEAS="));  Serial.print(measured_lbs, 1);
  Serial.print(F("  ERR="));   Serial.print(SETPOINT_LBS - measured_lbs, 1);
  Serial.print(F("  RAW="));   Serial.print(reading);
  Serial.print(F("  DUTY="));  Serial.print(OCR5B);
  Serial.print(F("  DIR="));   Serial.print(digitalRead(Winch_DIR) ? F("HIGH(too-tight corr)") : F("LOW(too-loose corr)"));
  Serial.print(F("  I="));     Serial.print(integral, 2);
  Serial.print(F("  Dq="));    Serial.print(derivFiltered, 2);
  Serial.print(F("  VALID=")); Serial.print(sensorValid ? F("Y") : F("N"));
  if (emergencyStop) Serial.print(F("  [ESTOP]"));
  Serial.println();
}

// ============================================================================
//  HARDWARE INIT — load cell + Timer 5 PWM  (copied from master file so the
//  winch is driven identically). Timer 5 in CTC mode; ISRs toggle Winch_PWM.
// ============================================================================
void loadCellInit() {
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_gain(32);          // matches master file
  delay(500);                  // one-time startup stabilization (not in loop)
  scale.tare(20);              // zero the scale over 20 samples
}

void winchPWMTimerInit() {
  // Timer 5 -> Winch PWM. Reproduces the master file's setup EXACTLY.
  TCCR5A = 0;

  // Clear WGM and CS bits
  TCCR5B &= ~((1 << WGM53) | (1 << WGM52) | (1 << WGM51) | (1 << WGM50));
  TCCR5B &= ~((1 << CS52)  | (1 << CS51)  | (1 << CS50));

  // CTC mode (WGM52), prescaler = 64 (CS51 + CS50)  [matches master file bits]
  TCCR5B |= (1 << WGM52) | (1 << CS51) | (1 << CS50);

  OCR5A = 194;   // PWM period
  OCR5B = 0;     // duty 0 = off

  TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);   // enable compare-match interrupts

  sei();         // global interrupt enable
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(Winch_PWM, OUTPUT);
  pinMode(Winch_DIR, OUTPUT);
  digitalWrite(Winch_DIR, LOW);

  loadCellInit();
  winchPWMTimerInit();

  Serial.println(F("=== Winch Tension PID test ==="));
  Serial.println(F("Type a number (lbs) to set tension. 's'=stop, 'r'=recover, 'z'=zero integrator."));
  Serial.print(F("Initial setpoint (lbs): "));
  Serial.println(SETPOINT_LBS, 1);

  lastControlTime = millis();
  lastPrintTime   = millis();
}

// ============================================================================
//  MAIN LOOP  — non-blocking, fixed-rate control via millis()
// ============================================================================
void loop() {
  unsigned long now = millis();

  // Input handling can run every loop (cheap, responsive).
  handleInput();

  // Fixed-rate control cycle.
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime += CONTROL_PERIOD_MS;   // schedule next (no drift)

    readSensor();      // Section 1
    applyControl();    // Sections 4 + 3 + 5 (safety -> PID -> actuator)
  }

  // Throttled debug output.
  if (now - lastPrintTime >= PRINT_PERIOD_MS) {
    lastPrintTime = now;
    printDebug();      // Section 6
  }
}

// ============================================================================
//  TIMER 5 ISRs — generate the winch PWM (identical to master file)
//  COMPA fires at start of period: drive pin HIGH if duty != 0.
//  COMPB fires at the duty count:  drive pin LOW.
// ============================================================================
ISR(TIMER5_COMPA_vect) {
  if (OCR5B != 0) {
    digitalWrite(Winch_PWM, HIGH);
  }
}

ISR(TIMER5_COMPB_vect) {
  digitalWrite(Winch_PWM, LOW);
}
