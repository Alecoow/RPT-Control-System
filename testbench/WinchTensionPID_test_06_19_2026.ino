// ============================================================================
//  Winch Tension PID — Standalone Test Sketch (Arduino Mega 2560)
//
//  Drives only the winch and reads only the HX711 load cell, so the tension
//  PID can be tuned on the bench before it runs on the machine.
//
//    Feedback : cable tension from the load cell, in lbs
//    Setpoint : SETPOINT_LBS, changeable over Serial
//    Error    : setpoint - measured (positive = too loose)
//    Output   : PID size -> winch duty (OCR5B), PID sign -> Winch_DIR
//
//  The PID steps once per new load cell sample, using the real time between
//  samples as dt. Nothing in loop() blocks.
// ============================================================================

#include "HX711.h"

// ============================================================================
//  PINS (same as the master control file)
// ============================================================================

#define DT_PIN   17   // HX711 data
#define SCK_PIN  18   // HX711 clock

#define Winch_PWM  3    // Driven by the Timer 5 ISRs at the bottom
// HIGH = correction for too tight, LOW = correction for too loose, same as the
// master. Which way each one physically spools depends on the driver wiring.
// VERIFY on the bench.
#define Winch_DIR  29

// ============================================================================
//  CALIBRATION
// ============================================================================

// HX711 counts per pound. VERIFY against your own calibration.
const float QUANTA_PER_LB = 10430.0f;

// ============================================================================
//  WINCH OUTPUT LIMITS
// ============================================================================
// The winch only moves for duties inside this band:
//   WINCH_DUTY_MIN = lowest duty that turns the winch (below it, it stalls)
//   WINCH_DUTY_MAX = quarter power ceiling, so the cable is never yanked
const uint8_t WINCH_DUTY_MIN = 12;
const uint8_t WINCH_DUTY_MAX = 30;

// Within this many lbs of the setpoint we brake, so noise doesn't make the winch chatter
const float TENSION_DEADBAND_LBS = 2.0f;

// ============================================================================
//  PID GAINS (placeholders, tune on the bench)
// ============================================================================
// Start with P only. Once that is stable, add Ki to remove steady offset, then
// a little Kd to damp overshoot.
float Kp = 1.5f;    // Duty per lb
float Ki = 0.0f;    // Duty per lb-second
float Kd = 0.0f;    // Duty per lb/second

// ============================================================================
//  PID / LOOP STATE
// ============================================================================

// Desired tension in lbs. Type a new value in the Serial Monitor to change it.
float SETPOINT_LBS = 30.0f;
const float SETPOINT_MIN_LBS = 10.0f;   // Same range the master accepts
const float SETPOINT_MAX_LBS = 175.0f;

// If the HX711 stops sending samples for this long, stop the winch
const unsigned long SENSOR_TIMEOUT_MS = 250;
unsigned long lastControlTime = 0;
unsigned long lastSampleTime = 0;

const unsigned long PRINT_PERIOD_MS = 200;    // Debug print rate, 5 Hz
unsigned long lastPrintTime = 0;

HX711 scale;

long  reading       = 0;       // Tare-corrected load cell counts
float measured_lbs  = 0.0f;    // Tension in lbs
bool  sensorValid   = false;   // False when the last reading was rejected

float integral      = 0.0f;    // Accumulated error, lb-seconds
float prevError     = 0.0f;    // Last error, for the derivative
float derivFiltered = 0.0f;    // Smoothed derivative

// Derivative smoothing, 0 to 1. Lower is smoother but slower. The load cell is
// noisy, so this is fairly low.
const float DERIV_ALPHA = 0.15f;

// Largest duty the integral term may add, so it can't build up while the output is maxed
const float INTEGRAL_CLAMP_DUTY = (float)WINCH_DUTY_MAX;

// Readings outside this window are rejected. A disconnected or saturated HX711
// returns extreme values. PLACEHOLDERS, set from logged data on your rig.
const long RAW_MIN_VALID = -50000L;
const long RAW_MAX_VALID =  3000000L;

// At or above this tension the winch stops. PLACEHOLDER, set to the cable's safe limit.
const float TENSION_HARD_LIMIT_LBS = 185.0f;

// Set by the 's' command. Holds the winch off until 'r' is sent.
bool emergencyStop = false;

// ============================================================================
//  SENSOR READING
// ============================================================================
bool readSensor() {  // Returns true when a new sample was read
  if (!scale.is_ready()) {
    return false;
  }

  long taredReading = (long)scale.get_value(1);

  if (taredReading >= RAW_MIN_VALID && taredReading <= RAW_MAX_VALID) {
    reading      = taredReading;
    measured_lbs = (float)taredReading / QUANTA_PER_LB;
    sensorValid  = true;
  } else {
    sensorValid = false;  // Keep the last good measured_lbs
  }

  return true;
}

// ============================================================================
//  SERIAL COMMANDS
//    <number> + Enter : new setpoint in lbs
//    z                : reset the integral
//    s                : emergency stop
//    r                : clear the emergency stop
// ============================================================================
void handleInput() {
  if (Serial.available() > 0) {
    int c = Serial.peek();

    if (c == 's' || c == 'S') {
      Serial.read();
      emergencyStop = true;
      winchStop();
      Serial.println(F(">> EMERGENCY STOP latched. Send 'r' to clear."));
    } else if (c == 'r' || c == 'R') {
      Serial.read();
      emergencyStop = false;
      integral = 0.0f;
      Serial.println(F(">> Emergency cleared."));
    } else if (c == 'z' || c == 'Z') {
      Serial.read();
      integral = 0.0f;
      Serial.println(F(">> Integrator reset."));
    } else {
      float val = Serial.parseFloat();
      if (val >= SETPOINT_MIN_LBS && val <= SETPOINT_MAX_LBS) {
        SETPOINT_LBS = val;
        integral = 0.0f;          // Don't carry the old integral into a new setpoint
        Serial.print(F(">> New setpoint (lbs): "));
        Serial.println(SETPOINT_LBS);
      } else {
        Serial.print(F(">> Setpoint out of range ["));
        Serial.print(SETPOINT_MIN_LBS); Serial.print(F(".."));
        Serial.print(SETPOINT_MAX_LBS); Serial.println(F("] — ignored."));
      }
    }
    // Drop the trailing newline, otherwise it is read as 0 next loop
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r' || Serial.peek() == ' ')) {
      Serial.read();
    }
  }
}

// ============================================================================
//  PID
//  Returns a signed effort in duty units: positive = tighten, negative = release.
// ============================================================================
float computePID(float dt) {
  float error = SETPOINT_LBS - measured_lbs;

  float pTerm = Kp * error;

  // Limit how much duty the integral can add, then back the integral off to match
  integral += error * dt;
  float iContribution = Ki * integral;
  if (iContribution >  INTEGRAL_CLAMP_DUTY) { iContribution =  INTEGRAL_CLAMP_DUTY; integral = (Ki != 0.0f) ?  INTEGRAL_CLAMP_DUTY / Ki : 0.0f; }
  if (iContribution < -INTEGRAL_CLAMP_DUTY) { iContribution = -INTEGRAL_CLAMP_DUTY; integral = (Ki != 0.0f) ? -INTEGRAL_CLAMP_DUTY / Ki : 0.0f; }

  // Derivative, smoothed because the load cell is noisy
  float rawDeriv = (error - prevError) / dt;
  derivFiltered  = DERIV_ALPHA * rawDeriv + (1.0f - DERIV_ALPHA) * derivFiltered;
  float dTerm    = Kd * derivFiltered;

  prevError = error;

  return pTerm + iContribution + dTerm;
}

// ============================================================================
//  WINCH OUTPUT
// ============================================================================
void winchStop() {
  OCR5B = 0;                 // Duty 0, the Timer 5 ISRs then hold Winch_PWM low
}

void driveWinch(float effort) {  // Signed effort -> direction and duty
  if (effort == 0.0f) { winchStop(); return; }

  if (effort > 0.0f) {
    digitalWrite(Winch_DIR, LOW);    // Too loose
  } else {
    digitalWrite(Winch_DIR, HIGH);   // Too tight
    effort = -effort;
  }

  // Round, then keep it inside the band. Any move needs at least the stall floor.
  uint16_t duty = (uint16_t)(effort + 0.5f);
  if (duty < WINCH_DUTY_MIN) duty = WINCH_DUTY_MIN;
  if (duty > WINCH_DUTY_MAX) duty = WINCH_DUTY_MAX;

  OCR5B = (uint8_t)duty;
}

// ============================================================================
//  CONTROL STEP
//  Safety checks come first and can override the PID.
// ============================================================================
void applyControl(float dt) {
  if (emergencyStop) { winchStop(); return; }

  if (!sensorValid) { winchStop(); return; }  // Don't drive on a bad reading

  if (measured_lbs >= TENSION_HARD_LIMIT_LBS) {
    winchStop();
    integral = 0.0f;
    return;
  }

  // Close enough, brake
  float error = SETPOINT_LBS - measured_lbs;
  if (error >= -TENSION_DEADBAND_LBS && error <= TENSION_DEADBAND_LBS) {
    winchStop();
    return;
  }

  float effort = computePID(dt);
  driveWinch(effort);
}

// ============================================================================
//  DEBUG OUTPUT
// ============================================================================
void printDebug() {
  Serial.print(F("SET="));     Serial.print(SETPOINT_LBS, 1);
  Serial.print(F("  MEAS="));  Serial.print(measured_lbs, 1);
  Serial.print(F("  ERR="));   Serial.print(SETPOINT_LBS - measured_lbs, 1);
  Serial.print(F("  TARED=")); Serial.print(reading);
  Serial.print(F("  DUTY="));  Serial.print(OCR5B);
  Serial.print(F("  DIR="));   Serial.print(digitalRead(Winch_DIR) ? F("HIGH(too-tight corr)") : F("LOW(too-loose corr)"));
  Serial.print(F("  I="));     Serial.print(integral, 2);
  Serial.print(F("  Dq="));    Serial.print(derivFiltered, 2);
  Serial.print(F("  VALID=")); Serial.print(sensorValid ? F("Y") : F("N"));
  if (emergencyStop) Serial.print(F("  [ESTOP]"));
  Serial.println();
}

// ============================================================================
//  HARDWARE SETUP (same settings as the master, so the winch behaves the same)
// ============================================================================
void loadCellInit() {
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_gain(32);          // Gain 32 reads HX711 channel B
  delay(500);                  // Let the HX711 settle
  scale.tare(20);              // Zero over 20 samples
}

void winchPWMTimerInit() {
  // Timer 5, CTC mode. The ISRs below turn the pin on and off to make the PWM.
  TCCR5A = 0;

  TCCR5B &= ~((1 << WGM53) | (1 << WGM52) | (1 << WGM51) | (1 << WGM50));
  TCCR5B &= ~((1 << CS52)  | (1 << CS51)  | (1 << CS50));

  // CTC mode, prescaler 64
  TCCR5B |= (1 << WGM52) | (1 << CS51) | (1 << CS50);

  OCR5A = 194;   // PWM period
  OCR5B = 0;     // Duty, 0 = off

  TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);

  sei();
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
  lastSampleTime  = lastControlTime;
  lastPrintTime   = lastControlTime;
}

// ============================================================================
//  MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  handleInput();

  if (readSensor()) {
    unsigned long sampleTime = millis();
    float dt = (float)(sampleTime - lastControlTime) / 1000.0f;
    lastControlTime = sampleTime;
    lastSampleTime = sampleTime;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.25f) dt = 0.25f;
    applyControl(dt);
  } else if (now - lastSampleTime > SENSOR_TIMEOUT_MS) {
    sensorValid = false;
    integral = 0.0f;
    winchStop();  // The sensor went quiet, don't keep driving
  }

  if (now - lastPrintTime >= PRINT_PERIOD_MS) {
    lastPrintTime = now;
    printDebug();
  }
}

// ============================================================================
//  TIMER 5 ISRs — winch PWM
//  COMPA at the start of each period turns the pin on (unless duty is 0).
//  COMPB at the duty count turns it off.
// ============================================================================
ISR(TIMER5_COMPA_vect) {
  if (OCR5B != 0) {
    digitalWrite(Winch_PWM, HIGH);
  }
}

ISR(TIMER5_COMPB_vect) {
  digitalWrite(Winch_PWM, LOW);
}
