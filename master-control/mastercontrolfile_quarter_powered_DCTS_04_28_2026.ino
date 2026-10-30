// Main control file, runs on the machine.
// Receives commands from the wireless controller, drives every motor, runs the
// tension PID and the radial range hold, and sends load and range back.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "HX711.h"
#include "printf.h"

//////////////////
// Pin definitions

// HX711 Pins
#define DT_PIN 17   // HX711 data
#define SCK_PIN 18  // HX711 clock

// Traction Wheel Pins
#define TractionWheel1_PWM 6
#define TractionWheel2_PWM 5
#define TractionWheel3_PWM 4
#define TractionWheel1_DIR 23
#define TractionWheel2_DIR 25
#define TractionWheel3_DIR 27

// Winch Motor Pins
#define Winch_PWM 3
#define Winch_DIR 29

// Circumferential Motor Pins
#define Circumferential_PWM 45
#define Circumferential_DIR 31

// Stepper Motor Pins
// The board labels these C1 and C2, VERIFY which is step and which is direction
#define Stepper_PWM 37  // Step pulses
#define Stepper_DIR 35

// Chainsaw Pin
#define Chainsaw_EN 39  // HIGH = on

// Load Cell Definitions
#define PLMI 5  // Unused

// Ultrasonic Sensor Pin Def
#define Ultrasonic_ECHO 20
#define Ultrasonic_TRIG 19

// RF pins
#define CE_PIN 49
#define CSN_PIN 53

//////////////////

unsigned long lastReceivedTime = 0;
const unsigned long connectionTimeout = 200;  // ms without a packet before the link counts as lost

uint8_t Target_Weight = 30;  // Tension setpoint in lbs, set from the controller

// Ultrasonic sensor
const unsigned long ULTRA_PERIOD_MS = 60;      // Time between pings, lets old echoes die out
const unsigned long ULTRA_TIMEOUT_US = 15000;  // ~257 cm max, also the longest pulseIn can stall the loop
const uint8_t ULTRA_MIN_CM = 2;                // Shorter readings aren't reliable
const uint8_t ULTRA_MAX_MISSES = 3;            // Missed echoes in a row before the range is invalid
unsigned long ultra_dur;                       // Echo pulse width, us
unsigned long lastPingTime = 0;
unsigned long lastRangeTime = 0;  // millis() of the last valid range
uint16_t rangeHistory[3];         // Last valid readings, for the median filter
uint8_t rangeCount = 0;
uint8_t rangeIdx = 0;
uint8_t rangeMisses = 0;
uint16_t rangeCm = 0;         // Filtered distance to the trunk
bool rangeValid = false;      // False until the first reading, and after too many misses
bool rangeNewSample = false;  // Set on each valid reading, cleared by rangeHold()

// HX711 & load cell variables
HX711 scale;
bool DCTS_EN = 0;          // Automatic tension (PID) on
long reading = 0;          // Tare-corrected load cell counts
short int counter = 0;     // Unused
uint8_t duty_percent = 0;  // Unused
uint8_t temp_OCR3B = 0;    // Speed before it is clamped into OCR3B
uint8_t temp_OCR4B = 0;    // Same for OCR4B
uint8_t temp_OCR5B = 0;    // Same for OCR5B

// Old setpoint and deadband in load cell counts. Unused by the PID.
long int TW_Adj = Target_Weight * (long)10430;
long int PLMI_Adj = PLMI * (long)10430;

// RF Variables

/* Remote buttons 1-8
  bool b_DCTS_ON;
  bool b_DCTS_OFF;
  bool b_SET_TENSION;
  bool b_DISP_TENSION;
  bool b_CHAIN_ON;
  bool b_CHAIN_OFF;
  bool b_CHAIN_DIST_SET;
  bool b_CHAIN_DIST_DISP;
*/

// Must match the struct in the wireless controller exactly
struct ControllerPacket {
  char buttonID;
  bool b_EMERGENCY;
  uint8_t winch_spd;
  bool winch_dir;
  uint8_t traction_spd;
  bool traction_dir;
  uint8_t circum_spd;
  bool circum_dir;
  uint8_t radial_spd;
  bool radial_dir;
  uint8_t tensionSet;
  uint8_t rangeSet;  // Range hold setpoint in cm, 0 = hold off
};

bool isValid(const ControllerPacket& p) {  // Packet check, not implemented yet
  return true;
}

// Radio Initialization
RF24 radio(CE_PIN, CSN_PIN);
const byte address[5] = { 'T', 'E', 'S', 'T', '1' };  // Must match the controller
ControllerPacket rxPkt;
ControllerPacket tempPkt;
bool radioSent = 0;

// Send back tension (lbs), distance (cm, 255 = no reading), and status bits
uint8_t ackArr[3] = { 0, 255, 0 };
#define ACK_RANGE_HOLD 0x01   // Range hold is running
#define ACK_RANGE_FAULT 0x02  // Range hold stopped itself, see rangeHoldFault()

// Used by the commented-out traction direction delay
bool currDIR = 0;
uint16_t dirCTR = 0;

// Debug
unsigned long startTimer;
char testchar = '0';  // Command letter for manualController()

void loadCellInit() {
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_gain(32);  // Gain 32 reads HX711 channel B
  // Serial.println("Initializing HX711...");
  delay(500);      // Let the HX711 settle
  scale.tare(20);  // Zero over 20 samples
  // Serial.print("Tare offset: ");
  // Serial.println(scale.get_offset());  // Display the tare offset value
}

void loadCellDebug() {  // Prints load cell values, uncomment to use
  //reading = (long)scale.get_value(1);
  // Serial.print("\n\nTare-corrected Reading: ");
  // Serial.println(reading);  // Display the digital reading as a decimal
  // Serial.print("\n\nTarget Weight: ");
  // Serial.println(TW_Adj);
  // Serial.print("\nWeight (lbs): ");
  // Serial.println(reading / 10430.0);  // About 10430 counts per lb
  // Serial.print("\nWeight (kg): ");
  // Serial.println(reading / 22998.0);
  //manDelay(2000000);
  //delay(100);
}

void pinInit() {
  pinMode(TractionWheel1_PWM, OUTPUT);
  pinMode(TractionWheel2_PWM, OUTPUT);
  pinMode(TractionWheel3_PWM, OUTPUT);
  pinMode(TractionWheel1_DIR, OUTPUT);
  pinMode(TractionWheel2_DIR, OUTPUT);
  pinMode(TractionWheel3_DIR, OUTPUT);

  pinMode(Winch_PWM, OUTPUT);
  pinMode(Winch_DIR, OUTPUT);

  pinMode(Circumferential_PWM, OUTPUT);
  pinMode(Circumferential_DIR, OUTPUT);

  pinMode(Stepper_PWM, OUTPUT);
  pinMode(Stepper_DIR, OUTPUT);

  pinMode(Chainsaw_EN, OUTPUT);
  digitalWrite(Chainsaw_EN, LOW);  // Chainsaw starts off

  pinMode(Ultrasonic_TRIG, OUTPUT);
  pinMode(Ultrasonic_ECHO, INPUT);
  digitalWrite(Ultrasonic_TRIG, LOW);
}

void radioInit() {  // Settings must match the controller
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(76);
  radio.openReadingPipe(1, address);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setAutoAck(true);
  radio.startListening();
}

void emergency() {  // Stop everything and wait until the controller clears the emergency
  // Serial.println("EMERGENCY PRESSED");
  DCTS_EN = 0;  // Button 1 is needed again after recovery
  manualCircumControl(0, 0);
  manualTractControl(0, 0);
  manualTensionControl(0, 0);
  rangeHoldOff();
  digitalWrite(Chainsaw_EN, LOW);
  while (rxPkt.b_EMERGENCY) {  // Only a packet with b_EMERGENCY = 0 gets out
    radioRX();
  }
  resetPID();  // The load cell wasn't read while locked
}

// Motor PWM is made in software: each timer runs in CTC mode, its COMPA ISR turns
// the pin on at the start of the period and its COMPB ISR turns it off at OCRnB.
// OCRnA is the period, OCRnB the duty.
void PWMTimerInit() {
  // Timer 3, traction wheels, prescaler 1024
  TCCR3A = 0;
  TCCR3B &= ~((1 << WGM33) | (1 << WGM32) | (1 << WGM31) | (1 << WGM30));
  TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));
  TCCR3B |= (1 << WGM32) | (1 << CS32) | (1 << CS30);

  OCR3A = 194;
  OCR3B = 0;

  TIMSK3 |= (1 << OCIE3A) | (1 << OCIE3B);

  // Timer 4, circumferential motor, prescaler 1024
  TCCR4A = 0;
  TCCR4B &= ~((1 << WGM43) | (1 << WGM42) | (1 << WGM41) | (1 << WGM40));
  TCCR4B &= ~((1 << CS42) | (1 << CS41) | (1 << CS40));
  TCCR4B |= (1 << WGM42) | (1 << CS42) | (1 << CS40);

  OCR4A = 194;
  OCR4B = 0;

  TIMSK4 |= (1 << OCIE4A) | (1 << OCIE4B);

  // Timer 5, winch, prescaler 64
  TCCR5A = 0;
  TCCR5B &= ~((1 << WGM53) | (1 << WGM52) | (1 << WGM51) | (1 << WGM50));
  TCCR5B &= ~((1 << CS52) | (1 << CS51) | (1 << CS50));
  TCCR5B |= (1 << WGM52) | (1 << CS51) | (1 << CS50);

  OCR5A = 194;
  OCR5B = 0;

  TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);

  // Timer 1, stepper, prescaler 1024. Its ISR toggles the step pin.
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);

  // 16 MHz / 1024 / 201 = ~77.7 toggles/s, so ~39 steps/s. Lower is faster.
  OCR1A = 200;

  TIMSK1 &= ~(1 << OCIE1A);  // Off until the stepper is told to move
  //TIMSK1 |= (1 << OCIE1A);

  sei();
}

void radioRX() {  // Read a packet if one arrived and queue the reply
  if (radio.available()) {
    radioSent = 1;
    lastReceivedTime = millis();
    radio.read(&tempPkt, sizeof(tempPkt));
    if (isValid(tempPkt)) {
      rxPkt = tempPkt;
    }
    radio.writeAckPayload(1, ackArr, sizeof(ackArr));
    //debugRX();
  }
}

void rampMotorSpeed(volatile uint16_t* OCRx, uint8_t currentSpeed, uint8_t targetSpeed, uint8_t rampStep, uint16_t rampDelay_us) {
  if (currentSpeed < targetSpeed) {
    // Ramp Up
    for (uint8_t spd = currentSpeed; spd <= targetSpeed; spd += rampStep) {
      *OCRx = spd;
      delayMicroseconds(rampDelay_us);
    }
  } else {
    // Ramp Down
    for (uint8_t spd = currentSpeed; spd >= targetSpeed; spd -= rampStep) {
      *OCRx = spd;
      delayMicroseconds(rampDelay_us);
      if (spd < rampStep) break;  // Stop before spd wraps below 0
    }
  }
}

void debugRX() {  // Prints the received packet, uncomment to use
  // Serial.print(F("Button Pressed: "));
  // Serial.println(rxPkt.buttonID);
  // Serial.print(F("Emergency Status: "));
  // Serial.println(rxPkt.b_EMERGENCY);

  // Serial.print(F("Winch Speed: "));
  // Serial.println(rxPkt.winch_spd);
  // Serial.print(F("Winch Direction: "));
  // Serial.println(rxPkt.winch_dir);

  // Serial.print(F("Traction Speed: "));
  // Serial.println(rxPkt.traction_spd);
  // Serial.print(F("Traction Direction: "));
  // Serial.println(rxPkt.traction_dir);

  // Serial.print(F("Circumferential Speed: "));
  // Serial.println(rxPkt.circum_spd);
  // Serial.print(F("Circumferential Direction: "));
  // Serial.println(rxPkt.circum_dir);

  // Serial.print(F("Radial Speed: "));
  // Serial.println(rxPkt.radial_spd);
  // Serial.print(F("Radial Direction: "));
  // Serial.println(rxPkt.radial_dir);
}

void manDelay(int del) {  // Busy-wait, unused
  int i = 0;
  while (i < del) {
    i++;
  }
}

//////////////////
// Winch Tension PID
// Compares the load cell (lbs) with Target_Weight and drives the winch toward it:
// the size of the PID output sets the duty (OCR5B), its sign sets Winch_DIR.
// Runs once per new load cell sample, using the real time between samples as dt.

const float QUANTA_PER_LB = 10430.0f;         // Load cell counts per lb
const uint8_t WINCH_DUTY_MIN = 12;            // Below this the winch stalls
const uint8_t WINCH_DUTY_MAX = 30;            // Quarter power ceiling
const float TENSION_DEADBAND_LBS = 2.0f;      // Within this many lbs of the setpoint, brake
const float TENSION_HARD_LIMIT_LBS = 185.0f;  // Never pull past this
// Readings outside this window are rejected. PLACEHOLDERS, set from logged data on the rig.
const long RAW_MIN_VALID = -50000L;
const long RAW_MAX_VALID = 3000000L;

// PID gains. Tune on the bench, ideally in WinchTensionPID_test first.
float Kp = 1.5f;  // Duty per lb
float Ki = 0.0f;  // Duty per lb-second
float Kd = 0.0f;  // Duty per lb/second

const float DERIV_ALPHA = 0.15f;          // Derivative smoothing, 0 to 1, lower is smoother
const float INTEGRAL_CLAMP_DUTY = 30.0f;  // Largest duty the integral term can add

// PID output from 0 to EFFORT_FULL_SCALE is spread across the whole 12-30 duty band.
// At Kp = 1.5 the winch reaches full duty at about 20 lbs of error.
const float EFFORT_FULL_SCALE = 30.0f;
const float EFFORT_DEADZONE = 0.5f;  // Smaller outputs brake instead of creeping

// Over the hard limit the winch slowly lets cable out so the tension can come back
// down. It only does this for OVERTENSION_RELIEF_MS, in case Winch_DIR is reversed
// and it is actually tightening.
const unsigned long OVERTENSION_RELIEF_MS = 1500;

// PID state
float measured_lbs = 0.0f;           // Tension in lbs
bool sensorValid = false;            // False when the last reading was rejected
float integral = 0.0f;               // Accumulated error, lb-seconds
float prevError = 0.0f;              // Last error, for the derivative
float derivFiltered = 0.0f;          // Smoothed derivative
bool newSample = false;              // Set by mainControl() on each new load cell reading
unsigned long lastPIDTime = 0;       // millis() of the last PID step
unsigned long overTensionStart = 0;  // millis() the hard limit was first hit, 0 when under it
bool overTensionLatched = false;     // Relief timed out, winch held off until resetPID()

void winchStop() {  // Duty 0, the Timer 5 ISRs then hold Winch_PWM low
  OCR5B = 0;
}

void winchRelieve() {  // Slowly let cable out
  digitalWrite(Winch_DIR, HIGH);
  OCR5B = WINCH_DUTY_MIN;
}

void resetPID() {  // Clear PID state, call whenever the PID starts or the setpoint changes
  integral = 0.0f;
  prevError = 0.0f;
  derivFiltered = 0.0f;
  newSample = false;
  lastPIDTime = millis();
  overTensionStart = 0;
  overTensionLatched = false;
}

void driveWinch(float effort) {  // Signed PID output -> direction and duty
  float mag = (effort < 0.0f) ? -effort : effort;
  if (mag < EFFORT_DEADZONE) {
    winchStop();
    return;
  }

  if (effort > 0.0f) {
    digitalWrite(Winch_DIR, LOW);   // Too loose
  } else {
    digitalWrite(Winch_DIR, HIGH);  // Too tight
  }

  float frac = mag / EFFORT_FULL_SCALE;
  if (frac > 1.0f) frac = 1.0f;
  OCR5B = (uint8_t)(WINCH_DUTY_MIN + frac * (WINCH_DUTY_MAX - WINCH_DUTY_MIN) + 0.5f);
}

void DCTS() {  // Tension PID, one step per new load cell sample (~10 Hz)
  if (!newSample) {
    return;
  }
  newSample = false;

  unsigned long now = millis();
  float dt = (float)(now - lastPIDTime) / 1000.0f;
  lastPIDTime = now;
  if (dt < 0.005f) dt = 0.005f;  // Avoid dividing by ~0
  if (dt > 0.25f) dt = 0.25f;    // A long gap would make the I and D terms jump

  if (!sensorValid) {  // Don't drive on a bad reading
    winchStop();
    integral = 0.0f;
    return;
  }

  if (measured_lbs >= TENSION_HARD_LIMIT_LBS) {  // Over tension, let cable out
    integral = 0.0f;
    if (overTensionLatched) {
      winchStop();
      return;
    }
    if (overTensionStart == 0) {
      overTensionStart = now;
    } else if (now - overTensionStart > OVERTENSION_RELIEF_MS) {
      overTensionLatched = true;  // Relief isn't working, stay stopped until Button 2 then Button 1
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
    prevError = error;  // So the derivative doesn't jump when we leave the deadband
    derivFiltered = 0.0f;
    return;
  }

  float pTerm = Kp * error;

  integral += error * dt;
  float iContribution = Ki * integral;
  if (iContribution > INTEGRAL_CLAMP_DUTY) {  // Limit the integral's duty, then back the integral off to match
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

void manualCircumControl(int speed, bool dir) { 
  temp_OCR4B = speed;
  OCR4B = constrain((uint8_t)temp_OCR4B, 0, 180);
  if (dir) {
    digitalWrite(Circumferential_DIR, LOW);
  } else {
    digitalWrite(Circumferential_DIR, HIGH);
  }
}
void manualTensionControl(int speed, bool dir) { 
  temp_OCR5B = speed;
  OCR5B = constrain((uint8_t)temp_OCR5B, 0, 150);
  if (dir) {
    digitalWrite(Winch_DIR, HIGH);
  } else {
    digitalWrite(Winch_DIR, LOW);
  }
}

void manualTractControl(int speed, bool dir) {
  // temp_OCR3B = speed;
  // OCR3B = constrain((uint8_t)temp_OCR3B, 0, 100);
  // if (dir) {
  //   digitalWrite(TractionWheel1_DIR, HIGH);
  //   digitalWrite(TractionWheel2_DIR, HIGH);
  //   digitalWrite(TractionWheel3_DIR, HIGH);
  // } else {
  //   digitalWrite(TractionWheel1_DIR, LOW);
  //   digitalWrite(TractionWheel2_DIR, LOW);
  //   digitalWrite(TractionWheel3_DIR, LOW);
  // }
  // if (currDIR != dir) {  // When direction is changed, wait 1000 loop cycles before you can move it
  //   OCR3B = 0;
  //   if (dirCTR < 1000) {
  //     dirCTR++;
  //     if (DCTS_EN) {
  //       dirCTR += 100;
  //     }
  //     //// Serial.println(dirCTR);
  //   } else {
  //     currDIR = dir;
  //     dirCTR = 0;
  //   }
  // } else {
  //rampMotorSpeed(&OCR3B, OCR3B, constrain((uint8_t)speed, 0, 130), 1, 3000);
  temp_OCR3B = speed;
  OCR3B = constrain((uint8_t)temp_OCR3B, 0, 193);
  if (dir) {
    digitalWrite(TractionWheel1_DIR, LOW);
    digitalWrite(TractionWheel2_DIR, LOW);
    digitalWrite(TractionWheel3_DIR, LOW);
  } else {
    digitalWrite(TractionWheel1_DIR, HIGH);
    digitalWrite(TractionWheel2_DIR, HIGH);
    digitalWrite(TractionWheel3_DIR, HIGH);
  }
  //}
}

void manualRadialControl(bool on, bool dir) {
  if (on) {
    TIMSK1 |= (1 << OCIE1A);
    if (dir) {
      digitalWrite(Stepper_DIR, HIGH);
    } else {
      digitalWrite(Stepper_DIR, LOW);
    }
  } else {
    TIMSK1 &= ~(1 << OCIE1A);
  }
}

//////////////////
// Radial Range Hold
// Keeps the stepper at rangeSet cm from the trunk using the ultrasonic reading.
// The stepper has one speed, so it just starts and stops: it starts once the
// distance is more than RANGE_DEADBAND_CM off, and stops when it reaches the setpoint.
// There are no limit switches, so a move that stops getting closer is a fault.

const uint8_t RANGE_SET_MIN_CM = 5;      // Must match the controller's Set Range limits
const uint8_t RANGE_SET_MAX_CM = 99;
const uint8_t RANGE_DEADBAND_CM = 2;     // How far off before the stage moves
const unsigned long RANGE_STALE_MS = 300;  // No valid reading for this long stops the hold

// The manualRadialControl() dir value that moves toward the trunk. VERIFY by jogging
// with the joystick. If it is wrong, the hold moves away and faults after a few cm.
const bool RADIAL_DIR_TOWARD_TREE = true;

// While moving, the error must shrink by RANGE_PROGRESS_CM every RANGE_PROGRESS_MS,
// or the stage counts as stalled or at the end of its travel.
// PLACEHOLDER, depends on how fast the stage moves.
const unsigned long RANGE_PROGRESS_MS = 5000;
const uint8_t RANGE_PROGRESS_CM = 1;
const uint8_t RANGE_WRONG_WAY_CM = 3;  // If the error grows this much while moving, it's going the wrong way

uint8_t Target_Range = 0;       // Active setpoint, cm
bool rangeHoldActive = false;
bool rangeFault = false;        // Stays set until rangeSet changes
bool rangeSetPrimed = false;    // False until the first packet after boot
uint8_t lastRangeSet = 0;       // Last rangeSet received, the hold reacts when it changes
int8_t radialMotion = 0;        // 1 toward the trunk, -1 away, 0 stopped
uint16_t moveStartAbsErr = 0;   // |error| when the current move began
uint16_t bestAbsErr = 0;        // Smallest |error| seen during the current move
unsigned long lastProgressTime = 0;

void radialStop() {
  radialMotion = 0;
  manualRadialControl(0, 0);
}

void rangeHoldOff() {  // Stop the stage and end the hold
  rangeHoldActive = false;
  radialStop();
}

void rangeHoldFault() {
  rangeFault = true;
  rangeHoldOff();
}

void radialMove(int8_t motion, uint16_t absErr) {
  radialMotion = motion;
  moveStartAbsErr = absErr;
  bestAbsErr = absErr;
  lastProgressTime = millis();
  manualRadialControl(1, (motion > 0) ? RADIAL_DIR_TOWARD_TREE : !RADIAL_DIR_TOWARD_TREE);
}

void rangeHold() {
  unsigned long now = millis();
  if (!rangeValid || now - lastRangeTime > RANGE_STALE_MS) {  // Never move blind
    rangeHoldFault();
    return;
  }
  if (!rangeNewSample) {
    return;
  }
  rangeNewSample = false;

  int error = (int)rangeCm - (int)Target_Range;  // Positive means too far from the trunk
  uint16_t absErr = abs(error);

  if (radialMotion != 0) {
    bool reached = (radialMotion > 0) ? (error <= 0) : (error >= 0);
    if (reached) {  // Stop. If it overshot, it reverses on a later sample, never directly.
      radialStop();
      return;
    }
    if (absErr >= moveStartAbsErr + RANGE_WRONG_WAY_CM) {
      rangeHoldFault();
      return;
    }
    if (absErr + RANGE_PROGRESS_CM <= bestAbsErr) {
      bestAbsErr = absErr;
      lastProgressTime = now;
    } else if (now - lastProgressTime > RANGE_PROGRESS_MS) {
      rangeHoldFault();
    }
    return;
  }

  if (error > RANGE_DEADBAND_CM) {
    radialMove(1, absErr);
  } else if (error < -RANGE_DEADBAND_CM) {
    radialMove(-1, absErr);
  }
}

void rangeSetUpdate() {  // Start, retarget, or cancel the hold when rangeSet changes
  if (!rangeSetPrimed) {  // After a reboot, don't start on a setpoint the controller was already sending
    rangeSetPrimed = true;
    lastRangeSet = rxPkt.rangeSet;
    return;
  }
  if (rxPkt.rangeSet == lastRangeSet) {
    return;
  }
  lastRangeSet = rxPkt.rangeSet;
  rangeFault = false;
  radialStop();
  if (rxPkt.rangeSet >= RANGE_SET_MIN_CM && rxPkt.rangeSet <= RANGE_SET_MAX_CM) {
    Target_Range = rxPkt.rangeSet;
    rangeHoldActive = true;
    rangeNewSample = false;  // Wait for a reading taken after the new setpoint
  } else {
    rangeHoldActive = false;  // 0 or out of range cancels
  }
}

void radialControl() {
  if (rxPkt.radial_spd > 20) {  // The joystick always wins and ends the hold
    rangeHoldActive = false;
    radialMotion = 0;
    manualRadialControl(1, rxPkt.radial_dir);
  } else if (rangeHoldActive) {
    rangeHold();
  } else {
    radialStop();
  }
}

// Bench testing without the controller: one letter per action, read from Serial.
// E emergency, O/P/I traction, F/R winch, Q/W stepper, T/Y circumferential,
// S stop all, C/N chainsaw on/off. Uncomment the Serial read at the bottom to use.
void manualController() {
  switch (testchar) {
    case 'E':
      // Serial.println("E Pressed.");
      emergency();
      break;

    case 'O':
      // Serial.println("O Pressed.");
      manualTractControl(50, 1);
      break;

    case 'P':
      // Serial.println("P Pressed.");
      manualTractControl(50, 0);
      break;

    case 'I':
      // Serial.println("I Pressed.");
      manualTractControl(0, 0);
      break;

    case 'F':
      // Serial.println("F Pressed.");
      //manualTractControl(50, 1);
      //TIMSK5 |= (1 << OCIE5A);
      manualTensionControl(100, 1);
      //manualCircumControl(100, 1);
      //manualRadialControl(1, 1);
      break;

    case 'R':
      // Serial.println("R Pressed.");
      //manualTractControl(50, 0);
      //TIMSK5 |= (1 << OCIE5A);
      manualTensionControl(100, 0);
      //manualCircumControl(100, 0);
      //manualRadialControl(1, 0);
      break;

    case 'Q':
      // Serial.println("Q Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      //manualCircumControl(100, 0);
      manualRadialControl(1, 0);
      break;

    case 'W':
      // Serial.println("W Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      //manualCircumControl(100, 0);
      manualRadialControl(1, 1);
      break;

    case 'T':
      // Serial.println("T Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      manualCircumControl(100, 0);
      //manualRadialControl(1, 0);
      break;

    case 'Y':
      // Serial.println("Y Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      manualCircumControl(100, 1);
      //manualRadialControl(1, 0);
      break;

    case 'S':
      // Serial.println("S Pressed.");
      //TIMSK5 &= ~(1 << OCIE5A);
      manualTractControl(0, 0);
      manualTensionControl(0, 0);
      manualTractControl(0, 0);
      manualCircumControl(0, 0);
      manualRadialControl(0, 0);
      break;

    case 'C':
      // Serial.println("E Pressed.");
      digitalWrite(Chainsaw_EN, HIGH);
      //DCTS_EN = 1;
      break;

    case 'N':
      // Serial.println("E Pressed.");
      //DCTS_EN = 0;
      digitalWrite(Chainsaw_EN, LOW);
      break;

    case 'Z':
      // Serial.println("E Pressed.");
      //DCTS_EN = 0;
      //TIMSK4 &= ~(1 << OCIE4A);
      //TIMSK5 &= ~(1 << OCIE5A);
      break;

    case 'X':
      // Serial.println("E Pressed.");
      //DCTS_EN = 0;
      //TIMSK4 |= (1 << OCIE4A);
      //TIMSK5 |= (1 << OCIE5A);
      break;

    default:
      break;
  }


  /*if ( Serial.available() > 0) {
    testchar = toUpperCase(// Serial.read());
  }*/
}


/*void readDebug() {  // Send a typed number to the controller as a fake load value
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    int val = s.toInt();
    ackArr[0] = val;
    radio.writeAckPayload(1, ackArr, sizeof(ackArr));
    // Serial.print("ACK set to: ");
    // Serial.println(ackArr[0]);
  }
}*/

void buttonSel() {  // Act on the button the controller reported
  switch (rxPkt.buttonID) {
    case '1':  // Automatic tension on
      // Serial.println("1 Pressed.");
      if (!DCTS_EN) {
        resetPID();
      }
      DCTS_EN = 1;
      break;

    case '2':  // Automatic tension off
      // Serial.println("2 Pressed.");
      DCTS_EN = 0;
      resetPID();
      break;

    // 3, 4, 7, 8 only change screens on the controller. Setpoints arrive in
    // tensionSet and rangeSet instead.
    case '3':
      // Serial.println("3 Pressed.");

      break;

    case '4':
      // Serial.println("4 Pressed.");
      // reading = (long)scale.get_value(1);
      // ackArr[0] = reading / 10430.0;
      break;

    case '5':  // Chainsaw on
      // Serial.println("5 Pressed.");
      digitalWrite(Chainsaw_EN, HIGH);
      break;

    case '6':  // Chainsaw off
      // Serial.println("6 Pressed.");
      digitalWrite(Chainsaw_EN, LOW);
      break;

    case '7':
      // Serial.println("7 Pressed.");

      break;

    case '8':
      // Serial.println("8 Pressed.");

      break;

    default:
      break;
  }
}

void printControllerDebug() {  // Prints every joystick value, uncomment to use
  radioRX();
  // Serial.print(F("Winch Speed: "));
  // Serial.println(rxPkt.winch_spd);
  // Serial.print(F("Winch Direction: "));
  // Serial.println(rxPkt.winch_dir ? "Forward" : "Reverse");

  // Serial.print(F("Traction Speed: "));
  // Serial.println(rxPkt.traction_spd);
  // Serial.print(F("Traction Direction: "));
  // Serial.println(rxPkt.traction_dir ? "Forward" : "Reverse");

  // Serial.print(F("Circumferential Speed: "));
  // Serial.println(rxPkt.circum_spd);
  // Serial.print(F("Circumferential Direction: "));
  // Serial.println(rxPkt.circum_dir ? "Forward" : "Reverse");

  // Serial.print(F("Radial Speed: "));
  // Serial.println(rxPkt.radial_spd);
  // Serial.print(F("Radial Direction: "));
  // Serial.println(rxPkt.radial_dir ? "Forward" : "Reverse");

  // Serial.println();
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    uint16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) b = c;
  return (a > b) ? a : b;
}

void ultraSonic() {  // One ping, updates rangeCm and rangeValid
  digitalWrite(Ultrasonic_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(Ultrasonic_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultrasonic_TRIG, LOW);
  ultra_dur = pulseIn(Ultrasonic_ECHO, HIGH, ULTRA_TIMEOUT_US);
  uint16_t cm = (uint16_t)(ultra_dur * 0.0343f / 2.0f);
  // Serial.println(ultra_dur);
  // Serial.println(cm);

  if (ultra_dur == 0 || cm < ULTRA_MIN_CM) {  // 0 means no echo, not 0 cm
    if (rangeMisses < 255) rangeMisses++;
    if (rangeMisses >= ULTRA_MAX_MISSES) {
      rangeValid = false;
      rangeCount = 0;  // Start the filter fresh when the echo comes back
    }
    return;
  }
  rangeMisses = 0;

  // Median of the last 3 readings, ignores a single bad echo
  rangeHistory[rangeIdx] = cm;
  rangeIdx = (rangeIdx + 1) % 3;
  if (rangeCount < 3) rangeCount++;
  rangeCm = (rangeCount < 3) ? cm : median3(rangeHistory[0], rangeHistory[1], rangeHistory[2]);
  rangeValid = true;
  rangeNewSample = true;
  lastRangeTime = millis();
}

void setup() {
  Serial.begin(115200);
  radioInit();
  Serial.println("Done Initializing.");
  pinInit();
  loadCellInit();
  PWMTimerInit();
  resetPID();  // Start the PID's dt clock

  printf_begin();
  radio.printPrettyDetails();
}

void mainControl() {  // One pass: read radio and sensors, then drive everything
  radioRX();
  if (millis() - lastReceivedTime > connectionTimeout) {
    radioSent = 0;  // Lost connection
  }

  if (scale.is_ready()) {
    reading = (long)scale.get_value(1);  // Tare-corrected
    if (reading >= RAW_MIN_VALID && reading <= RAW_MAX_VALID) {
      sensorValid = true;
      measured_lbs = (float)reading / QUANTA_PER_LB;
    } else {
      sensorValid = false;  // The PID brakes on this
    }
    if (DCTS_EN) {  // Only flag samples the PID will use, so it never acts on an old one
      newSample = true;
    }
    ackArr[0] = reading / 10430.0;
    //// Serial.println(ackArr[0]);
  }

  // Separate from the load cell so the ping doesn't delay the tension PID
  if (millis() - lastPingTime >= ULTRA_PERIOD_MS) {
    lastPingTime = millis();
    ultraSonic();
    ackArr[1] = rangeValid ? (uint8_t)min(rangeCm, (uint16_t)254) : 255;
  }
  //// Serial.println(radioSent);
  if (radioSent) {  // Link is up

    if (rxPkt.b_EMERGENCY) {
      emergency();
    }
    // 0 means no setpoint confirmed yet. Out of range values are ignored. They must
    // not be clamped first, or the clamped value never matches and resetPID() runs every pass.
    if (rxPkt.tensionSet >= 10 && rxPkt.tensionSet <= 175 && rxPkt.tensionSet != Target_Weight) {
      Target_Weight = rxPkt.tensionSet;
      TW_Adj = Target_Weight * (long)10430;
      resetPID();
      // Serial.print("New tension set: ");
      // Serial.println(Target_Weight);
    }
    if (DCTS_EN) {
      DCTS();
    } else {
      manualTensionControl(rxPkt.winch_spd, rxPkt.winch_dir);
    }

    buttonSel();

    manualCircumControl(rxPkt.circum_spd, rxPkt.circum_dir);
    manualTractControl(rxPkt.traction_spd, rxPkt.traction_dir);
    rangeSetUpdate();
    radialControl();

    // if (rxPkt.winch_spd < rxPkt.circum_spd) {
    //   manualCircumControl(rxPkt.circum_spd, rxPkt.circum_dir);
    // }
    // if (rxPkt.traction_spd > rxPkt.radial_spd) {
    //   manualTractControl(rxPkt.traction_spd, rxPkt.traction_dir);
    // } else {
    //   manualRadialControl(((rxPkt.radial_spd > 20)), rxPkt.radial_dir);
    // }
  } else {
    // Link is down: stop everything. Automatic tension needs Button 1 again and the
    // range hold needs a new confirmed range.
    DCTS_EN = 0;
    manualCircumControl(0, 0);
    manualTractControl(0, 0);
    manualTensionControl(0, 0);
    rangeHoldOff();
    digitalWrite(Chainsaw_EN, LOW);
    resetPID();
  }

  ackArr[2] = (rangeHoldActive ? ACK_RANGE_HOLD : 0) | (rangeFault ? ACK_RANGE_FAULT : 0);
}

void loop() {
  //manualController();
  //loadCellDebug();
  // if (DCTS_EN) {
  //   DCTS();
  // }
  //printControllerDebug();
  //readDebug();
  //ultraSonic();
  mainControl();
}


// Software PWM ISRs, see PWMTimerInit()
ISR(TIMER3_COMPA_vect) {  // Traction
  if (OCR3B != 0) {
    digitalWrite(TractionWheel1_PWM, HIGH);
    digitalWrite(TractionWheel2_PWM, HIGH);
    digitalWrite(TractionWheel3_PWM, HIGH);
  }
}
ISR(TIMER3_COMPB_vect) {
  digitalWrite(TractionWheel1_PWM, LOW);
  digitalWrite(TractionWheel2_PWM, LOW);
  digitalWrite(TractionWheel3_PWM, LOW);
}
ISR(TIMER4_COMPA_vect) {  // Circumferential
  if (OCR4B != 0) {
    digitalWrite(Circumferential_PWM, HIGH);
  }
}

ISR(TIMER4_COMPB_vect) {
  digitalWrite(Circumferential_PWM, LOW);
}

ISR(TIMER5_COMPA_vect) {  // Winch
  if (OCR5B != 0) {
    digitalWrite(Winch_PWM, HIGH);
  }
}

ISR(TIMER5_COMPB_vect) {
  digitalWrite(Winch_PWM, LOW);
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(Stepper_PWM, !digitalRead(Stepper_PWM));  // One step every two toggles
}
