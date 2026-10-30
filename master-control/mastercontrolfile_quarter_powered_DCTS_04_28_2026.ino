// Main Control File on Machine
// Version 1.1 4-28-2026
// Changes made: Changed DCTS equations and limits to operate with 1/4th the power. Now allows sending of range data to controller. Allow for greater auto tension values from controller.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "HX711.h"
#include "printf.h"

//////////////////
// Pin definitions

// HX711 Pins
#define DT_PIN 17   // HX711 DT pin connected to Arduino D2
#define SCK_PIN 18  // HX711 SCK pin connected to Arduino D3

// Traction Wheel Pins
#define TractionWheel1_PWM 6
#define TractionWheel2_PWM 5
#define TractionWheel3_PWM 4
#define TractionWheel1_DIR 23
#define TractionWheel2_DIR 25
#define TractionWheel3_DIR 27

// Circumferential Motor Pins
#define Winch_PWM 3
#define Winch_DIR 29

// Winch Motor Pins
#define Circumferential_PWM 45
#define Circumferential_DIR 31

// Stepper Motor Pins
#define Stepper_PWM 37  // These may be flipped
#define Stepper_DIR 35  // They are named C1 and C2??

// Chainsaw Pin
#define Chainsaw_EN 39  // Toggle on/off high/low

// Load Cell Definitions
#define PLMI 5  // The Plus/minus value. The padding for the target weight.

// Ultrasonic Sensor Pin Def
#define Ultrasonic_ECHO 20
#define Ultrasonic_TRIG 19

// RF pins
#define CE_PIN 49
#define CSN_PIN 53

//////////////////

unsigned long lastReceivedTime = 0;
const unsigned long connectionTimeout = 200;  // ms

uint8_t Target_Weight = 30;  // Adjustable Target Weight global var

// Ultrasonic sensor
const unsigned long ULTRA_PERIOD_MS = 60;      // HC-SR04 needs ~60 ms between pings for old echoes to die out
const unsigned long ULTRA_TIMEOUT_US = 15000;  // ~257 cm, also caps how long pulseIn can block the loop
const uint8_t ULTRA_MIN_CM = 2;                // Below the HC-SR04's working range
const uint8_t ULTRA_MAX_MISSES = 3;            // Consecutive missed echoes before the range is invalid
unsigned long ultra_dur;                       // Echo pulse width, us
unsigned long lastPingTime = 0;
unsigned long lastRangeTime = 0;  // millis() of the last valid range
uint16_t rangeHistory[3];         // Last valid readings, for the median filter
uint8_t rangeCount = 0;
uint8_t rangeIdx = 0;
uint8_t rangeMisses = 0;
uint16_t rangeCm = 0;         // Filtered distance to the trunk
bool rangeValid = false;      // False until a reading arrives, and after ULTRA_MAX_MISSES misses
bool rangeNewSample = false;  // Set on each valid reading, consumed by rangeHold()

// HX711 & load cell variables
HX711 scale;               // ADC Init
bool DCTS_EN = 0;          // Enable DCTS
long reading = 0;          // Tare-corrected reading from load cell
short int counter = 0;     // Counter to slow the rate of message speed
uint8_t duty_percent = 0;  // Duty cycle percentage
uint8_t temp_OCR3B = 0;    // OCR3B temporary variable
uint8_t temp_OCR4B = 0;    // OCR4B temporary variable
uint8_t temp_OCR5B = 0;    // OCR5B temporary variable

// Convert weight in lbs to quanta
long int TW_Adj = Target_Weight * (long)10430;
long int PLMI_Adj = PLMI * (long)10430;

// RF Variables

/* Buttons 1-8
  bool b_DCTS_ON;
  bool b_DCTS_OFF;
  bool b_SET_TENSION;
  bool b_DISP_TENSION;
  bool b_CHAIN_ON;
  bool b_CHAIN_OFF;
  bool b_CHAIN_DIST_SET;
  bool b_CHAIN_DIST_DISP;
*/

// Define the same structure used in the transmitter
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

bool isValid(const ControllerPacket& p) {  // Verifies packet sends valid struct
  // WIP
  return true;
}

// Radio Initialization
RF24 radio(CE_PIN, CSN_PIN);
const byte address[5] = { 'T', 'E', 'S', 'T', '1' };  // Must match transmitter
ControllerPacket rxPkt;
ControllerPacket tempPkt;
bool radioSent = 0;

// Send back tension (lbs), distance (cm, 255 = no reading), and status bits
uint8_t ackArr[3] = { 0, 255, 0 };
#define ACK_RANGE_HOLD 0x01   // Range hold is running
#define ACK_RANGE_FAULT 0x02  // Range hold stopped itself, see rangeHoldFault()

// Dir Speed Dampening
bool currDIR = 0;
uint16_t dirCTR = 0;

//RF Debug
unsigned long startTimer;
char testchar = '0';  // Debug // Serial.Read val (for controlling with just main control)

void loadCellInit() {            // Initialize load cell scale
  scale.begin(DT_PIN, SCK_PIN);  // Initialize HX711 with DT and SCK pins
  scale.set_gain(32);
  // Serial.println("Initializing HX711...");
  delay(500);      // Allow some time for stabilization
  scale.tare(20);  // Tare the scale
  // Serial.print("Tare offset: ");
  // Serial.println(scale.get_offset());  // Display the tare offset value
}

void loadCellDebug() {  // Used to read the load cell for debug purposes
  //reading = (long)scale.get_value(1);
  // Serial.print("\n\nTare-corrected Reading: ");
  // Serial.println(reading);  // Display the digital reading as a decimal
  // Serial.print("\n\nTarget Weight: ");
  // Serial.println(TW_Adj);
  // Serial.print("\nWeight (lbs): ");
  // Serial.println(reading / 10430.0);  // Linear regression slope, every pound is about 10430 quanta, +-30
  // Serial.print("\nWeight (kg): ");
  // Serial.println(reading / 22998.0);
  //manDelay(2000000);
  //delay(100);
}

void pinInit() {  // Initialize every pin we will use according to Arduino Mega PCB schematic
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
  digitalWrite(Chainsaw_EN, LOW);  // Fail safe: cutting tool starts disabled

  pinMode(Ultrasonic_TRIG, OUTPUT);
  pinMode(Ultrasonic_ECHO, INPUT);
  digitalWrite(Ultrasonic_TRIG, LOW);
}

void radioInit() {  // Intializes RF, this is used for both RX and TX
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

void emergency() {  // Turn off all motors when emergency button is pressed and stay locked in it
  // Serial.println("EMERGENCY PRESSED");
  DCTS_EN = 0;  // Require the operator to re-enable automatic tension after recovery
  manualCircumControl(0, 0);
  manualTractControl(0, 0);
  manualTensionControl(0, 0);
  rangeHoldOff();
  digitalWrite(Chainsaw_EN, LOW);
  while (rxPkt.b_EMERGENCY) {  // Stay locked until the controller explicitly clears its emergency state
    radioRX();
  }
  resetPID();  // The load cell went unread while locked, don't resume on stale readings
}

void PWMTimerInit() {  // initializes the PWM for motor speed
  // Timer mode
  TCCR3A = 0;  // Clear all bits in control register A

  // Clear all WGM and CS bits first
  TCCR3B &= ~((1 << WGM33) | (1 << WGM32) | (1 << WGM31) | (1 << WGM30));
  TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));

  // Set CTC mode (WGM32 = 1), Prescaler = 64 cs31 + cs30
  TCCR3B |= (1 << WGM32) | (1 << CS32) | (1 << CS30);

  OCR3A = 194;
  OCR3B = 0;

  TIMSK3 |= (1 << OCIE3A) | (1 << OCIE3B);  // Enable interrupts

  TCCR4A = 0;  // Clear all bits in control register A CIRCUM MOTOR

  // Clear WGM and CS bits
  TCCR4B &= ~((1 << WGM43) | (1 << WGM42) | (1 << WGM41) | (1 << WGM40));
  TCCR4B &= ~((1 << CS42) | (1 << CS41) | (1 << CS40));

  // Set CTC mode (WGM42 = 1), Prescaler =  64 cs41 + cs40
  TCCR4B |= (1 << WGM42) | (1 << CS42) | (1 << CS40);

  OCR4A = 194;
  OCR4B = 0;

  TIMSK4 |= (1 << OCIE4A) | (1 << OCIE4B);  // Enable interrupts

  TCCR5A = 0;  // Clear all bits in control register A WINCH TIMER

  // Clear WGM and CS bits
  TCCR5B &= ~((1 << WGM53) | (1 << WGM52) | (1 << WGM51) | (1 << WGM50));
  TCCR5B &= ~((1 << CS52) | (1 << CS51) | (1 << CS50));

  // Set CTC mode (WGM42 = 1), Prescaler = 1024 (CS42 + CS40)
  TCCR5B |= (1 << WGM52) | (1 << CS51) | (1 << CS50);

  OCR5A = 194;
  OCR5B = 0;

  TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);  // Enable interrupts

  // --- Clear Timer/Counter Control Registers ---
  TCCR1A = 0;
  TCCR1B = 0;

  // --- Set to CTC Mode (WGM12), Timer1 is the 16-bit stepper timer ---
  TCCR1B |= (1 << WGM12);

  // --- Set Prescaler ---
  // Prescaler = 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);

  // 16 MHz / 1024 / 201 = ~77.7 toggles/s, so ~39 steps/s. Lower is faster.
  OCR1A = 200;

  // --- Enable Timer Compare Interrupt ---
  TIMSK1 &= ~(1 << OCIE1A);  // Keep off till needed
  //TIMSK1 |= (1 << OCIE1A);


  // Enable global system interrupts
  sei();
}

void radioRX() {  // Simply print what was received.
  if (radio.available()) {
    radioSent = 1;
    lastReceivedTime = millis();
    radio.read(&tempPkt, sizeof(tempPkt));
    if (isValid(tempPkt)) {  // optional extra safety
      rxPkt = tempPkt;       // now update the live struct
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
      if (spd < rampStep) break;  // Prevent underflow when spd becomes < 0
    }
  }
}

void debugRX() {

  // Now print the rest of the packet values
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

void manDelay(int del) {
  int i = 0;
  while (i < del) {
    i++;
  }
}

//////////////////
// Winch Tension PID
// Feedback is the load cell converted to lbs, the setpoint is Target_Weight from the
// controller pot, and the signed PID effort becomes a winch duty (OCR5B) plus a
// direction on Winch_DIR. Steps once per fresh load cell sample, using the real
// elapsed time as dt.

const float QUANTA_PER_LB = 10430.0f;         // Quanta per pound, same slope as the ack conversion
const uint8_t WINCH_DUTY_MIN = 12;            // Stall floor, below this the winch won't turn
const uint8_t WINCH_DUTY_MAX = 30;            // Quarter power ceiling
const float TENSION_DEADBAND_LBS = 2.0f;      // Error padding, within this we brake
const float TENSION_HARD_LIMIT_LBS = 185.0f;  // Never pull past this
const long RAW_MIN_VALID = -50000L;           // Reject readings below this. PLACEHOLDER, verify on the rig
const long RAW_MAX_VALID = 3000000L;          // Reject readings above this. PLACEHOLDER, verify on the rig

// PID gains. Tune on the bench, ideally in WinchTensionPID_test first.
float Kp = 1.5f;  // Duty per lb
float Ki = 0.0f;  // Duty per lb-second
float Kd = 0.0f;  // Duty per lb/second

const float DERIV_ALPHA = 0.15f;          // Derivative low pass, the load cell is noisy
const float INTEGRAL_CLAMP_DUTY = 30.0f;  // Anti windup limit on the Ki contribution

// The duty band is only 18 counts wide, so all of it has to cover the error range we
// actually operate in. Effort magnitudes from 0 to EFFORT_FULL_SCALE map linearly onto
// 12-30, so at Kp = 1.5 the winch saturates near 20 lbs of error.
const float EFFORT_FULL_SCALE = 30.0f;
const float EFFORT_DEADZONE = 0.5f;  // Below this we brake instead of creeping at the floor

// At the hard limit the winch creeps toward slack, since a brake would leave the cable
// parked above the limit with nothing able to bring it down. Time boxed, because a
// reversed Winch_DIR would make the creep pull harder rather than let off.
const unsigned long OVERTENSION_RELIEF_MS = 1500;

// PID state
float measured_lbs = 0.0f;           // Load cell reading converted to lbs
bool sensorValid = false;            // Cleared when a reading fails the range check
float integral = 0.0f;               // Accumulated integral term
float prevError = 0.0f;              // Previous error, for the derivative
float derivFiltered = 0.0f;          // Filtered derivative
bool newSample = false;              // Set by mainControl on a fresh load cell read
unsigned long lastPIDTime = 0;       // millis() of the last PID step
unsigned long overTensionStart = 0;  // millis() the hard limit was first hit, 0 when clear
bool overTensionLatched = false;     // Set once relief times out, cleared by resetPID

void winchStop() {  // Duty 0, the Timer5 ISRs then hold Winch_PWM low
  OCR5B = 0;
}

void winchRelieve() {  // Creep toward slack at the stall floor
  digitalWrite(Winch_DIR, HIGH);
  OCR5B = WINCH_DUTY_MIN;
}

void resetPID() {  // Wipe the loop's memory whenever the winch changes hands
  integral = 0.0f;
  prevError = 0.0f;
  derivFiltered = 0.0f;
  newSample = false;
  lastPIDTime = millis();
  overTensionStart = 0;
  overTensionLatched = false;
}

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
// Holds the stepper at rangeSet cm from the trunk using the ultrasonic reading.
// The stepper only has one speed, so this is on/off with a deadband: start moving
// once the error passes RANGE_DEADBAND_CM, stop once the error reaches zero.
// There are no limit switches, so a move that stops closing the error faults out.

const uint8_t RANGE_SET_MIN_CM = 5;      // Must match the remote's Set Range pot mapping
const uint8_t RANGE_SET_MAX_CM = 99;
const uint8_t RANGE_DEADBAND_CM = 2;     // Error allowed before the stage moves
const unsigned long RANGE_STALE_MS = 300;  // No valid reading for this long stops the hold

// manualRadialControl() dir value that moves the stage toward the trunk. VERIFY on the
// rig with the joystick. If it is wrong the hold moves away and faults on RANGE_WRONG_WAY_CM.
const bool RADIAL_DIR_TOWARD_TREE = true;

// A move must close the error by RANGE_PROGRESS_CM every RANGE_PROGRESS_MS, otherwise the
// stage is stalled or at the end of its travel. PLACEHOLDER: depends on the stage's mm/step.
const unsigned long RANGE_PROGRESS_MS = 5000;
const uint8_t RANGE_PROGRESS_CM = 1;
const uint8_t RANGE_WRONG_WAY_CM = 3;  // Error growth during a move that means the wrong direction

uint8_t Target_Range = 0;       // Active setpoint, cm
bool rangeHoldActive = false;
bool rangeFault = false;        // Latched until the remote sends a new rangeSet
bool rangeSetPrimed = false;    // First packet after boot only records rangeSet
uint8_t lastRangeSet = 0;       // Last rangeSet received, holds engage on a change
int8_t radialMotion = 0;        // 1 toward the trunk, -1 away, 0 stopped
uint16_t moveStartAbsErr = 0;   // |error| when the current move began
uint16_t bestAbsErr = 0;        // Smallest |error| seen during the current move
unsigned long lastProgressTime = 0;

void radialStop() {
  radialMotion = 0;
  manualRadialControl(0, 0);
}

void rangeHoldOff() {  // Stop the stage and drop the hold, the remote must send a new setpoint
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
    if (reached) {  // Stop, a reversal waits for the next sample
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

void rangeSetUpdate() {  // Engage, retarget, or cancel the hold when the remote's rangeSet changes
  if (!rangeSetPrimed) {  // A master reboot must not start the stage on a stale remote setpoint
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
    rangeNewSample = false;  // Act on a reading taken after the setpoint arrived
  } else {
    rangeHoldActive = false;  // 0 or out of range cancels
  }
}

void radialControl() {
  if (rxPkt.radial_spd > 20) {  // The joystick always wins and drops the hold
    rangeHoldActive = false;
    radialMotion = 0;
    manualRadialControl(1, rxPkt.radial_dir);
  } else if (rangeHoldActive) {
    rangeHold();
  } else {
    radialStop();
  }
}

void manualController() {
  switch (testchar) {
    case 'E':  // Emergency
      // Handle 'E'
      // Serial.println("E Pressed.");
      emergency();
      break;

    case 'O':  // Stop traction wheels
      // Handle 'E'
      // Serial.println("O Pressed.");
      manualTractControl(50, 1);
      break;

    case 'P':  // Up on traction wheels
      // Handle 'E'
      // Serial.println("P Pressed.");
      manualTractControl(50, 0);
      break;

    case 'I':
      // Handle 'E'
      // Serial.println("I Pressed.");
      manualTractControl(0, 0);
      break;

    case 'F':
      // Handle 'F'
      // Serial.println("F Pressed.");
      //manualTractControl(50, 1);
      //TIMSK5 |= (1 << OCIE5A);
      manualTensionControl(100, 1);
      //manualCircumControl(100, 1);
      //manualRadialControl(1, 1);
      break;

    case 'R':
      // Handle 'R'
      // Serial.println("R Pressed.");
      //manualTractControl(50, 0);
      //TIMSK5 |= (1 << OCIE5A);
      manualTensionControl(100, 0);
      //manualCircumControl(100, 0);
      //manualRadialControl(1, 0);
      break;

    case 'Q':
      // Handle 'R'
      // Serial.println("Q Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      //manualCircumControl(100, 0);
      manualRadialControl(1, 0);
      break;

    case 'W':
      // Handle 'R'
      // Serial.println("W Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      //manualCircumControl(100, 0);
      manualRadialControl(1, 1);
      break;

    case 'T':
      // Handle 'R'
      // Serial.println("T Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      manualCircumControl(100, 0);
      //manualRadialControl(1, 0);
      break;

    case 'Y':
      // Handle 'R'
      // Serial.println("Y Pressed.");
      // manualTractControl(50, 0);
      //manualTensionControl(100, 0);
      manualCircumControl(100, 1);
      //manualRadialControl(1, 0);
      break;

    case 'S':
      // Handle 'R'
      // Serial.println("S Pressed.");
      //TIMSK5 &= ~(1 << OCIE5A);
      manualTractControl(0, 0);
      manualTensionControl(0, 0);
      manualTractControl(0, 0);
      manualCircumControl(0, 0);
      manualRadialControl(0, 0);
      break;

    case 'C':
      // Handle 'E'
      // Serial.println("E Pressed.");
      digitalWrite(Chainsaw_EN, HIGH);
      //DCTS_EN = 1;
      break;

    case 'N':
      // Handle 'E'
      // Serial.println("E Pressed.");
      //DCTS_EN = 0;
      digitalWrite(Chainsaw_EN, LOW);
      break;

    case 'Z':
      // Handle 'E'
      // Serial.println("E Pressed.");
      //DCTS_EN = 0;
      //TIMSK4 &= ~(1 << OCIE4A);
      //TIMSK5 &= ~(1 << OCIE5A);
      break;

    case 'X':
      // Handle 'E'
      // Serial.println("E Pressed.");
      //DCTS_EN = 0;
      //TIMSK4 |= (1 << OCIE4A);
      //TIMSK5 |= (1 << OCIE5A);
      break;

    default:
      // Do nothing for any other character
      break;
  }


  /*if ( Serial.available() > 0) {  // Read serial values from the serial monitor on debug laptop
    testchar = toUpperCase(// Serial.read());
  }*/
}


/*void readDebug() {  // Read multiple digit values from serial monitor to send mock values for load/distance vals
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');  // read up to newline
    s.trim();                                 // strip CR/LF
    int val = s.toInt();                      // convert "203" → 203
    ackArr[0] = val;                          // ‘5’ → 5
    radio.writeAckPayload(1, ackArr, sizeof(ackArr));
    // Serial.print("ACK set to: ");
    // Serial.println(ackArr[0]);
  }
}*/

void buttonSel() {  // Switch case for buttons pressed on the remote controller
  switch (rxPkt.buttonID) {
    case '1':
      // Serial.println("1 Pressed.");
      if (!DCTS_EN) {  // Start the loop clean on every enable
        resetPID();
      }
      DCTS_EN = 1;
      break;

    case '2':
      // Handle 'F'
      // Serial.println("2 Pressed.");
      DCTS_EN = 0;
      resetPID();
      break;

    case '3':
      // Handle 'R'
      // Serial.println("3 Pressed.");

      break;

    case '4':
      // Handle 'R'
      // Serial.println("4 Pressed.");
      // reading = (long)scale.get_value(1);
      // ackArr[0] = reading / 10430.0;
      break;

    case '5':
      // Handle 'E'
      // Serial.println("5 Pressed.");
      digitalWrite(Chainsaw_EN, HIGH);
      break;

    case '6':
      // Handle 'E'
      // Serial.println("6 Pressed.");
      digitalWrite(Chainsaw_EN, LOW);
      break;

    case '7':
      // Handle 'E'
      // Serial.println("7 Pressed.");

      break;

    case '8':
      // Handle 'E'
      // Serial.println("8 Pressed.");

      break;

    default:
      // Do nothing for any other character
      break;
  }
}

void printControllerDebug() {  // Prints every joystick value
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

  // Serial.println();  // Blank line for readability
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

void ultraSonic() {  // One ping, filtered into rangeCm / rangeValid
  digitalWrite(Ultrasonic_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(Ultrasonic_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultrasonic_TRIG, LOW);
  ultra_dur = pulseIn(Ultrasonic_ECHO, HIGH, ULTRA_TIMEOUT_US);
  uint16_t cm = (uint16_t)(ultra_dur * 0.0343f / 2.0f);
  // Serial.println(ultra_dur);
  // Serial.println(cm);

  if (ultra_dur == 0 || cm < ULTRA_MIN_CM) {  // A timeout returns 0, which is not "touching the trunk"
    if (rangeMisses < 255) rangeMisses++;
    if (rangeMisses >= ULTRA_MAX_MISSES) {
      rangeValid = false;
      rangeCount = 0;  // Don't let old readings vote once the echo comes back
    }
    return;
  }
  rangeMisses = 0;

  rangeHistory[rangeIdx] = cm;
  rangeIdx = (rangeIdx + 1) % 3;
  if (rangeCount < 3) rangeCount++;
  rangeCm = (rangeCount < 3) ? cm : median3(rangeHistory[0], rangeHistory[1], rangeHistory[2]);
  rangeValid = true;
  rangeNewSample = true;
  lastRangeTime = millis();
}

void setup() {  // Runs all initializing functions
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

void mainControl() {  // Manages all movement and calls radio RX function
  radioRX();
  if (millis() - lastReceivedTime > connectionTimeout) {
    radioSent = 0;  // Lost connection
  }

  if (scale.is_ready()) {
    reading = (long)scale.get_value(1);  // Apply the offset captured by tare()
    if (reading >= RAW_MIN_VALID && reading <= RAW_MAX_VALID) {  // Ignore obviously bad readings
      sensorValid = true;
      measured_lbs = (float)reading / QUANTA_PER_LB;
    } else {
      sensorValid = false;  // Keep the last measured_lbs, the PID brakes on this
    }
    if (DCTS_EN) {  // Only latch for the PID, otherwise the flag goes stale in manual mode
      newSample = true;
    }
    ackArr[0] = reading / 10430.0;
    //// Serial.println(ackArr[0]);
  }

  // Pinged on its own clock so pulseIn doesn't delay the tension PID's samples
  if (millis() - lastPingTime >= ULTRA_PERIOD_MS) {
    lastPingTime = millis();
    ultraSonic();
    ackArr[1] = rangeValid ? (uint8_t)min(rangeCm, (uint16_t)254) : 255;
  }
  // Get reading
  //// Serial.println(radioSent);
  if (radioSent) {  // Only do this stuff if packets are being sent radioSent

    if (rxPkt.b_EMERGENCY) {  // If emergency button is pressed at all
      emergency();
    }
    // 0 means the remote has not confirmed a setpoint. Compare the value as sent,
    // never a clamped copy: clamping 1-9 up to 10 made every later packet look new,
    // so resetPID() ran forever and the winch held its last duty with no over-tension check.
    if (rxPkt.tensionSet >= 10 && rxPkt.tensionSet <= 175 && rxPkt.tensionSet != Target_Weight) {
      Target_Weight = rxPkt.tensionSet;
      TW_Adj = Target_Weight * (long)10430;
      resetPID();  // New setpoint, don't carry a stale integrator into it
      // Serial.print("New tension set: ");
      // Serial.println(Target_Weight);
    }
    if (DCTS_EN) {  // If the DCTS is enabled
      DCTS();
    } else {  // If DCTS isn't enabled, allow manual control
      manualTensionControl(rxPkt.winch_spd, rxPkt.winch_dir);
    }

    buttonSel();  // Button logic

    // Manual control of motors
    manualCircumControl(rxPkt.circum_spd, rxPkt.circum_dir);
    manualTractControl(rxPkt.traction_spd, rxPkt.traction_dir);
    rangeSetUpdate();
    radialControl();

    // if (rxPkt.winch_spd < rxPkt.circum_spd) {  // If DCTS isn't enabled, allow manual control
    //   manualCircumControl(rxPkt.circum_spd, rxPkt.circum_dir);
    // }
    // if (rxPkt.traction_spd > rxPkt.radial_spd) {  // If DCTS isn't enabled, allow manual control
    //   manualTractControl(rxPkt.traction_spd, rxPkt.traction_dir);
    // } else {
    //   manualRadialControl(((rxPkt.radial_spd > 20)), rxPkt.radial_dir);
    // }
  } else {
    DCTS_EN = 0;  // Link is down; require Button 1 again before automatic tension resumes
    manualCircumControl(0, 0);
    manualTractControl(0, 0);
    manualTensionControl(0, 0);
    rangeHoldOff();  // Not resumed by the link returning, the remote must send a new setpoint
    digitalWrite(Chainsaw_EN, LOW);
    resetPID();  // Link is down, start clean when it comes back
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
  mainControl();  // This is the only function that needs to be in here
}


// Timer Handling the PWM signal for the motor.
ISR(TIMER3_COMPA_vect) {
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
ISR(TIMER4_COMPA_vect) {  // Winch
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
  digitalWrite(Stepper_PWM, !digitalRead(Stepper_PWM));  // Toggle Stepper pin
}
