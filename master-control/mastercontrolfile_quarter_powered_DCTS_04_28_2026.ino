// Main Control File on Machine
// Version 1.1 4-28-2026
// Changes made: Replaced the wireless controller with controls wired directly
// to the Arduino Mega and added load-cell logging over Serial.
#include "HX711.h"

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

// Local control inputs. Buttons use INPUT_PULLUP and are active LOW.
#define Tension_On_Button 22
#define Tension_Off_Button 24
#define Tension_Set_Point_Button 26
#define Tension_Indicated_Button 28
#define Chainsaw_On_Button 30
#define Chainsaw_Off_Button 32
#define Chainsaw_Distance_Set_Point_Button 34
#define Chainsaw_Distance_Indicated_Button 36
#define Emergency_Button 38

// Local joystick and setpoint inputs
#define Left_Joystick_Left_Right_X A0
#define Left_Joystick_Up_Down_Y A1
#define Right_Joystick_Left_Right_X A8
#define Right_Joystick_Up_Down_Y A9
#define Tension_Potentiometer A4

//////////////////

uint8_t Target_Weight = 30;  // Adjustable Target Weight global var

// Ultrasonic sensor duration var
unsigned long ultra_dur;
unsigned long ultra_dis;

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

// Local input settings
const uint8_t LOCAL_BUTTON_PINS[] = {
  Tension_On_Button,
  Tension_Off_Button,
  Tension_Set_Point_Button,
  Tension_Indicated_Button,
  Chainsaw_On_Button,
  Chainsaw_Off_Button,
  Chainsaw_Distance_Set_Point_Button,
  Chainsaw_Distance_Indicated_Button
};
const uint8_t LOCAL_BUTTON_COUNT = sizeof(LOCAL_BUTTON_PINS) / sizeof(LOCAL_BUTTON_PINS[0]);
const unsigned long DEBOUNCE_MS = 25;
bool lastButtonReading[LOCAL_BUTTON_COUNT];
bool stableButtonState[LOCAL_BUTTON_COUNT];
unsigned long lastButtonChange[LOCAL_BUTTON_COUNT];
bool emergencyActive = false;

const int JOYSTICK_CENTER = 512;
const int JOYSTICK_DEADZONE = 100;
const uint8_t JOYSTICK_OUTPUT_DEADZONE = 15;

void loadCellInit() {            // Initialize load cell scale
  scale.begin(DT_PIN, SCK_PIN);  // Initialize HX711 with DT and SCK pins
  scale.set_gain(32);
  // Serial.println("Initializing HX711...");
  delay(500);      // Allow some time for stabilization
  scale.tare(20);  // Tare the scale
  // Serial.print("Tare offset: ");
  // Serial.println(scale.get_offset());  // Display the tare offset value
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

  for (uint8_t i = 0; i < LOCAL_BUTTON_COUNT; i++) {
    pinMode(LOCAL_BUTTON_PINS[i], INPUT_PULLUP);
    lastButtonReading[i] = HIGH;
    stableButtonState[i] = HIGH;
    lastButtonChange[i] = 0;
  }
  pinMode(Emergency_Button, INPUT_PULLUP);

  pinMode(Left_Joystick_Left_Right_X, INPUT);
  pinMode(Left_Joystick_Up_Down_Y, INPUT);
  pinMode(Right_Joystick_Left_Right_X, INPUT);
  pinMode(Right_Joystick_Up_Down_Y, INPUT);
  pinMode(Tension_Potentiometer, INPUT);
}

void stopAllMotion() {
  manualCircumControl(0, 0);
  manualTractControl(0, 0);
  manualTensionControl(0, 0);
  manualRadialControl(0, 0);
  digitalWrite(Chainsaw_EN, LOW);
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

  // --- Set to CTC Mode ---
  // WGM62:0 = 0b010 → CTC mode for Timer6 (8-bit timer on Mega)
  TCCR1B |= (1 << WGM12);  // On Mega, Timer6 WGM bits are slightly different

  // --- Set Prescaler ---
  // Prescaler = 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);

  OCR1A = 200;

  // --- Enable Timer Compare Interrupt ---
  TIMSK1 &= ~(1 << OCIE1A);  // Keep off till needed
  //TIMSK1 |= (1 << OCIE1A);


  // Enable global system interrupts
  sei();
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

void manDelay(int del) {
  int i = 0;
  while (i < del) {
    i++;
  }
}

//////////////////
// Winch Tension PID
// Feedback is the load cell converted to lbs, the setpoint is Target_Weight captured
// from the local potentiometer, and the signed PID effort becomes a winch duty (OCR5B) plus a
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

void loadCellDebug() {
  Serial.print(F("LOAD_CELL raw="));
  Serial.print(reading);
  Serial.print(F(" lbs="));
  Serial.print((float)reading / QUANTA_PER_LB, 2);
  Serial.print(F(" valid="));
  Serial.println(sensorValid ? F("Y") : F("N"));
}

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

bool buttonPressed(uint8_t index) {
  bool currentReading = digitalRead(LOCAL_BUTTON_PINS[index]);
  if (currentReading != lastButtonReading[index]) {
    lastButtonChange[index] = millis();
    lastButtonReading[index] = currentReading;
  }

  if ((millis() - lastButtonChange[index]) >= DEBOUNCE_MS &&
      currentReading != stableButtonState[index]) {
    stableButtonState[index] = currentReading;
    return currentReading == LOW;
  }
  return false;
}

uint8_t joystickSpeed(int value) {
  int distanceFromCenter = abs(value - JOYSTICK_CENTER);
  if (distanceFromCenter <= JOYSTICK_DEADZONE) {
    return 0;
  }

  long speed = map(distanceFromCenter, JOYSTICK_DEADZONE, JOYSTICK_CENTER,
                   0, 193);
  speed = constrain(speed, 0, 193);
  return speed < JOYSTICK_OUTPUT_DEADZONE ? 0 : (uint8_t)speed;
}

void handleLocalButtons() {
  if (buttonPressed(0)) {
    if (!DCTS_EN) {
      winchStop();
      resetPID();
    }
    DCTS_EN = true;
    Serial.println(F("DCTS enabled"));
  }
  if (buttonPressed(1)) {
    DCTS_EN = false;
    resetPID();
    winchStop();
    Serial.println(F("DCTS disabled"));
  }
  if (buttonPressed(2)) {
    Target_Weight = (uint8_t)map(analogRead(Tension_Potentiometer),
                                 0, 1023, 10, 175);
    TW_Adj = Target_Weight * (long)10430;
    resetPID();
    Serial.print(F("Target tension set to "));
    Serial.print(Target_Weight);
    Serial.println(F(" lbs"));
  }
  if (buttonPressed(3)) {
    loadCellDebug();
  }
  if (buttonPressed(4)) {
    digitalWrite(Chainsaw_EN, HIGH);
    Serial.println(F("Chainsaw enabled"));
  }
  if (buttonPressed(5)) {
    digitalWrite(Chainsaw_EN, LOW);
    Serial.println(F("Chainsaw disabled"));
  }
  if (buttonPressed(6)) {
    Serial.println(F("Distance set-point input is not used by this controller"));
  }
  if (buttonPressed(7)) {
    Serial.print(F("DISTANCE cm="));
    Serial.println(ultra_dis);
  }
}

void handleLocalJoysticks() {
  int winchValue = analogRead(Left_Joystick_Left_Right_X);
  int tractionValue = analogRead(Left_Joystick_Up_Down_Y);
  int circumValue = analogRead(Right_Joystick_Left_Right_X);
  int radialValue = analogRead(Right_Joystick_Up_Down_Y);

  if (!DCTS_EN) {
    manualTensionControl(joystickSpeed(winchValue),
                         winchValue < JOYSTICK_CENTER);
  }
  manualTractControl(joystickSpeed(tractionValue),
                     tractionValue < JOYSTICK_CENTER);
  manualCircumControl(joystickSpeed(circumValue),
                      circumValue < JOYSTICK_CENTER);
  manualRadialControl(joystickSpeed(radialValue) > 20,
                      radialValue < JOYSTICK_CENTER);
}

void ultraSonic() {
  digitalWrite(Ultrasonic_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(Ultrasonic_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultrasonic_TRIG, LOW);
  ultra_dur = pulseIn(Ultrasonic_ECHO, HIGH, 30000);
  ultra_dis = ultra_dur * 0.0343 / 2;
  // Serial.println(ultra_dur);
  // Serial.println(ultra_dis);
}

void setup() {  // Runs all initializing functions
  Serial.begin(115200);
  pinInit();
  loadCellInit();
  PWMTimerInit();
  resetPID();  // Start the PID's dt clock

  Serial.println(F("Local wired controller initialized."));
  Serial.println(F("Load-cell format: LOAD_CELL raw=<counts> lbs=<weight> valid=<Y/N>"));
}

void mainControl() {
  bool emergencyPressed = digitalRead(Emergency_Button) == LOW;
  if (emergencyPressed) {
    if (!emergencyActive) {
      emergencyActive = true;
      DCTS_EN = false;  // Require the operator to re-enable DCTS after recovery
      resetPID();
      Serial.println(F("EMERGENCY STOP active"));
    }
    stopAllMotion();
    return;
  }

  if (emergencyActive) {
    emergencyActive = false;
    resetPID();
    Serial.println(F("Emergency stop released; DCTS remains disabled"));
  }

  handleLocalButtons();
  handleLocalJoysticks();

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
    ultraSonic();
    loadCellDebug();
  }

  if (DCTS_EN) {
    DCTS();
  }
}

void loop() {
  mainControl();
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
