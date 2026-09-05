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

// Ultrasonic sensor duration var
unsigned long ultra_dur;
unsigned long ultra_dis;

// HX711 & load cell variables
HX711 scale;               // ADC Init
bool DCTS_EN = 0;          // Enable DCTS
long reading = 0;          // Raw reading from load cell
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

uint8_t ackArr[2];  // Send back tension and distance values

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
  //reading = scale.read();
  // Serial.print("\n\nRaw Reading: ");
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

  pinMode(Ultrasonic_TRIG, OUTPUT);
  pinMode(Ultrasonic_ECHO, OUTPUT);
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
  manualRadialControl(0, 0);
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
      // reading = scale.read();
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

void ultraSonic() {
  digitalWrite(Ultrasonic_TRIG, LOW);
  // manDelay(2);
  // delayMicroseconds(2);
  digitalWrite(Ultrasonic_TRIG, HIGH);
  // delayMicroseconds(10);
  // manDelay(10);
  digitalWrite(Ultrasonic_TRIG, LOW);
  ultra_dur = pulseIn(Ultrasonic_ECHO, HIGH, 30000);
  ultra_dis = ultra_dur * 0.0343 / 2;
  // Serial.println(ultra_dur);
  // Serial.println(ultra_dis);
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
    reading = scale.read();
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
    ultraSonic();
    ackArr[1] = ultra_dis;
    //// Serial.println(ackArr[0]);
  }
  // Get reading
  //// Serial.println(radioSent);
  if (radioSent) {  // Only do this stuff if packets are being sent radioSent

    if (rxPkt.b_EMERGENCY) {  // If emergency button is pressed at all
      emergency();
    }
    if (rxPkt.tensionSet != Target_Weight && rxPkt.tensionSet < 176) {  // If a new tension is detected, set it and make sure its between 10 and 175
      Target_Weight = constrain((uint8_t)rxPkt.tensionSet, 10, 175);
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
    manualRadialControl(((rxPkt.radial_spd > 20)), rxPkt.radial_dir);

    // if (rxPkt.winch_spd < rxPkt.circum_spd) {  // If DCTS isn't enabled, allow manual control
    //   manualCircumControl(rxPkt.circum_spd, rxPkt.circum_dir);
    // }
    // if (rxPkt.traction_spd > rxPkt.radial_spd) {  // If DCTS isn't enabled, allow manual control
    //   manualTractControl(rxPkt.traction_spd, rxPkt.traction_dir);
    // } else {
    //   manualRadialControl(((rxPkt.radial_spd > 20)), rxPkt.radial_dir);
    // }
  } else {
    manualCircumControl(0, 0);
    manualTractControl(0, 0);
    manualTensionControl(0, 0);
    manualRadialControl(0, 0);
    resetPID();  // Link is down, start clean when it comes back
  }
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
