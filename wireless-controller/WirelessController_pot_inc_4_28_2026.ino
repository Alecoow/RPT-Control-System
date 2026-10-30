// Wireless controller file.
// Reads the buttons, joysticks, and knob, sends them to the machine, and shows the
// machine's load and range on the LCD.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "printf.h"

// Buttons
#define Tension_On_Button 23                   // Button 1, automatic tension on
#define Tension_Off_Button 25                  // Button 2, automatic tension off
#define Tension_Set_Point_Button 27            // Button 3, open Set Load
#define Tension_Indicated_Button 29            // Button 4, confirm Set Load
#define Chainsaw_On_Button 24                  // Button 5, chainsaw on
#define Chainsaw_Off_Button 26                 // Button 6, chainsaw off
#define Chainsaw_Distance_Set_Point_Button 28  // Button 7, open Set Range
#define Chainsaw_Distance_Indicated_Button 30  // Button 8, confirm Set Range, also e-stop recovery
#define Emergency_Button 22                    // Button 9, emergency stop

// Define Joysticks
#define Left_Joystick_Left_Right_X A0   // Left Joystick Horizontal (X) Direction For Tension Loose/Tighten (Pin A0)
#define Left_Joystick_Up_Down_Y A1      // Left Joystick Vertical (Y) Direction For Elevator Climbing/Descending (Pin A1)
#define Right_Joystick_Left_Right_X A8  // Right Joystick Horizontal (X) Direction For Circumferential Position (Pin A8)
#define Right_Joystick_Up_Down_Y A9    // Right Joystick Vertical (Y) Direction For Chainsaw In/Out (Pin A10)
#define Potentiometer A4                // Knob for Set Load and Set Range

// NRF24
#define CE_PIN 49
#define CSN_PIN 53

// LCD, for reference only, the I2C library uses these pins itself
#define SDA_PIN 20
#define SCL_PIN 21

const unsigned long debounceDelay = 25;  // ms a button must stay steady to count


// Debounce state for buttons 1-8
int lastButtonState[8] = { LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW };                 // Last raw reading
unsigned long lastDebounceTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };                      // When the reading last changed
bool buttonPressed[8] = { false, false, false, false, false, false, false, false };  // Debounced state, LOW = pressed

bool emergencyButtonState = false;  // In emergency stop
bool setTens = false;               // Set Load screen open
bool setDist = false;               // Set Range screen open
// Joysticks Settings
int leftJoystickxVal;
int leftJoystickyVal;
int rightJoystickxVal;
int rightJoystickyVal;
int deadzone = 160;         // Counts from center before a stick does anything
int joystickDeadzone = 25;  // Smallest speed sent, anything lower is sent as 0
// Each stick's resting position, measured at power-up
int centerLeftX = 512;
int centerLeftY = 512;
int centerRightX = 512;
int centerRightY = 512;

uint8_t mapPotVal = 0;  // Knob value on the open setpoint screen, lbs or cm

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16x2
uint16_t potCTR = 0;                 // Unused

// The NRF24L01+PA+LNA runs on 3.3 V. Connecting it to 5 V will destroy it.
RF24 radio(CE_PIN, CSN_PIN);
const byte address[5] = { 'T', 'E', 'S', 'T', '1' };  // Must match the machine
const char message[] = "Transmission Testing. . . ";  // Unused
char receivedData[32];                                 // Unused

// Must match the struct in the master control file exactly
struct ControllerPacket {
  char buttonID;  // '1'-'8' on a button press, '0' otherwise
  bool b_EMERGENCY;
  uint8_t winch_spd;  // From A1
  bool winch_dir;
  uint8_t traction_spd;  // From A0
  bool traction_dir;
  uint8_t circum_spd;  // From A9
  bool circum_dir;
  uint8_t radial_spd;  // From A8
  bool radial_dir;
  uint8_t tensionSet;  // Tension setpoint in lbs, 0 = none yet
  uint8_t rangeSet;  // Range hold setpoint in cm, 0 = hold off
};

ControllerPacket packet = { '9', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Must match the master's RANGE_SET_MIN_CM / RANGE_SET_MAX_CM
const uint8_t RANGE_SET_MIN_CM = 5;
const uint8_t RANGE_SET_MAX_CM = 99;

// Reply from the machine
uint8_t ackArr[3] = { 0, 255, 0 };  // Load lbs, range cm (255 = no reading), status bits
#define ACK_RANGE_HOLD 0x01
#define ACK_RANGE_FAULT 0x02

// Brief "Confirmed" message after a setpoint is accepted
unsigned long confirmUntil = 0;
uint8_t confirmValue = 0;
const char* confirmUnit = "";

void ZeroMotionCommands() {
  packet.winch_spd = 0;
  packet.traction_spd = 0;
  packet.circum_spd = 0;
  packet.radial_spd = 0;
}

void setup() {
  Serial.begin(115200);

  // Buttons use the internal pull-ups, so they read LOW when pressed
  pinMode(Tension_On_Button, INPUT_PULLUP);                   // Button 1
  pinMode(Tension_Off_Button, INPUT_PULLUP);                  // Button 2
  pinMode(Tension_Set_Point_Button, INPUT_PULLUP);            // Button 3
  pinMode(Tension_Indicated_Button, INPUT_PULLUP);            // Button 4
  pinMode(Chainsaw_On_Button, INPUT_PULLUP);                  // Button 5
  pinMode(Chainsaw_Off_Button, INPUT_PULLUP);                 // Button 6
  pinMode(Chainsaw_Distance_Set_Point_Button, INPUT_PULLUP);  // Button 7
  pinMode(Chainsaw_Distance_Indicated_Button, INPUT_PULLUP);  // Button 8
  pinMode(Emergency_Button, INPUT_PULLUP);                    // Button 9
  pinMode(Left_Joystick_Left_Right_X, INPUT);                 // Left Joystick Horizontal (X) Direction
  pinMode(Left_Joystick_Up_Down_Y, INPUT);                    // Left Joystick Vertical (Y) Direction
  pinMode(Right_Joystick_Left_Right_X, INPUT);                // Right Joystick Horizontal (X) Direction
  pinMode(Right_Joystick_Up_Down_Y, INPUT);                   // Right Joystick Vertical (Y) Direction
  pinMode(Potentiometer, INPUT);                              // LCD Screen Joystick

  // Don't touch the sticks during power-up. A stick more than 150 counts off center is ignored and 512 used.
  centerLeftX = CalibrateAxis(Left_Joystick_Left_Right_X);
  centerLeftY = CalibrateAxis(Left_Joystick_Up_Down_Y);
  centerRightX = CalibrateAxis(Right_Joystick_Left_Right_X);
  centerRightY = CalibrateAxis(Right_Joystick_Up_Down_Y);

  // Radio, settings must match the machine
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setAutoAck(true);
  radio.setChannel(76);
  radio.openWritingPipe(address);
  printf_begin();
  radio.printPrettyDetails();

  lcd.init();
  lcd.backlight();
  LCDScreen();
}

// Debounces button i (0-7). On a new press it handles the setpoint screens and
// sends a packet with that button's ID.
void CheckButtonDebouncePressed(int pin, int i, const char* buttonName) {
  int reading = digitalRead(pin);

  if (reading != lastButtonState[i]) {  // Still bouncing, restart the timer
    lastDebounceTime[i] = millis();
  }

  if ((millis() - lastDebounceTime[i]) > debounceDelay) {  // Steady long enough
    if (reading != buttonPressed[i]) {
      buttonPressed[i] = reading;

      if (buttonPressed[i] == LOW) {  // Pressed
        Serial.print("Button ");
        Serial.print(i + 1);
        Serial.print(" Pressed: ");
        Serial.println(buttonName);

        // Only one setpoint screen can be open. Its confirm button sends the value and closes it.
        if (i == 2) {
          setTens = !setTens;
          setDist = false;
        } else if (i == 3 && setTens) {
          packet.tensionSet = PotToTension();
          setTens = false;
          ShowConfirm(packet.tensionSet, " lb");
        } else if (i == 6) {
          setDist = !setDist;
          setTens = false;
          if (setDist) packet.rangeSet = 0;  // Stop the hold while a new range is chosen
        } else if (i == 7 && setDist) {
          packet.rangeSet = PotToRange();
          setDist = false;
          ShowConfirm(packet.rangeSet, " cm");
        }

        packet.buttonID = '1' + i;
        SendPacket();
      }
    }
  }

  lastButtonState[i] = reading;
}

// Emergency button enters the emergency stop. Emergency + Button 8 together leave it.
void CheckEmergencyButton() {
  int emergencyReading = digitalRead(Emergency_Button);
  int button8Reading = digitalRead(Chainsaw_Distance_Indicated_Button);

  if (!emergencyButtonState) {
    if (emergencyReading == LOW) {
      emergencyButtonState = true;
      Serial.println("EMERGENCY BUTTON PRESSED - SYSTEM SHUTDOWN!");

      ZeroMotionCommands();
      packet.rangeSet = 0;  // The range hold needs a new confirm after recovery
      setTens = false;      // Close the screens so Button 8 in the recovery can't confirm a range
      setDist = false;
      packet.b_EMERGENCY = 1;
      packet.buttonID = '0';
      SendPacket();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("EMERGENCY!");
      lcd.setCursor(0, 1);
      lcd.print("SYSTEM SHUTDOWN");

      // Flash the backlight, resending the stop in case a packet was lost
      for (int i = 0; i < 5; i++) {
        lcd.noBacklight();
        delay(200);
        SendPacket();
        lcd.backlight();
        delay(200);
        SendPacket();
      }
    }
  } else {
    if (emergencyReading == LOW && button8Reading == LOW) {
      emergencyButtonState = false;
      Serial.println("EMERGENCY RECOVERED - SYSTEM CONTINUING OPERATION");

      ZeroMotionCommands();
      packet.b_EMERGENCY = 0;  // Releases the machine
      packet.buttonID = '8';
      SendPacket();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SYSTEM RECOVERED");
      lcd.setCursor(0, 1);
      lcd.print("RESUMING...");

      delay(2000);  // Time to let go of the buttons
    }
  }
}

void ReadJoystick() {
  leftJoystickxVal = analogRead(Left_Joystick_Left_Right_X);
  leftJoystickyVal = analogRead(Left_Joystick_Up_Down_Y);
  rightJoystickxVal = analogRead(Right_Joystick_Left_Right_X);
  rightJoystickyVal = analogRead(Right_Joystick_Up_Down_Y);

  JoystickLeftPWM(leftJoystickyVal, leftJoystickxVal);
  JoystickRightPWM(rightJoystickyVal, rightJoystickxVal);

  if (packet.radial_spd > 0) packet.rangeSet = 0;  // Moving the stage by hand cancels the range hold

  // Serial.print("Left Joystick X: ");
  // Serial.print(leftJoystickxVal);
  // Serial.print(" | Left Joystick Y: ");
  // Serial.print(leftJoystickyVal);
  // Serial.print(" || Right Joystick X: ");
  // Serial.print(rightJoystickxVal);
  // Serial.print(" | Right Joystick Y: ");
  // Serial.println(rightJoystickyVal);

  // Only used by the debug printout below. Its labels are wrong, see the pin definitions.
  bool leftJoystickxValCentered = abs(leftJoystickxVal - 512) < deadzone;
  bool leftJoystickyValCentered = abs(leftJoystickyVal - 512) < deadzone;
  bool rightJoystickxValCentered = abs(rightJoystickxVal - 512) < deadzone;
  bool rightJoystickyValCentered = abs(rightJoystickyVal - 512) < deadzone;

  /*
  // LEFT JOYSTICK X
  if (leftJoystickxVal < (512 - deadzone) && leftJoystickyValCentered) {
    Serial.println("Left Joystick X (Left): Tension Loose");
  } else if (leftJoystickxVal > (512 + deadzone) && leftJoystickyValCentered) {
    Serial.println("Left Joystick X (Right): Tension Tighten");
  }

  // LEFT JOYSTICK Y
  if (leftJoystickyVal < (512 - deadzone) && leftJoystickxValCentered) {
    Serial.println("Left Joystick Y (Up): Elevator Climbing");
  } else if (leftJoystickyVal > (512 + deadzone) && leftJoystickxValCentered) {
    Serial.println("Left Joystick Y (Down): Elevator Descending");
  }

  // RIGHT JOYSTICK X
  if (rightJoystickxVal < (512 - deadzone) && rightJoystickyValCentered) {
    Serial.println("Right Joystick X (Left): Circumferential Position Left");
  } else if (rightJoystickxVal > (512 + deadzone) && rightJoystickyValCentered) {
    Serial.println("Right Joystick X (Right): Circumferential Position Right");
  }

  // RIGHT JOYSTICK Y
  if (rightJoystickyVal < (512 - deadzone) && rightJoystickxValCentered) {
    Serial.println("Right Joystick Y (Up): Chainsaw In");
  } else if (rightJoystickyVal > (512 + deadzone) && rightJoystickxValCentered) {
    Serial.println("Right Joystick Y (Down): Chainsaw Out");
  }
  */
}

int CalibrateAxis(int pin) {  // Average resting reading of one stick axis
  long sum = 0;
  const int samples = 16;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  int center = (int)(sum / samples);
  if (abs(center - 512) > 150) return 512;  // Stick held or faulty, use mid-scale
  return center;
}

int AxisSpeed(int value, int center) {  // 0-193 by how far the stick is past the deadzone
  int distance = abs(value - center);
  if (distance <= deadzone) return 0;
  int output = map(distance - deadzone, 0, 512 - deadzone, 0, 193);
  if (output > 193) output = 193;
  if (output < joystickDeadzone) return 0;
  return output;
}

// Sets one motor's speed and direction from a stick axis. Direction and speed
// both change only outside the deadzone, so a motor never starts in a stale direction.
void ApplyAxis(int value, int center, uint8_t& speed, bool& dir) {
  if (value < center - deadzone) {
    dir = true;
    speed = (uint8_t)AxisSpeed(value, center);
  } else if (value > center + deadzone) {
    dir = false;
    speed = (uint8_t)AxisSpeed(value, center);
  } else {
    speed = 0;
  }
}

void JoystickLeftPWM(int joystickxVal, int joystickyVal) {
  // First argument is A1 (winch), second is A0 (traction)
  ApplyAxis(joystickxVal, centerLeftY, packet.winch_spd, packet.winch_dir);
  ApplyAxis(joystickyVal, centerLeftX, packet.traction_spd, packet.traction_dir);
}

void JoystickRightPWM(int joystickxVal, int joystickyVal) {
  // First argument is A9 (circumferential), second is A8 (radial)
  ApplyAxis(joystickxVal, centerRightY, packet.circum_spd, packet.circum_dir);
  ApplyAxis(joystickyVal, centerRightX, packet.radial_spd, packet.radial_dir);
}

void SendPacket() {
  radio.stopListening();
  bool sent = radio.write(&packet, sizeof(packet));

  if (sent) {

    Serial.println(" Packet Sent Successfully! -> ");
    /*
    Serial.print("Button: ");
    Serial.print(packet.buttonID);

    Serial.print(" Left Joystick X: ");
    Serial.print(packet.winch_spd);
    Serial.print(" Left Joystick Y: ");
    Serial.print(packet.traction_spd);

    Serial.print(" Right Joystick X: ");
    Serial.print(packet.circum_spd);
    Serial.print(" Right Joystick Y: ");
    Serial.println(packet.radial_spd);
*/
    if (radio.isAckPayloadAvailable()) {  // The machine's reply: load, range, status
      radio.read(ackArr, sizeof(ackArr));
      //Serial.print("Received ACK array: ");
      //Serial.print(ackArr[0]);
      //Serial.print(',');
      //Serial.print(ackArr[1]);
    }
  } else {
    Serial.println("Packet Sent Failed!");
  }
}

uint8_t PotToTension() {
  return map(analogRead(Potentiometer), 0, 1023, 10, 175);
}

uint8_t PotToRange() {
  return map(analogRead(Potentiometer), 0, 1023, RANGE_SET_MIN_CM, RANGE_SET_MAX_CM);
}

void ShowConfirm(uint8_t value, const char* unit) {  // "Confirmed N" for 1.5 s
  confirmValue = value;
  confirmUnit = unit;
  confirmUntil = millis() + 1500;
}

void LCDLine(uint8_t row, const char* text) {  // Pads to 16 chars to overwrite old text
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void LCDScreen() {  // Setpoint screen if one is open, otherwise load and range
  char line[17];

  if (setTens) {
    mapPotVal = PotToTension();
    snprintf(line, sizeof(line), "Set Load: %u lb", mapPotVal);
    LCDLine(0, line);
    LCDLine(1, "Btn 4 = confirm");
    return;
  }
  if (setDist) {
    mapPotVal = PotToRange();
    snprintf(line, sizeof(line), "Set Range: %u cm", mapPotVal);
    LCDLine(0, line);
    LCDLine(1, "Btn 8 = confirm");
    return;
  }

  if ((long)(confirmUntil - millis()) > 0) {
    snprintf(line, sizeof(line), "Confirmed %u%s", confirmValue, confirmUnit);
  } else if (ackArr[0] == 255) {
    snprintf(line, sizeof(line), "Load: 255+ lbs");
  } else {
    snprintf(line, sizeof(line), "Load: %u lbs", ackArr[0]);
  }
  LCDLine(0, line);

  char range[5];
  if (ackArr[1] == 255) {
    snprintf(range, sizeof(range), "--");  // No echo
  } else if (ackArr[1] > 200) {
    snprintf(range, sizeof(range), "200+");
  } else {
    snprintf(range, sizeof(range), "%u", ackArr[1]);
  }

  if (ackArr[2] & ACK_RANGE_FAULT) {
    snprintf(line, sizeof(line), "Rng:%scm FAULT", range);
  } else if (ackArr[2] & ACK_RANGE_HOLD) {
    snprintf(line, sizeof(line), "Rng:%scm H:%u", range, packet.rangeSet);
  } else {
    snprintf(line, sizeof(line), "Range: %s cm", range);
  }
  LCDLine(1, line);
}

void loop() {
  CheckEmergencyButton();
  if (emergencyButtonState) {
    ZeroMotionCommands();
    packet.b_EMERGENCY = 1;
    packet.buttonID = '0';
    SendPacket();  // Keep sending the stop until recovery
    return;
  }

  // Joystick packet every loop, plus one extra packet per button press
  ReadJoystick();
  packet.buttonID = '0';
  SendPacket();

  CheckButtonDebouncePressed(Tension_On_Button, 0, "Tension On Button");
  CheckButtonDebouncePressed(Tension_Off_Button, 1, "Tension Off Button");
  CheckButtonDebouncePressed(Tension_Set_Point_Button, 2, "Tension Set Point Button");
  CheckButtonDebouncePressed(Tension_Indicated_Button, 3, "Tension Indicated Button");
  CheckButtonDebouncePressed(Chainsaw_On_Button, 4, "Chainsaw On Button");
  CheckButtonDebouncePressed(Chainsaw_Off_Button, 5, "Chainsaw Off Button");
  CheckButtonDebouncePressed(Chainsaw_Distance_Set_Point_Button, 6, "Chainsaw Distance Set Point Button");
  CheckButtonDebouncePressed(Chainsaw_Distance_Indicated_Button, 7, "Chainsaw Distance Indicated Button");

  LCDScreen();
}