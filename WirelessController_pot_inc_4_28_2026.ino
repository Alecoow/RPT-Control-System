// Wireless Controller File
// Version 1.1 4-28-2026
// Changes made: Allows for tension to be set to an upper limit of 175 lbs when using pot in dcts.
// Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "printf.h"

// Defines Buttons
#define Tension_On_Button 23                   // Left Side Button 1 (Pin 24)
#define Tension_Off_Button 25                  // Left Side Button 2 (Pin 26)
#define Tension_Set_Point_Button 27            // Left Side Button 3 (Pin 28)
#define Tension_Indicated_Button 29            // Left Side Button 4 (Pin 30)
#define Chainsaw_On_Button 24                  // Right Side Button 5 (Pin 23)
#define Chainsaw_Off_Button 26                 // Right Side Button 6 (Pin 25)
#define Chainsaw_Distance_Set_Point_Button 28  // Right Side Button 7 (Pin 27)
#define Chainsaw_Distance_Indicated_Button 30  // Right Side Button 8 (Pin 29)
#define Emergency_Button 22                    // Button 9 (Pin 36)

// Define Joysticks
#define Left_Joystick_Left_Right_X A0   // Left Joystick Horizontal (X) Direction For Tension Loose/Tighten (Pin A0)
#define Left_Joystick_Up_Down_Y A1      // Left Joystick Vertical (Y) Direction For Elevator Climbing/Descending (Pin A1)
#define Right_Joystick_Left_Right_X A8  // Right Joystick Horizontal (X) Direction For Circumferential Position (Pin A8)
#define Right_Joystick_Up_Down_Y A9    // Right Joystick Vertical (Y) Direction For Chainsaw In/Out (Pin A10)
#define Potentiometer A4

// Define NRF24
#define CE_PIN 49   // Chip Enable pin (activates TX or RX mode)
#define CSN_PIN 53  // Chip Select Not pin (SPI communication control)

// Define LCD Screen
#define SDA_PIN 20  // I2C Data Line Pin (Not used in the code, initialized for clarification)
#define SCL_PIN 21  // I2C Clock Line Pin (Not used in the code, initialized for clarification)

// Debounce Settings
const unsigned long debounceDelay = 25;  // The debounce time in ms to consider a press stable


// Arrays to track last state and debounce timing for each button (initially, all buttons are LOW state and 0 debounce time)
int lastButtonState[8] = { LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW };                 // Stores last state (HIGH or LOW) for each button
unsigned long lastDebounceTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };                      // Stores last time the output pin was toggled (change was detected)
bool buttonPressed[8] = { false, false, false, false, false, false, false, false };  // if a button is pressed (active = true)

// Track if emergency button is pressed (active = true)
bool emergencyButtonState = false;
bool setTens = false;
bool setDist = false;
bool conf = 0;
// Joysticks Settings
int leftJoystickxVal;
int leftJoystickyVal;
int rightJoystickxVal;
int rightJoystickyVal;
int deadzone = 100;
int joystickDeadzone = 15;

// Potentiometer Settings
uint16_t potVal = 0;  // Potentiometer value
uint8_t mapPotVal = 0;

// LCD Screen Settings
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for 16 chars and 2 line display
uint16_t potCTR = 0;

// NRF24 Settings *** NOTE *** : NRF24L01+PA+LNA USES 3.3V ON THE ARDUINO NOT 5V. CONNECTING IT TO 5V WILL FRY IT
RF24 radio(CE_PIN, CSN_PIN);
const byte address[5] = { 'T', 'E', 'S', 'T', '1' };  // Unique address for communication (must match on both ends)
const char message[] = "Transmission Testing. . . ";
char receivedData[32];

// Structure to send controller data
struct ControllerPacket {
  char buttonID;  // '#' to '#', 'E' for emergency button, '0' for nothing
  bool b_EMERGENCY;
  uint8_t winch_spd;  // Left joystick horizontal direction data
  bool winch_dir;
  uint8_t traction_spd;  // Left joystick vertical direction data
  bool traction_dir;
  uint8_t circum_spd;  // Right joystick horizontal direction data
  bool circum_dir;
  uint8_t radial_spd;  // Right joystick vertical direction data
  bool radial_dir;
  uint8_t tensionSet;
};

ControllerPacket packet = { '9', 0, 0, 0, 0, 0, 0, 0, 0, 0, 20 };

// Initliaze and decalre receiving variables
uint8_t ackArr[2] = { 0, 0 };

void ZeroMotionCommands() {
  packet.winch_spd = 0;
  packet.traction_spd = 0;
  packet.circum_spd = 0;
  packet.radial_spd = 0;
}

void setup() {
  Serial.begin(115200);  // Start Serial Monitor

  // Set Pin Modes
  // Note: All buttons are connected with internal pull-up resistors (reads LOW when not pressed)
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

  // Radio Setup Settings
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setAutoAck(true);
  radio.setChannel(76);
  radio.openWritingPipe(address);  // Where we send data
  printf_begin();
  radio.printPrettyDetails();

  // LCD Screen Setup Settings
  lcd.init();       // Initialize the LCD
  lcd.backlight();  // Turn on the backlight and print a message
  LCDScreen();
}

// Function to check for button debounce and prints the name of the button if it is pressed
void CheckButtonDebouncePressed(int pin, int i, const char* buttonName) {
  int reading = digitalRead(pin);

  // If the reading is different from the last time, reset the debounce timer
  if (reading != lastButtonState[i]) {
    lastDebounceTime[i] = millis();  // Save debounce time
  }

  // Update the >> specific << button (at index i) state if it has stayed the same for debounceDelay time
  if ((millis() - lastDebounceTime[i]) > debounceDelay) {
    // If the button state has changed (stable reading)
    if (reading != buttonPressed[i]) {
      buttonPressed[i] = reading;  // Save the new button pressed state

      if (buttonPressed[i] == LOW) {  // Only do action on LOW (pressed)
        Serial.print("Button ");
        Serial.print(i + 1);
        Serial.print(" Pressed: ");
        Serial.println(buttonName);

        if (i == 2) {
          setTens = !setTens;
          conf = 0;
        }
        if (i == 6) {
          setDist = !setDist;
          conf = 0;
        }

        packet.buttonID = '1' + i;
        SendPacket();
      }
    }
  }

  // Update last known state button
  lastButtonState[i] = reading;
}

void CheckEmergencyButton() {
  int emergencyReading = digitalRead(Emergency_Button);
  int button8Reading = digitalRead(Chainsaw_Distance_Indicated_Button);  // Button 8

  if (!emergencyButtonState) {
    // NORMAL operation: Detect if Emergency Button is pressed to enter EMERGENCY
    if (emergencyReading == LOW) {
      emergencyButtonState = true;  // Enter emergency mode
      Serial.println("EMERGENCY BUTTON PRESSED - SYSTEM SHUTDOWN!");

      ZeroMotionCommands();
      packet.b_EMERGENCY = 1;
      packet.buttonID = '0';
      SendPacket();

      // LCD Print Warning
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("EMERGENCY!");
      lcd.setCursor(0, 1);
      lcd.print("SYSTEM SHUTDOWN");

      // Flash LCD
      for (int i = 0; i < 5; i++) {
        lcd.noBacklight();
        delay(200);
        SendPacket();  // Retry the emergency state while the display is blocking
        lcd.backlight();
        delay(200);
        SendPacket();
      }
    }
  } else {
    // EMERGENCY MODE: Detect if Emergency Button + Button 8 are pressed together to recover
    if (emergencyReading == LOW && button8Reading == LOW) {
      emergencyButtonState = false;  // Exit emergency mode
      Serial.println("EMERGENCY RECOVERED - SYSTEM CONTINUING OPERATION");

      ZeroMotionCommands();
      packet.b_EMERGENCY = 0;  // Notify system recovery
      packet.buttonID = '8';
      SendPacket();

      // LCD Recovery Message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SYSTEM RECOVERED");
      lcd.setCursor(0, 1);
      lcd.print("RESUMING...");

      delay(2000);  // Small recovery delay to avoid re-triggering immediately
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

  // Serial.print("Left Joystick X: ");
  // Serial.print(leftJoystickxVal);
  // Serial.print(" | Left Joystick Y: ");
  // Serial.print(leftJoystickyVal);
  // Serial.print(" || Right Joystick X: ");
  // Serial.print(rightJoystickxVal);
  // Serial.print(" | Right Joystick Y: ");
  // Serial.println(rightJoystickyVal);
  

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

void JoystickLeftPWM(int joystickxVal, int joystickyVal) {
  int distanceFromCenterX = abs(joystickxVal - 512) * 2;
  int outputValueX = map(distanceFromCenterX, 0, 1024, 0, 193);

  if (outputValueX < joystickDeadzone) {
    outputValueX = 0;
  }

  packet.winch_spd = outputValueX;

  if (joystickxVal < (512 - deadzone)) {
    packet.winch_dir = true;
  } else if (joystickxVal > (512 + deadzone)) {
    packet.winch_dir = false;
  }

  int distanceFromCenterY = abs(joystickyVal - 512) * 2;
  int outputValueY = map(distanceFromCenterY, 0, 1024, 0, 193);

  if (outputValueY < joystickDeadzone) {
    outputValueY = 0;
  }

  if (joystickyVal < (512 - deadzone)) {
    packet.traction_dir = true;
  } else if (joystickyVal > (512 + deadzone)) {
    packet.traction_dir = false;
  }

  packet.traction_spd = outputValueY;
}

void JoystickRightPWM(int joystickxVal, int joystickyVal) {
  int distanceFromCenterX = abs(joystickxVal - 512) * 2;
  int outputValueX = map(distanceFromCenterX, 0, 1024, 0, 193);

  if (outputValueX < joystickDeadzone) {
    outputValueX = 0;
  }

  packet.circum_spd = outputValueX;

  if (joystickxVal < (512 - deadzone)) {
    packet.circum_dir = true;
  } else if (joystickxVal > (512 + deadzone)) {
    packet.circum_dir = false;
  }

  int distanceFromCenterY = abs(joystickyVal - 512) * 2;
  int outputValueY = map(distanceFromCenterY, 0, 1024, 0, 193);

  if (outputValueY < joystickDeadzone) {
    outputValueY = 0;
  }

  packet.radial_spd = outputValueY;

  if (joystickyVal < (512 - deadzone)) {
    packet.radial_dir = true;
  } else if (joystickyVal > (512 + deadzone)) {
    packet.radial_dir = false;
  }
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
    if (radio.isAckPayloadAvailable()) {
      // Read back exactly sizeof(ackArr) bytes
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

// void LCDScreen() {
//   //lcd.clear();

//   //lcd.print("Load: ");
//   //lcd.print(mapPotVal);

//   lcd.setCursor(0, 0);  // Start of first line (first row, first column)

//   if (setTens) {
//     lcd.print("Set Load: ");
//     lcd.print(mapPotVal);
//     setDist = 0;
//     packet.tensionSet = mapPotVal;
//   } else if (setDist) {
//     lcd.print("Set Range: ");
//     lcd.print(mapPotVal);
//     setTens = 0;
//   } else {
//     lcd.print("Load: ");
//     if (ackArr[0] == 255) {
//       lcd.print("255+");
//       lcd.print(" lbs");
//     } else {
//       lcd.print(ackArr[0]);
//       lcd.print(" lbs");
//     }

//     lcd.setCursor(0, 1);  // Start of second line (second row, first column)
//     lcd.print("Range: ");
//     if (ackArr[1] > 200) {
//       lcd.print("200+");
//       lcd.print(" cm");
//     } else {
//       lcd.print(ackArr[1]);
//       lcd.print(" cm");
//     }
//   }
// }

void LCDScreen() {
  if (setTens) {
    if (packet.buttonID == '4' || conf) {
      lcd.setCursor(0, 0);
      lcd.print("Confirmed ");
      lcd.print(mapPotVal);
      lcd.print(" lbs");
      lcd.print("       ");
      conf = 1;
      packet.tensionSet = mapPotVal;
      lcd.setCursor(0, 1);
      lcd.print("              ");
    } else {
      lcd.setCursor(0, 0);
      potVal = analogRead(Potentiometer);
      mapPotVal = map(potVal, 0, 1023, 0, 175);
      lcd.print("Set Load: ");
      lcd.print(mapPotVal);
      lcd.print(" lbs");
      lcd.print(" ");  // Pad spaces
      lcd.setCursor(0, 1);
      lcd.print("              ");
    }
  } else if (setDist) {
    if (packet.buttonID == '8' || conf) {
      lcd.setCursor(0, 0);
      lcd.print("Confirmed ");
      lcd.print(mapPotVal);
      lcd.print(" cm");
      lcd.print("       ");
      conf = 1;
      lcd.setCursor(0, 1);
      lcd.print("              ");
    } else {
      potVal = analogRead(Potentiometer);
      mapPotVal = map(potVal, 0, 1023, 0, 99);

      lcd.setCursor(0, 0);
      lcd.print("Set Range: ");
      lcd.print(mapPotVal);
      lcd.print(" cm");
      lcd.print("   ");  // Pad spaces
      lcd.setCursor(0, 1);
      lcd.print("              ");
    }
  } else {
    // Update Load if changed
    lcd.setCursor(0, 0);
    lcd.print("Load: ");
    if (ackArr[0] == 255) {
      lcd.print("255+ lbs ");
      lcd.print("    ");
    } else {
      lcd.print(ackArr[0]);
      lcd.print(" lbs ");
      lcd.print("    ");
    }


    // Update Range if changed
    lcd.setCursor(0, 1);
    lcd.print("Range: ");
    if (ackArr[1] > 200) {
      lcd.print("200+ cm ");
    } else {
      lcd.print(ackArr[1]);
      lcd.print(" cm ");
    }
  }
}

void loop() {
  // Always check emergency button first
  CheckEmergencyButton();
  if (emergencyButtonState) {
    ZeroMotionCommands();
    packet.b_EMERGENCY = 1;
    packet.buttonID = '0';
    SendPacket();  // Keep asserting the stop until the recovery packet is sent
    return;
  }

  // Read Joystick Movements
  ReadJoystick();

  // If not button pressed, send only current joysticks data
  packet.buttonID = '0';
  SendPacket();

  // Check each button debounce/pressed
  CheckButtonDebouncePressed(Tension_On_Button, 0, "Tension On Button");
  CheckButtonDebouncePressed(Tension_Off_Button, 1, "Tension Off Button");
  CheckButtonDebouncePressed(Tension_Set_Point_Button, 2, "Tension Set Point Button");
  CheckButtonDebouncePressed(Tension_Indicated_Button, 3, "Tension Indicated Button");
  CheckButtonDebouncePressed(Chainsaw_On_Button, 4, "Chainsaw On Button");
  CheckButtonDebouncePressed(Chainsaw_Off_Button, 5, "Chainsaw Off Button");
  CheckButtonDebouncePressed(Chainsaw_Distance_Set_Point_Button, 6, "Chainsaw Distance Set Point Button");
  CheckButtonDebouncePressed(Chainsaw_Distance_Indicated_Button, 7, "Chainsaw Distance Indicated Button");

  // Display Values on LCD Screen

  LCDScreen();


  //delay(500);  // Slight pause to reduce Serial spam & avoid false readings
}