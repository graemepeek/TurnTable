// Author: Graeme Peek (The Village Workshop)
// Turntable for 3D scanning.

#include <EEPROM.h>
#include "TurnTable.h"
#include "Encoder.h"
#include "Potentiometer.h"

// #define DEBUG_BUILD

String Version = "V1.0.0*";

// declare IO variables and their address
int directionPin            = 8;  // Direction output to stepper controller
int stepPin                 = 9;  // Step signal to stepper controller
int clockWisePin            = 2;
int counterClockWisePin     = 3;
int shutterTriggerPin       = 13; // Signal to scanner to command a scan
int CaptureComplete         = 5;  // Signal back from scanner to say scan complete

// declare EEPROM variables
int stepperSpeed            = 0;
int stepDegree              = 0;
int Mode                    = 0;
int SpeedMultiplier         = 0;

// declare EEPROM address
int ModeSettingAddress      = 1;
int StepperSpeedAddress     = 2;
int StepDegreeAddress       = 3;
int SpeedMultiplierAddress  = 4;

// declare trigger varibles 
bool MoveClockWise          = LOW;
bool MoveCounterClockWise   = LOW;

bool quitWaitingForCapture  = LOW;

bool WaitingForTriggerInput = LOW;

// Defines here (enumerations)
#define MODE_DIGITAL_INPUTS 0
#define MODE_USE_SERIAL     1
#define MODE_STEP_DEGREES   2
#define MODE_STEP_WITHOUT_DIGITAL 3

int StepCounter = 0;

// Setup instances of our classes.
TurnTable thisTable(stepPin, directionPin, clockWisePin, counterClockWisePin);

void setup() {
  // Setup system:
  Mode            = EEPROM.read(ModeSettingAddress);
  stepperSpeed    = EEPROM.read(StepperSpeedAddress);
  stepDegree      = EEPROM.read(StepDegreeAddress);
  SpeedMultiplier = EEPROM.read(SpeedMultiplierAddress);
  
  Serial.begin(19200); // Setup serial baud rate
  Serial.print("3D Scanner Turntable Control Version:");
  Serial.println(Version);
  Serial.println("H Displays Help");
  Serial.println("? Displays Current Settings");
   
  // Setup IO 
  thisTable.setup();
  pinMode(shutterTriggerPin, OUTPUT);
  //Setup Input pins
  pinMode(CaptureComplete,INPUT_PULLUP);

}

void loop()
{
  
  ReadSerial(); // Call read serial data routine
  // Select mode Mode
  switch(Mode){
   case MODE_DIGITAL_INPUTS: // Look for pin to enable clockwise travel to be high or low
       if (!digitalRead(clockWisePin))
        {
          MoveClockWise = HIGH;
        }
      else
        {
          MoveClockWise = LOW;
        }
      // Look for pin to enable counter clockwise travel to be high or low
      if (!digitalRead(counterClockWisePin))
        {
          MoveCounterClockWise = HIGH;
        }
      else
        {
         MoveCounterClockWise = LOW;
        }
      if (MoveClockWise == LOW && MoveCounterClockWise == LOW) {
          thisTable.stopMotion();
      }
    break;
    
    case MODE_USE_SERIAL: // Get start/stop commands from serial
    // if no motion requested stop all motion
    if (MoveClockWise == LOW && MoveCounterClockWise == LOW) {
      thisTable.stopMotion();
    }
    break;
    
    case MODE_STEP_DEGREES: // Turn table in steps, wait then step again.
        StepCounter = 0;
        while(StepCounter < stepDegree){
            thisTable.startMotionCW(stepperSpeed,SpeedMultiplier);
            #ifdef DEBUG_BUILD
              Serial.println("Motion Started");
              Serial.println("Stepping Mode");
             #endif
            StepCounter ++;
            quitWaitingForCapture = LOW;
        }
            thisTable.stopMotion();
            Serial.println("S");
            digitalWrite(shutterTriggerPin, HIGH);
            while(digitalRead(CaptureComplete)) {  
              ReadSerial(); 
              #ifdef DEBUG_BUILD
                Serial.println(quitWaitingForCapture);  
              #endif
              if(quitWaitingForCapture){
                break;      
              }
            }
            digitalWrite(shutterTriggerPin, LOW);
        break;

    case MODE_STEP_WITHOUT_DIGITAL: // Turn table in steps, wait then step again.
        StepCounter = 0;
        #ifdef DEBUG_BUILD
              Serial.println("Motion Starting");
              Serial.println("Stepping Mode");
             #endif
        thisTable.startMotionCW(stepperSpeed,SpeedMultiplier);
        while(StepCounter < stepDegree){
            StepCounter ++;
            quitWaitingForCapture = LOW;
        }
            thisTable.stopMotion();
            Serial.println("S");
            digitalWrite(shutterTriggerPin, HIGH);
            delay(500);
            digitalWrite(shutterTriggerPin, LOW);
            Serial.println("Motion Stopped, Waiting for R to be received");
            while(WaitingForTriggerInput = LOW){
            ReadSerial();
            }
            WaitingForTriggerInput = LOW;          
    }
   
   if (MoveClockWise == HIGH){
      thisTable.startMotionCW(stepperSpeed,SpeedMultiplier);
   }

  if (MoveCounterClockWise == HIGH){
   thisTable.startMotionCCW(stepperSpeed,SpeedMultiplier);
  } 
}

// Get data from USB port
void ReadSerial(){
  String readString;
  int StepDelay = stepperSpeed * SpeedMultiplier;
   
  // We check to see if any new data has come into the serial port from the Android device
  if (Serial.available() > 0) {
    int SerialInByte = Serial.read();
    quitWaitingForCapture = HIGH;    
    switch(SerialInByte){
      case 'A': // Counter Clockwise
        Serial.println("[OK:CCW]");
        MoveClockWise = HIGH;
        MoveCounterClockWise = LOW;
        break;
        
      case 'a': // Counter Clockwise
        Serial.println("[OK:CCW]");
        MoveClockWise = HIGH;
        MoveCounterClockWise = LOW;
        break;
        
      case 'S': // Stop
        MoveClockWise = LOW;
        MoveCounterClockWise = LOW;
        Serial.println("[OK:Stop]");
        break;
        
      case 's': // Stop
        MoveClockWise = LOW;
        MoveCounterClockWise = LOW;
        Serial.println("[OK:Stop]");
        break;
        
      case 'C': // Clockwise
        Serial.println("[OK:CW]");
        MoveCounterClockWise = HIGH;
        MoveClockWise = LOW;
        break;
        
      case 'c': // Clockwise
        Serial.println("[OK:CW]");
        MoveCounterClockWise = HIGH;
        MoveClockWise = LOW;
        break;

      case 'R': // Resume capture
        Serial.println("[OK:R]");
        WaitingForTriggerInput = HIGH;
        break;
        
       case 'r': // Resume capture
        Serial.println("[OK:R]");
        WaitingForTriggerInput = HIGH;
        break;
         
      case '?': //Get Settings
          Serial.print("Version:");
          Serial.println(Version);
          Serial.print("Speed:");
          Serial.println(stepperSpeed);
          Serial.print("Mode:");
          Serial.println(Mode);
          Serial.print("Step Angle:");
          Serial.println(stepDegree);
          Serial.print("Speed Multiplier:");
          Serial.println(SpeedMultiplier);
          Serial.print("PWM Delay Value = ");
          Serial.println(StepDelay);
          break;
        
      case 'H': //Display Help
          Serial.println("");
          Serial.println("**************HELP****************");
          Serial.println("? Displays Current Settings");
          Serial.println("A Start table in clockwise direction");
          Serial.println("C Start table in counter clockwise");
          Serial.println("S Stop table");
          Serial.println("Mn Set mode (M0 Mode 0 (Use digital Input Pins), M1 Mode 1 (Use Serial Input), M2 Mode 2 (Step in degrees) M3 Mode 3 (Step Degrees without waiting for capture complete)");
          Serial.println("Dn Set step degrees (x is degrees of step)");
          Serial.println("Xnnnn Set Speed of table (Where n is speed)");
          Serial.println("Ln Set Speed Multiplier (Where n is the multipler)");
          Serial.println("Pin 5 is input signal Capture complete from scanner");
          Serial.println("Pin 13 is output signal to scanner to trigger a scan cycle");
          Serial.println("Pin 2 is digital input signal to trigger clockwise move");
          Serial.println("Pin 3 is digital input signal to trigger counter clockwise move");
          Serial.println("Pin 8 is digital input signal for direction");
          Serial.println("Pin 9 is digital input signal for step signal");
        break;
        
      case 'M': // Mode
        readString = Serial.readString(); //makes the string readString
        Serial.print("[OK: Mode Setting = ");
        Mode = readString.toInt();
        Serial.print(Mode);
        Serial.println("]");
        EEPROM.write(ModeSettingAddress, Mode);
        break;
        
      case 'X': // Speed Setting
        readString = Serial.readString(); //makes the string readString
        stepperSpeed = readString.toInt();
        if (stepperSpeed < 256){
          Serial.print("[OK: Speed Setting = ");
          Serial.print(stepperSpeed);
          Serial.println("]");
          EEPROM.write(StepperSpeedAddress, stepperSpeed);
          }
        else
          {
            Serial.print("[ERROR:");
            Serial.print("Speed Setting Must be Value 0..255");
            Serial.println("]");
            stepperSpeed = EEPROM.read(StepperSpeedAddress);
          }
        break;
        
      case 'D': // Step Degree setting
        readString = Serial.readString(); //makes the string readString
        Serial.print("[OK: Degree Setting = ");
        stepDegree = readString.toInt();
        Serial.print(stepDegree);
        Serial.println("]");
        EEPROM.write(StepDegreeAddress, stepDegree);
        break;

       case 'L': // Speed Setting
        readString = Serial.readString(); //makes the string readString
        SpeedMultiplier = readString.toInt();
        if (SpeedMultiplier < 256){
          Serial.print("[OK:");
          Serial.print(SpeedMultiplier);
          Serial.println("]");
          EEPROM.write(SpeedMultiplierAddress, SpeedMultiplier);
          }
        else
          {
            Serial.print("[ERROR:");
            Serial.print("Speed Multiplier Must be Value 0..255");
            Serial.println("]");
            SpeedMultiplier = EEPROM.read(SpeedMultiplierAddress);
          }
        break;
    }
  }
}

