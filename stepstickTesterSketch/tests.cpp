#include "Arduino.h"
#include "tests.h"
#include "config.h"
#include "util.h"
#include "stepper.h"
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

extern SSD1306AsciiWire display;
extern Stepper stepper;

extern int testSpeedVar;

bool testFunction_testInputVoltage() {
  bool failed = false;
  
  float input_rail = dividerVoltage(analogRead(POWER_BOARD_INPUT_TEST), INPUT_RAIL_DIV_R1, INPUT_RAIL_DIV_R2);
  Serial.print(F("Testing input voltage... "));
  Serial.print(input_rail);
  Serial.print(F("V... "));

  display.print(F("Input power... "));

  if(input_rail < INPUT_RAIL_THRESHOLD) {
    failed = true;
  } 

  return failed;
}


bool testFunction_test12V() {
  bool failed = false;

  Serial.print(F("Enabling 12V line... "));
  display.print(F("12V line... "));

  float input_rail = dividerVoltage(analogRead(POWER_BOARD_INPUT_TEST), INPUT_RAIL_DIV_R1, INPUT_RAIL_DIV_R2);

  digitalWrite(POWER_MOTOR_SUPPLY_EN,  LOW);
  delay(RAIL_SHORT_WAIT_TIME);  

  float supply_rail = dividerVoltage(analogRead(POWER_MOTOR_SUPPLY_TEST), SUPPLY_RAIL_DIV_R1, SUPPLY_RAIL_DIV_R2);

  if(supply_rail < (input_rail - LOGIC_RAIL_THRESHOLD)) {
    failed = true;
  } 
  
  return failed;
}

bool testFunction_test5V() {
  bool failed = false;

  Serial.print(F("Enabling 5V line... "));
  display.print(F("5V line... "));

  digitalWrite(POWER_MOTOR_LOGIC_EN,  LOW);
  delay(RAIL_SHORT_WAIT_TIME);   
  float logic_rail = dividerVoltage(analogRead(POWER_MOTOR_LOGIC_TEST), LOGIC_RAIL_DIV_R1, LOGIC_RAIL_DIV_R2);

  if(logic_rail < LOGIC_RAIL_THRESHOLD) {
    failed = true;
  } 

  return failed;
}

bool testFunction_powerDown() {
  bool failed = false;
  
  Serial.print(F("Powering down... "));
  display.print(F("Power down... "));

  digitalWrite(POWER_MOTOR_LOGIC_EN,  HIGH);
  digitalWrite(POWER_MOTOR_SUPPLY_EN, HIGH);
  stepper.enableMotor(false);

  delay(RAIL_SHORT_WAIT_TIME);   

  unsigned long startPowerDownTime = millis();
  float logic_rail, supply_rail;

  do {
    logic_rail  = dividerVoltage(analogRead(POWER_MOTOR_LOGIC_TEST), LOGIC_RAIL_DIV_R1, LOGIC_RAIL_DIV_R2);
    supply_rail = dividerVoltage(analogRead(POWER_MOTOR_SUPPLY_TEST), SUPPLY_RAIL_DIV_R1, SUPPLY_RAIL_DIV_R2);

    if((millis() - startPowerDownTime) > POWER_DOWN_TIMEOUT) {
      failed = true;
      break;
    } 
  }  while(logic_rail > POWER_DOWN_THRESHOLD || supply_rail > POWER_DOWN_THRESHOLD);
  return failed;
}

bool testFunction_testStepperMotion() {
  bool failed = false;

  Serial.print(F("Driver has "));
  Serial.print(stepper.getMicrosteppingModes());
  Serial.println(F(" microstepping modes."));
  
  //display.print(stepper.getMicrosteppingModes());
  //display.print(F("DRV has "));
  //display.println(F(" MS modes"));
  //display.display();

  for(int i = 0; i < stepper.getMicrosteppingModes(); i++) {
    int startPosition = readEncoderAngle(); 
    int angularDiff;
    
    if(!failed) {
      stepper.setMicrosteppingMode(i);   
      stepper.setMotorSpeed(testSpeedVar); 
      Serial.print(F("Testing microstepping mode "));
      Serial.print(i);
      Serial.print(F(" ("));
      Serial.print(stepper.getMicrosteppingMultiplier(i));
      Serial.println(F("x microstepping)"));   

      display.print(F("MS mode "));
      display.print(i);
      display.print(F(" ("));
      display.print(stepper.getMicrosteppingMultiplier(i));
      display.print(F("x)"));
    }
    
    /////////////////////////////////////////////////////////////////////
    // Rotate by X degrees, tests MS settings and STEP pin.
    /////////////////////////////////////////////////////////////////////
    if(!failed) {
      Serial.print(F("Forward rotate test... "));
      stepper.setDirectionForward(true);
      stepper.enableMotor(true);  
      delay(100); 
      stepper.moveMotor((float)MOTOR_TEST_ROTATE_DEGREES / 360.0f);   //move forward by X degrees

      delay(MOTOR_TEST_REST_TIME);

      if(HAVE_ROTATIONAL_ENCODER) {
        int forwardPosition = readEncoderAngle();
        angularDiff = abs(angularDifference(startPosition, forwardPosition));

        Serial.print(F("Start angle: "));
        Serial.print(startPosition);
        Serial.print(F(", end angle: "));
        Serial.print(forwardPosition);
        Serial.print(F(", difference: "));
        Serial.println(angularDiff);
        
        if(abs(angularDiff - MOTOR_TEST_ROTATE_DEGREES) > MOTOR_TEST_ANGULAR_TOLERANCE) {
          failed = true;
          Serial.println(F("failed!"));
        } else {
          Serial.println(F("passed!"));
        }
      } else {
        Serial.println(F("done."));
      }
    }

    /////////////////////////////////////////////////////////////////////
    // Reverse direction, rotate back by X degrees. Tests DIR pin.
    /////////////////////////////////////////////////////////////////////
    if(!failed) {
      Serial.print(F("Reverse rotate test... "));
      stepper.setDirectionForward(false);
      delay(100);
      stepper.moveMotor((float)MOTOR_TEST_ROTATE_DEGREES / 360.0f);   //move back by X degrees

      delay(MOTOR_TEST_REST_TIME);

      if(HAVE_ROTATIONAL_ENCODER) {
        int returnedPosition = readEncoderAngle();
        angularDiff = abs(angularDifference(startPosition, returnedPosition));

        Serial.print(F("Start angle: "));
        Serial.print(startPosition);
        Serial.print(F(", end angle: "));
        Serial.print(returnedPosition);
        Serial.print(F(", difference: "));
        Serial.println(angularDiff);
        
        if(abs(angularDiff) > MOTOR_TEST_ANGULAR_TOLERANCE) {
          failed = true;
          Serial.println(F("failed!"));
        } else {
          Serial.println(F("passed!"));
        }
      } else {
        Serial.println(F("done."));
      }
    }
    
    /////////////////////////////////////////////////////////////////////
    // Disable driver, attempt rotate by X degrees. Tests EN pin.
    /////////////////////////////////////////////////////////////////////
    if(!failed) {
      Serial.print(F("Disabled rotate test... "));
      stepper.setDirectionForward(true);
      stepper.enableMotor(false);
      delay(100);
      stepper.moveMotor((float)20 / 360.0f);   //move forward by X degrees

      delay(MOTOR_TEST_REST_TIME);

      if(HAVE_ROTATIONAL_ENCODER) {
        int disabledPosition = readEncoderAngle();
        angularDiff = abs(angularDifference(startPosition, disabledPosition));

        Serial.print(F("Start angle: "));
        Serial.print(startPosition);
        Serial.print(F(", end angle: "));
        Serial.print(disabledPosition);
        Serial.print(F(", difference: "));
        Serial.println(angularDiff);
    
        if(abs(angularDiff) > MOTOR_TEST_ANGULAR_TOLERANCE) {
          failed = true;
          Serial.println(F("failed!"));
        } else {
          Serial.println(F("passed!"));
        }  
      } else {
        Serial.println(F("done."));
      }
    }
    if(failed) {
      alignCursorRight(4);
      display.println(F("fail"));
      break;
    } else {
      alignCursorRight(4);
      display.println(F("pass"));
    }
  }
  display.print("Motion test...");
  return failed;
}