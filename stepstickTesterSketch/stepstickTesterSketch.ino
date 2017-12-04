#include "stepper.h"

#define STEPPER_STEP  2
#define STEPPER_EN    3
#define STEPPER_DIR   4
#define STEPPER_MS1   5
#define STEPPER_MS2   6
#define STEPPER_MS3   7

#define EN_12V        A0
#define EN_5V         A1
#define TEST_5V       A2
#define TEST_12V      A3

#define TEST_BUTTON   8

#define LCD_RST       9
#define LCD_CS        10
#define LCD_DATA      11
#define LCD_DC        12
#define LCD_CLK       13


#define DIV_5V_R1 4700
#define DIV_5V_R2 1000
#define RAIL_5V_THRESHOLD 4.5

#define DIV_12V_R1 4700
#define DIV_12V_R2 1000
#define RAIL_12V_THRESHOLD 10

#define RAIL_SHORT_WAIT_TIME 250

#define HAVE_ROTATIONAL_ENCODER false    //if we don't have an encoder, the user must watch the motor and verify it behaves correctly

#define ROTATE_DEGREES 90   //we rotate back or forwards 90 degrees
#define ANGULAR_TOLERANCE 5 //number of degrees forward/reverse test can be off by
#define MOTOR_REST_TIME 100
#define MOTOR_TEST_SPEED 2

Stepper stepper(STEPPER_STEP, STEPPER_DIR, STEPPER_EN, STEPPER_MS1, STEPPER_MS2, STEPPER_MS3, TYPE_DRV8825);

void setup() {

  Serial.begin(250000);

  //initialise pins
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_EN,   OUTPUT);
  pinMode(STEPPER_DIR,  OUTPUT);
  pinMode(STEPPER_MS1,  OUTPUT);
  pinMode(STEPPER_MS2,  OUTPUT);
  pinMode(STEPPER_MS3,  OUTPUT);  
  
  pinMode(EN_12V, OUTPUT);
  pinMode(EN_5V,  OUTPUT);  
  digitalWrite(EN_12V, HIGH);
  digitalWrite(EN_5V,  HIGH);
  
  pinMode(TEST_12V, INPUT);
  pinMode(TEST_5V,  INPUT);

  pinMode(TEST_BUTTON, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  runTest();
  delay(10000);
}

bool runTest() {
  bool failed = false;

  unsigned long testStartTime = millis();

  if(!failed) {
    Serial.print("Enabling 5V line... ");
    digitalWrite(EN_5V,  LOW);
    delay(RAIL_SHORT_WAIT_TIME);
    float rail_5V = 5;//dividerVoltage(analogRead(TEST_5V), DIV_5V_R1, DIV_5V_R2);
  
    if(rail_5V < RAIL_5V_THRESHOLD) {
      failed = true;
      Serial.println("5V fail!");
    }  else {
      Serial.println("Success!");
    }
  }

  if(!failed) {
    Serial.print("Enabling 12V line... ");
    digitalWrite(EN_12V,  LOW);
    delay(RAIL_SHORT_WAIT_TIME);
    float rail_12V = 12;//dividerVoltage(analogRead(TEST_12V), DIV_12V_R1, DIV_12V_R2);
  
    if(rail_12V < RAIL_12V_THRESHOLD) {
      failed = true;
      Serial.println("12V fail!");
    }  else {
      Serial.println("Success!");
    }
  }

  if(!failed) {

    Serial.print("Driver has ");
    Serial.print(stepper.getMicrosteppingModes());
    Serial.println(" microstepping modes.");
  
    for(int i = 0; i < stepper.getMicrosteppingModes(); i++) {
      int startPosition = readEncoderAngle(); 
      int angularDiff;
      
      if(!failed) {
        stepper.setMicrosteppingMode(i);   
        stepper.setMotorSpeed(MOTOR_TEST_SPEED); 
        Serial.print("Testing microstepping mode ");
        Serial.print(i);
        Serial.print(" (");
        Serial.print(stepper.getMicrosteppingMultiplier(i));
        Serial.println("x microstepping)");        
      }
      
      /////////////////////////////////////////////////////////////////////
      // Rotate by X degrees, tests MS settings and STEP pin.
      /////////////////////////////////////////////////////////////////////
      if(!failed) {
        Serial.print("Forward rotate test... ");
        
        stepper.setDirectionForward(true);
        stepper.enableMotor(true);                
        stepper.moveMotor((float)ROTATE_DEGREES / 360.0f);   //move forward by X degrees

        delay(MOTOR_REST_TIME);

        if(HAVE_ROTATIONAL_ENCODER) {
          int forwardPosition = readEncoderAngle();
          angularDiff = abs(angularDifference(startPosition, forwardPosition));
          
          if(abs(angularDiff - ROTATE_DEGREES) > ANGULAR_TOLERANCE) {
            failed = true;
            Serial.println("failed!");
          } else {
            Serial.println("passed!");
          }
        } else {
          Serial.println("done.");
        }
      }
  
      /////////////////////////////////////////////////////////////////////
      // Reverse direction, rotate back by X degrees. Tests DIR pin.
      /////////////////////////////////////////////////////////////////////
      if(!failed) {
        Serial.print("Reverse rotate test... ");
        stepper.setDirectionForward(false);
        stepper.moveMotor((float)ROTATE_DEGREES / 360.0f);   //move back by X degrees

        delay(MOTOR_REST_TIME);

        if(HAVE_ROTATIONAL_ENCODER) {
          int returnedPosition = readEncoderAngle();
          angularDiff = abs(angularDifference(startPosition, returnedPosition));
          
          if(abs(angularDiff) > ANGULAR_TOLERANCE) {
            failed = true;
            Serial.println("failed!");
          } else {
            Serial.println("passed!");
          }
        } else {
          Serial.println("done.");
        }
      }
      
      /////////////////////////////////////////////////////////////////////
      // Disable driver, attempt rotate by X degrees. Tests EN pin.
      /////////////////////////////////////////////////////////////////////
      if(!failed) {
        Serial.print("Disabled rotate test... ");
        stepper.enableMotor(false);
        stepper.setDirectionForward(true);
        stepper.moveMotor((float)ROTATE_DEGREES / 360.0f);   //move forward by X degrees

        delay(MOTOR_REST_TIME);

        if(HAVE_ROTATIONAL_ENCODER) {
          int disabledPosition = readEncoderAngle();
          angularDiff = abs(angularDifference(startPosition, disabledPosition));
      
          if(abs(angularDiff) > ANGULAR_TOLERANCE) {
            failed = true;
            Serial.println("failed!");
          } else {
            Serial.println("passed!");
          }  
        } else {
          Serial.println("done.");
        }
      }
    }
  }

  /////////////////////////////////////////////////////////////////////
  // Test is completed. Disable power to stepper driver.
  /////////////////////////////////////////////////////////////////////
  digitalWrite(EN_5V,  HIGH);
  digitalWrite(EN_12V, HIGH);

  unsigned long testEndTime = millis();
  unsigned long testDuration = testEndTime - testStartTime;
  Serial.print("Test took ");
  Serial.print(testDuration);
  Serial.println("ms to complete.");

  return !failed;
  
}


float dividerVoltage(int analogReading, int R1, int R2) {
  float voltage = ((5 * (analogReading / 1024)) * (R1 + R2)) / (R2);
  return voltage;
}

int angularDifference(int angleA, int angleB) {
  return 0;
}

int readEncoderAngle() {
  return 0;
}

