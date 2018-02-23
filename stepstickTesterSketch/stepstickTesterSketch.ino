#include "stepper.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define STEPPER_STEP  2
#define STEPPER_EN    3
#define STEPPER_DIR   4
#define STEPPER_MS1   5
#define STEPPER_MS2   6
#define STEPPER_MS3   7

#define EN_12V        13
#define EN_5V         A0
#define TEST_5V       A2
#define TEST_12V      A3

#define TEST_BUTTON   8
#define LED_GREEN     11
#define LED_RED       12
#define BUZZER        9

#define PASSED_BUZZER_FREQ  440
#define FAILED_BUZZER_FREQ  98

#define LCD_RST       10
Adafruit_SSD1306 display(LCD_RST);

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
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64) 
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setTextWrap(false);
  display.setCursor(0,0);
  display.println(F("Configuration:"));
  display.display();

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

  pinMode(TEST_BUTTON, INPUT_PULLUP);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  tone(BUZZER, PASSED_BUZZER_FREQ, 500);

  delay(1000);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
}

void loop() {
  while(digitalRead(TEST_BUTTON) == HIGH) {} 
  runTest();
}

bool runTest() {
  bool failed = false;

  display.clearDisplay();
  display.setCursor(0,0);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  unsigned long testStartTime = millis();

  if(!failed) {
    Serial.print(F("Enabling 5V line... "));
    display.print(F("5V line...  "));
    display.display();
    digitalWrite(EN_5V,  LOW);
    delay(RAIL_SHORT_WAIT_TIME);
    float rail_5V = 5;//dividerVoltage(analogRead(TEST_5V), DIV_5V_R1, DIV_5V_R2);
  
    if(rail_5V < RAIL_5V_THRESHOLD) {
      failed = true;
      Serial.println(F("5V fail!"));
      alignCursorRight(4);
      display.println("fail");
      display.println(F("fail"));
    }  else {
      alignCursorRight(4);
      Serial.println(F("pass"));
      display.println(F("pass"));
    }
    display.display();
  }
  

  if(!failed) {
    Serial.print(F("Enabling 12V line... "));

    //float rail_12V = dividerVoltage(analogRead(TEST_12V), DIV_12V_R1, DIV_12V_R2);
    //Serial.println(rail_12V);

    display.print(F("12V line... "));
    display.display();
    //digitalWrite(EN_12V,  LOW);

    //rail_12V = dividerVoltage(analogRead(TEST_12V), DIV_12V_R1, DIV_12V_R2);
    //Serial.println(rail_12V);
    Serial.println(analogRead(TEST_12V));
        
    delay(RAIL_SHORT_WAIT_TIME);   
    
    float rail_12V = dividerVoltage(analogRead(TEST_12V), DIV_12V_R1, DIV_12V_R2);
    Serial.println(rail_12V);
        
    if(rail_12V < RAIL_12V_THRESHOLD) {
      failed = true;
      Serial.println(F("12V fail!"));
      alignCursorRight(4);
      display.println(F("fail"));
    }  else {
      Serial.println(F("pass"));
      alignCursorRight(4);
      display.println(F("pass"));
    }
    display.display();
  }
  
  if(!failed) {

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
        stepper.setMotorSpeed(MOTOR_TEST_SPEED); 
        Serial.print("Testing microstepping mode ");
        Serial.print(i);
        Serial.print(" (");
        Serial.print(stepper.getMicrosteppingMultiplier(i));
        Serial.println("x microstepping)");        
        display.print(F("MS mode "));
        display.print(i);
        display.print(" (");
        display.print(stepper.getMicrosteppingMultiplier(i));
        display.print("x)");
        display.display();    
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
      if(failed) {
        alignCursorRight(4);
        display.println(F("fail"));
        display.display();
        break;
      } else {
        alignCursorRight(4);
        display.println(F("pass"));
        display.display();
      }
    }
  }

  /////////////////////////////////////////////////////////////////////
  // Test is completed. Disable power to stepper driver.
  /////////////////////////////////////////////////////////////////////
  digitalWrite(EN_5V,  HIGH);
  digitalWrite(EN_12V, HIGH);

  if(failed) {
    digitalWrite(LED_RED, HIGH);
    tone(BUZZER, FAILED_BUZZER_FREQ, 500);
  } else {
    digitalWrite(LED_GREEN, HIGH);
    tone(BUZZER, PASSED_BUZZER_FREQ, 500);
    display.println(F("Test passed!"));
    display.display();
  }
 

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

void alignCursorRight(int characters) {
  display.setCursor(display.width()-characters*6,display.getCursorY());
}

