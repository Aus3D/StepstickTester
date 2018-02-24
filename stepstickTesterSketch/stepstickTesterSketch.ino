#include "stepper.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AMS_5600.h"

#define STEPPER_STEP  2
#define STEPPER_EN    3
#define STEPPER_DIR   4
#define STEPPER_MS1   5
#define STEPPER_MS2   6
#define STEPPER_MS3   7

#define POWER_MOTOR_SUPPLY_EN     13
#define POWER_MOTOR_SUPPLY_TEST   A3
#define POWER_MOTOR_LOGIC_EN      A0
#define POWER_MOTOR_LOGIC_TEST    A2
#define POWER_BOARD_INPUT_TEST    A1

#define TEST_BUTTON   8
#define LED_GREEN     11
#define LED_RED       12
#define BUZZER        9

#define BUZZER_PASSED_FREQ  440
#define BUZZER_FAILED_FREQ  98
#define BUZZER_PASSFAIL_DUR 500
#define BUZZER_TICK_FREQ    300
#define BUZZER_TICK_DUR     10

#define LCD_RST       10
Adafruit_SSD1306 display(LCD_RST);

#define INPUT_RAIL_DIV_R1 4700
#define INPUT_RAIL_DIV_R2 1000
#define INPUT_RAIL_THRESHOLD 6    //minimum voltage input power must be to pass test

#define LOGIC_RAIL_DIV_R1 4700
#define LOGIC_RAIL_DIV_R2 1000
#define LOGIC_RAIL_THRESHOLD 4.5  //minimum voltage stepper driver logic rail can be to pass test

#define SUPPLY_RAIL_DIV_R1 4700
#define SUPPLY_RAIL_DIV_R2 1000
#define SUPPLY_RAIL_THRESHOLD 1    //minimum amount below input rail stepper supply rail can be to pass test

#define RAIL_SHORT_WAIT_TIME 250

#define HAVE_ROTATIONAL_ENCODER true    //if we don't have an encoder, the user must watch the motor and verify it behaves correctly

#define MOTOR_TEST_ROTATE_DEGREES     90  //we rotate back or forwards 90 degrees
#define MOTOR_TEST_ANGULAR_TOLERANCE  5   //number of degrees forward/reverse test can be off by
#define MOTOR_TEST_REST_TIME          100 //time (ms) to wait between test motions
#define MOTOR_TEST_SPEED              6   //motor speed (higher is faster)

Stepper stepper(STEPPER_STEP, STEPPER_DIR, STEPPER_EN, STEPPER_MS1, STEPPER_MS2, STEPPER_MS3, DRIVER_TYPE_DRV8825);
AMS_5600 ams5600;

void setup() {

  //Initialise I2C for display and AS5600 encoder
  Wire.begin();

  //Initialise display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64) 
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setTextWrap(false);
  display.clearDisplay();
  display.display();

  //Initialise serial
  Serial.begin(250000);
  Serial.println(F("Beginning stepper test sketch..."));
  Serial.print(F("Driver Type: "));
  Serial.println(stepper.getDriverTypeName());

  //Stepper pin initialisation
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_EN,   OUTPUT);
  pinMode(STEPPER_DIR,  OUTPUT);
  pinMode(STEPPER_MS1,  OUTPUT);
  pinMode(STEPPER_MS2,  OUTPUT);
  pinMode(STEPPER_MS3,  OUTPUT);  
  stepper.enableMotor(false);

  //Power rail pin initialisation
  pinMode(POWER_MOTOR_SUPPLY_EN,      OUTPUT);
  pinMode(POWER_MOTOR_LOGIC_EN,       OUTPUT); 
  pinMode(POWER_BOARD_INPUT_TEST,     INPUT);
  pinMode(POWER_MOTOR_SUPPLY_TEST,    INPUT);
  pinMode(POWER_MOTOR_LOGIC_TEST,     INPUT);
  digitalWrite(POWER_MOTOR_SUPPLY_EN, HIGH);
  digitalWrite(POWER_MOTOR_LOGIC_EN,  HIGH);

  //Button and LEDs initialisation
  pinMode(TEST_BUTTON,  INPUT_PULLUP);
  pinMode(LED_GREEN,    OUTPUT);
  pinMode(LED_RED,      OUTPUT);
  pinMode(BUZZER,       OUTPUT);

  //Indicate setup complete
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  tone(BUZZER, BUZZER_PASSED_FREQ, BUZZER_PASSFAIL_DUR);
  delay(1000);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  Serial.println(F("End of Setup"));

  while(digitalRead(TEST_BUTTON) == HIGH) {
    //Print startup screen
    configurationScreen();
  } 
}

void loop() {

  runTest();
  while(digitalRead(TEST_BUTTON) == HIGH) {}

}

bool runTest() {
  bool failed = false;

  display.clearDisplay();
  display.setCursor(0,0);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  unsigned long testStartTime = millis();
  float rail_input;

  if(!failed) {
    rail_input = dividerVoltage(analogRead(POWER_BOARD_INPUT_TEST), INPUT_RAIL_DIV_R1, INPUT_RAIL_DIV_R2);
    Serial.print(F("Testing input voltage... "));
    Serial.print(rail_input);
    Serial.print(F("V... "));

    display.print(F("Input power...  "));
    display.display();

    if(rail_input < INPUT_RAIL_THRESHOLD) {
      failed = true;
      alignCursorRight(4);
      display.println(F("fail"));
      Serial.println(F("fail"));
    } else {
      alignCursorRight(4);
      display.println(F("pass"));
      Serial.println(F("pass"));
    }
  }

  if(!failed) {
    Serial.print(F("Enabling 5V line... "));
    display.print(F("5V line...  "));
    display.display();
    digitalWrite(POWER_MOTOR_LOGIC_EN,  LOW);
    delay(RAIL_SHORT_WAIT_TIME);   
    float rail_5V = dividerVoltage(analogRead(POWER_MOTOR_LOGIC_TEST), LOGIC_RAIL_DIV_R1, LOGIC_RAIL_DIV_R2);
  
    if(rail_5V < LOGIC_RAIL_THRESHOLD) {
      failed = true;
      alignCursorRight(4);
      display.println(F("fail"));
      Serial.println(F("5V fail!"));
    }  else {
      alignCursorRight(4);
      display.println(F("pass"));
      Serial.println(F("pass"));
    }
    display.display();
  }
  
  if(!failed) {
    Serial.print(F("Enabling 12V line... "));
    display.print(F("12V line... "));
    display.display();
    digitalWrite(POWER_MOTOR_SUPPLY_EN,  LOW); 
    delay(RAIL_SHORT_WAIT_TIME);   
    float rail_12V = dividerVoltage(analogRead(POWER_MOTOR_SUPPLY_TEST), SUPPLY_RAIL_DIV_R1, SUPPLY_RAIL_DIV_R2);
        
    if(rail_12V < (rail_input - SUPPLY_RAIL_THRESHOLD)) {
      failed = true;
      alignCursorRight(4);
      display.println(F("fail"));
      Serial.println(F("12V fail!"));
    }  else {
      alignCursorRight(4);
      display.println(F("pass"));
      Serial.println(F("pass"));
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
        display.display();    
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
          angularDiff = angularDifference(startPosition, forwardPosition);

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
          angularDiff = angularDifference(startPosition, returnedPosition);

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
          angularDiff = angularDifference(startPosition, disabledPosition);

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
  digitalWrite(POWER_MOTOR_LOGIC_EN,  HIGH);
  digitalWrite(POWER_MOTOR_SUPPLY_EN, HIGH);
  stepper.enableMotor(false);

  if(failed) {
    digitalWrite(LED_RED, HIGH);
    tone(BUZZER, BUZZER_FAILED_FREQ, BUZZER_PASSFAIL_DUR);
    display.println(F("Test failed!"));
    display.display();
  } else {
    digitalWrite(LED_GREEN, HIGH);
    tone(BUZZER, BUZZER_PASSED_FREQ, BUZZER_PASSFAIL_DUR);
    display.println(F("Test passed!"));
    display.display();
  }

  unsigned long testEndTime = millis();
  unsigned long testDuration = testEndTime - testStartTime;
  Serial.print(F("Test took "));
  Serial.print(testDuration);
  Serial.println(F("ms to complete."));

  return !failed;
}

float dividerVoltage(int analogReading, int R1, int R2) {
  float voltage = (5.0 * ((float)analogReading / 1024)) * (R1 + R2) / (R2);
  return voltage;
}

int angularDifference(int angleA, int angleB) {
  int difference = angleA - angleB;
  while (difference < -180) difference += 360;
  while (difference > 180) difference -= 360;
  return difference;
}

int readEncoderAngle() {
  return (int)round(convertRawAngleToDegrees(ams5600.getRawAngle()));
}

float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */    
  float retVal = newAngle * 0.087;
  return retVal;
}

void alignCursorRight(int characters) {
  display.setCursor(display.width()-characters*6,display.getCursorY());
}

void configurationScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Configuration:"));
  display.print(F("Driver Type: "));
  display.println(stepper.getDriverTypeName());
  display.display();

  static int oldAngle = readEncoderAngle();
  int newAngle = readEncoderAngle();
  int angDiff = angularDifference(oldAngle, newAngle);

  //Serial.print(F("old angle: \t"));
  //Serial.print(oldAngle);
  //Serial.print(F("\tnew angle: \t"));
  //Serial.print(newAngle);
  //Serial.print(F("\tdiff: \t"));
  //Serial.print(angDiff);

  if(angDiff > 90) {
    stepper.setDriverType(stepper.getDriverType() + 1);
    if(stepper.getDriverType() >= DRIVER_TYPE_COUNT) {
      stepper.setDriverType(0);
    }
    //Serial.print(F(" increment"));
    tone(BUZZER, BUZZER_TICK_FREQ, BUZZER_TICK_DUR);
    oldAngle = newAngle;
  }

  if(angDiff < -90) {
    stepper.setDriverType(stepper.getDriverType() - 1);
    if(stepper.getDriverType() < 0) {
      stepper.setDriverType(DRIVER_TYPE_COUNT-1);
    }
    //Serial.print(F("decrement"));
    tone(BUZZER, BUZZER_TICK_FREQ, BUZZER_TICK_DUR);
    oldAngle = newAngle;
  }

  //Serial.print("\t");
  //Serial.print(stepper.getDriverType());
  //Serial.print("\t");
  //Serial.print(stepper.getDriverTypeName());
  //Serial.println();
}

