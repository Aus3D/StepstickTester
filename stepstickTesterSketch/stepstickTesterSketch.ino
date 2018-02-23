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
#define TICK_BUZZER_FREQ    1000

#define LCD_RST       10
Adafruit_SSD1306 display(LCD_RST);

#define DIV_5V_R1 4700
#define DIV_5V_R2 1000
#define RAIL_5V_THRESHOLD 4.5

#define DIV_12V_R1 4700
#define DIV_12V_R2 1000
#define RAIL_12V_THRESHOLD 10
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
  pinMode(EN_12V,       OUTPUT);
  pinMode(EN_5V,        OUTPUT);  
  pinMode(TEST_12V,     INPUT);
  pinMode(TEST_5V,      INPUT);
  digitalWrite(EN_12V,  HIGH);
  digitalWrite(EN_5V,   HIGH);

  //Button and LEDs initialisation
  pinMode(TEST_BUTTON,  INPUT_PULLUP);
  pinMode(LED_GREEN,    OUTPUT);
  pinMode(LED_RED,      OUTPUT);
  pinMode(BUZZER,       OUTPUT);

  //Indicate setup complete
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  tone(BUZZER, PASSED_BUZZER_FREQ, 500);
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

  if(!failed) {
    Serial.print(F("Enabling 5V line... "));
    display.print(F("5V line...  "));
    display.display();
    digitalWrite(EN_5V,  LOW);
    delay(RAIL_SHORT_WAIT_TIME);   
    float rail_5V = dividerVoltage(analogRead(TEST_5V), DIV_5V_R1, DIV_5V_R2);
  
    if(rail_5V < RAIL_5V_THRESHOLD) {
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
    digitalWrite(EN_12V,  LOW); 
    delay(RAIL_SHORT_WAIT_TIME);   
    float rail_12V = dividerVoltage(analogRead(TEST_12V), DIV_12V_R1, DIV_12V_R2);
        
    if(rail_12V < RAIL_12V_THRESHOLD) {
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
  stepper.enableMotor(false);

  if(failed) {
    digitalWrite(LED_RED, HIGH);
    tone(BUZZER, FAILED_BUZZER_FREQ, 500);
    display.println(F("Test failed!"));
    display.display();
  } else {
    digitalWrite(LED_GREEN, HIGH);
    tone(BUZZER, PASSED_BUZZER_FREQ, 500);
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
  return 180 - abs(abs(angleA - angleB) - 180);
}

int readEncoderAngle() {
  return (int)round(convertRawAngleToDegrees(ams5600.getRawAngle()));
}

/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates 
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle)
{
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
    tone(BUZZER, TICK_BUZZER_FREQ, 50);
    oldAngle = newAngle;
  }

  if(angDiff < -90) {
    stepper.setDriverType(stepper.getDriverType() - 1);
    if(stepper.getDriverType() == 0) {
      stepper.setDriverType(DRIVER_TYPE_COUNT-1);
    }
    //Serial.print(F("decrement"));
    tone(BUZZER, TICK_BUZZER_FREQ, 10);
    oldAngle = newAngle;
  }

  //Serial.print("\t");
  //Serial.print(stepper.getDriverType());
  //Serial.print("\t");
  //Serial.print(stepper.getDriverTypeName());
  //Serial.println();
}

