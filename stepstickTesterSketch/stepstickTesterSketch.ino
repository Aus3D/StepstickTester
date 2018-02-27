#include "config.h"
#include "stepper.h"
#include <SPI.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "AMS_5600.h"

SSD1306AsciiWire display;
Stepper stepper(STEPPER_STEP, STEPPER_DIR, STEPPER_EN, STEPPER_MS1, STEPPER_MS2, STEPPER_MS3, DRIVER_TYPE_DRV8825);
AMS_5600 ams5600;

typedef struct {
  char PROGMEM menuItemText[18];
  int *setVarPtr;
  int  setVarDef;
  int  setVarMin;
  int  setVarMax;
  char * (*returnFunc)(int);
} menuItem;

int testSpeedVar;
int testRangeVar;
int testStartVar;
int driverTypeVar;

char * menuItemValToChar(int val);


menuItem testDriver = {"Driver Type:   ",&driverTypeVar,0,0,(DRIVER_TYPE_COUNT-1),&(stepper.getDriverTypeName)};
menuItem testSpeed  = {"Motor Speed:   ",&testSpeedVar,MOTOR_TEST_SPEED,1,10,&(menuItemValToChar)};
menuItem testStart  = {"Begin Test",&testStartVar,0,0,0,&(menuItemValToChar)};


typedef struct {
  menuItem menuItems[5];
  int menuItemNum;
} menu;

menu optionsMenu = {{testDriver, testSpeed, testStart}, 3};

void setup() {

  //Initialise I2C for display and AS5600 encoder
  Wire.begin();
  Wire.setClock(400000L);

  //Initialise display
  display.begin(&Adafruit128x64, 0x3D, LCD_RST);  // initialize with the I2C addr 0x3D (for the 128x64) 
  display.setFont(Adafruit5x7);
  display.setScroll(true);
  display.clear();

  //Initialise serial
  Serial.begin(250000);
  Serial.println(F("Beginning stepper test sketch..."));
  Serial.print(F("Driver Type: "));
  Serial.println(stepper.getDriverTypeName(stepper.getDriverType()));

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

  configurationScreen();
  stepper.setDriverType(driverTypeVar);
}

void loop() {
  runTest();
  delay(500);
  while(digitalRead(TEST_BUTTON) == HIGH) {}
}

bool runTest() {
  bool failed = false;

  display.clear();
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
  }
  
  if(!failed) {
    Serial.print(F("Enabling 12V line... "));
    display.print(F("12V line... "));
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
        break;
      } else {
        alignCursorRight(4);
        display.println(F("pass"));
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
  } else {
    digitalWrite(LED_GREEN, HIGH);
    tone(BUZZER, BUZZER_PASSED_FREQ, BUZZER_PASSFAIL_DUR);
    display.println(F("Test passed!"));
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
  display.setCursor(display.displayWidth() - characters*6, display.row());
}

void configurationScreen() {
  int menuItemNum = 0;
  int menuItemVal = optionsMenu.menuItems[menuItemNum].setVarDef; 
  int menuItemValPrev = 0;
  bool finished = false;
  bool buttonPressed = false;
  bool displayChanged = true;

  //initialise menu options to their defaults
  for(int i = 0; i < optionsMenu.menuItemNum; i++) {
    *optionsMenu.menuItems[i].setVarPtr = optionsMenu.menuItems[i].setVarDef;
  }

  while(!finished) {
    delay(50);
    static int oldAngle = readEncoderAngle();
    int newAngle = readEncoderAngle();
    int angDiff = angularDifference(oldAngle, newAngle);

    /*
    Serial.print(F("old angle: \t"));
    Serial.print(oldAngle);
    Serial.print(F("\tnew angle: \t"));
    Serial.print(newAngle);
    Serial.print(F("\tdiff: \t"));
    Serial.print(angDiff);
    Serial.print(F("\tmenu num: \t"));
    Serial.print(menuItemNum);
    Serial.print(F("\tmenu val: \t"));
    Serial.print(menuItemVal);
    Serial.print(F("\titem val: \t"));
    Serial.print(*optionsMenu.menuItems[menuItemNum].setVarPtr);
    Serial.println();
    */

    //Handle input from encoder
    if(optionsMenu.menuItems[menuItemNum].setVarMax != optionsMenu.menuItems[menuItemNum].setVarMin) {
      if(angDiff > 90) {
        menuItemVal++;
        tone(BUZZER, BUZZER_TICK_FREQ, BUZZER_TICK_DUR);
        oldAngle = newAngle;
        displayChanged = true;
      }
    
      if(angDiff < -90) {
        menuItemVal--;
        tone(BUZZER, BUZZER_TICK_FREQ, BUZZER_TICK_DUR);
        oldAngle = newAngle;
        displayChanged = true;
      }
    }

    //Handle button input
    if(digitalRead(TEST_BUTTON) == LOW) {
      delay(10);
      if(digitalRead(TEST_BUTTON) == LOW && buttonPressed == false) {
        if(menuItemNum < optionsMenu.menuItemNum-1) {
          menuItemNum++;
          menuItemVal=optionsMenu.menuItems[menuItemNum].setVarDef; 
          buttonPressed = true;
        } else {
          finished = true;
        }
        displayChanged = true;
      }
    } else {
      buttonPressed = false;
    }

    //Update value if changed
    if(menuItemValPrev != menuItemVal) {
      while(menuItemVal > optionsMenu.menuItems[menuItemNum].setVarMax) {
        menuItemVal = optionsMenu.menuItems[menuItemNum].setVarMin;
      }
      
      while(menuItemVal < optionsMenu.menuItems[menuItemNum].setVarMin) {
        menuItemVal = optionsMenu.menuItems[menuItemNum].setVarMax;
      }
      
      *optionsMenu.menuItems[menuItemNum].setVarPtr = menuItemVal; 
      menuItemValPrev = menuItemVal;
    }

    //draw any updates to display
    if(displayChanged) {
      display.setCursor(0,0);
      display.println("Configuration");
    
      for(int i = 0; i < optionsMenu.menuItemNum; i++) {
        if(i == menuItemNum) {
          display.print(">");
        }
  
        char valBuffer[10];
        strcpy(valBuffer, (*optionsMenu.menuItems[i].returnFunc)(*optionsMenu.menuItems[i].setVarPtr));
  
        display.print(optionsMenu.menuItems[i].menuItemText);
        if(optionsMenu.menuItems[i].setVarMax != optionsMenu.menuItems[i].setVarMin) {
          display.clearToEOL();
          alignCursorRight(strlen(valBuffer));
          display.print(valBuffer);
        }
        display.println();
      }
      displayChanged = false;
    }
  }
}

char * menuItemValToChar(int val) {
  char charBuffer[10];
  itoa(val,charBuffer,10);
  return charBuffer;
}
