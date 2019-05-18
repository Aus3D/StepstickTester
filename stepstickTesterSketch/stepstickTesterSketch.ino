#include "config.h"
#include "stepper.h"
#include "tests.h"
#include "util.h"
#include <SPI.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "AMS_5600.h"

SSD1306AsciiWire display;
Stepper stepper(STEPPER_STEP, STEPPER_DIR, STEPPER_EN, STEPPER_MS1, STEPPER_MS2, STEPPER_MS3, 0);
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
  int testStatus = TEST_PASSED;

  display.clear();
  display.setCursor(0,0);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  unsigned long testStartTime = millis();

  for(int i = 0; i < stepper.getDriver().test.functionCount; i++) {
    int newStatus = (*stepper.getDriver().test.functions[i])(testStatus);

    if(newStatus == TEST_FAILED) {
      testStatus = TEST_FAILED;
      alignCursorRight(4);
      display.println(F("fail"));
      Serial.println(F("fail"));
    }

    if(newStatus == TEST_PASSED) {
      alignCursorRight(4);
      display.println(F("pass"));
      Serial.println(F("pass"));
    }
  }

  digitalWrite(POWER_MOTOR_LOGIC_EN,  HIGH);
  digitalWrite(POWER_MOTOR_SUPPLY_EN, HIGH);
  stepper.enableMotor(false);

  if(testStatus == TEST_FAILED) {
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

  if(testStatus == TEST_PASSED) {
    return true;
  } else {
    return false;
  }
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