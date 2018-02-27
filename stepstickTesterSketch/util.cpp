#include "util.h"
#include "Arduino.h"
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "AMS_5600.h"

extern SSD1306AsciiWire display;
extern AMS_5600 ams5600;

char * menuItemValToChar(int val) {
  char charBuffer[10];
  itoa(val,charBuffer,10);
  return charBuffer;
}

float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */    
  float retVal = newAngle * 0.087;
  return retVal;
}

void alignCursorRight(int characters) {
  display.setCursor(display.displayWidth() - characters*6, display.row());
}

float dividerVoltage(int analogReading, int R1, int R2) {
  float voltage = (5.0 * ((float)analogReading / 1024)) * (R1 + R2) / (R2);
  return voltage;
}

int angularDifference(int angleA, int angleB) {
  int difference = angleB - angleA;
  while (difference < -180) difference += 360;
  while (difference > 180) difference -= 360;
  return difference;
}

int readEncoderAngle() {
  int angle = (int)round(convertRawAngleToDegrees(ams5600.getRawAngle()));
  #ifdef ENCODER_INVERT
    return 360-angle;
  #else
    return angle;
  #endif
}
