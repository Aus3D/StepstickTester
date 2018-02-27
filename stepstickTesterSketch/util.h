#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

char * menuItemValToChar(int val);
float convertRawAngleToDegrees(word newAngle);
void alignCursorRight(int characters);
float dividerVoltage(int analogReading, int R1, int R2);
int angularDifference(int angleA, int angleB);
int readEncoderAngle();

#endif //UTIL_H