#ifndef STEPPER_H
#define STEPPER_H
#include "Arduino.h"
#include "tests.h"

#define DRIVER_TYPE_A4988		0
#define DRIVER_TYPE_DRV8825	1
#define DRIVER_TYPE_COUNT   2

typedef bool (* GenericFP)(void);

typedef struct {
	byte pin_states;
	byte step_multiplier; 
} microsteppingMode;

typedef struct {
  GenericFP functions[10];
  int functionCount;
} testProcedure;

typedef struct {
	microsteppingMode microsteps[8];
	byte microsteppingModes;
	bool enable_inverted;
	bool dir_inverted;
  testProcedure test;
  char driver_name[8];
} driverType;

typedef struct {
  driverType drivers[DRIVER_TYPE_COUNT];
  byte driverCount;
} driverList;


class Stepper
{
public:
	Stepper(int STEP_PIN, int DIR_PIN, int EN_PIN, int MS1_PIN, int MS2_PIN, int MS3_PIN, int type);
	void moveMotor(float rotations);
	void setMicrosteppingMode(int mode);
	int getMicrosteppingMode();
	int getMicrosteppingModes();
	int getMicrosteppingMultiplier(int mode);
	void enableMotor(bool enabled);
	void setDirectionForward(bool forward);
	void setMotorSpeed(float speed);
	void setDriverType(int type);
	static char * getDriverTypeName(int type);
	int getDriverType();
	driverType getDriver();

private:
	driverType _driver;

	byte _microsteppingMode;
	byte _step_pin;
	byte _dir_pin;
	byte _en_pin;
	byte _ms1_pin;
	byte _ms2_pin;
	byte _ms3_pin;

	int _stepsPerRevolution = 200;
	float _speed = 1;
  int _driver_type_num = 0;

};


#endif
