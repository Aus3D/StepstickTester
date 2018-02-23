#ifndef STEPPER_H
#define STEPPER_H
#include "Arduino.h"

#define TYPE_A4988		0
#define TYPE_DRV8825	1

typedef struct {
	bool ms1_state;
	bool ms2_state;
	bool ms3_state;
	int step_multiplier; 
} microsteppingMode;

typedef struct {
	microsteppingMode microsteps[8];
	int microsteppingModes;
	bool enable_inverted;
	bool dir_inverted;
} driverType;


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

private:
	driverType _driver;

	int _microsteppingMode;
	int _step_pin;
	int _dir_pin;
	int _en_pin;
	int _ms1_pin;
	int _ms2_pin;
	int _ms3_pin;

	int _stepsPerRevolution = 200;
	float _speed = 1;

};


#endif