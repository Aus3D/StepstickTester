#include "Arduino.h"
#include "stepper.h"

driverType drv8825 = {
	.microsteps = {
	{0, 0, 0, 1},
	{1, 0, 0, 2},
	{0, 1, 0, 4},
	{1, 1, 0, 8},
	{0, 0, 1, 16},
	{1, 0, 1, 32},
	{0, 1, 1, 32},
	{1, 1, 1, 32}},
	.microsteppingModes = 6,
	.enable_inverted = true,
	.dir_inverted = false
};

driverType a4988 = {
	.microsteps = {
	{0, 0, 0, 1},
	{1, 0, 0, 2},
	{0, 1, 0, 4},
	{1, 1, 0, 8},
	{1, 1, 1, 16}},
	.microsteppingModes = 5,
	.enable_inverted = true,
	.dir_inverted = false
};




Stepper::Stepper(int STEP_PIN, int DIR_PIN, int EN_PIN, int MS1_PIN, int MS2_PIN, int MS3_PIN, int type)
{
	_step_pin = STEP_PIN;
	_dir_pin = DIR_PIN;
	_en_pin = EN_PIN;
	_ms1_pin = MS1_PIN;
	_ms2_pin = MS2_PIN;
	_ms3_pin = MS3_PIN;
	_microsteppingMode = 0;

	switch(type) {
		case TYPE_A4988: 
			_driver = a4988;
			break;
		case TYPE_DRV8825:
			_driver = drv8825;
			break;
	}

}

void Stepper::enableMotor(bool enabled) {
	digitalWrite(_en_pin, _driver.enable_inverted? !enabled : enabled);
}

void Stepper::setDirectionForward(bool forward) {
	digitalWrite(_dir_pin, _driver.dir_inverted? !forward : forward);
}

int Stepper::getMicrosteppingMode() {
	return _microsteppingMode;
}

int Stepper::getMicrosteppingModes() {
	return _driver.microsteppingModes;
}

void Stepper::setMicrosteppingMode(int mode) {
	if(mode >= 0 && mode < getMicrosteppingModes()) {
		_microsteppingMode = mode;

		digitalWrite(_ms1_pin, _driver.microsteps[mode].ms1_state);
		digitalWrite(_ms2_pin, _driver.microsteps[mode].ms2_state);
		digitalWrite(_ms3_pin, _driver.microsteps[mode].ms3_state);
	}
}

int Stepper::getMicrosteppingMultiplier(int mode) {
	return _driver.microsteps[mode].step_multiplier;
}

void Stepper::moveMotor(float rotations) {
	int stepsToMove = _stepsPerRevolution * rotations * getMicrosteppingMultiplier(getMicrosteppingMode());

    for(int i = 0; i < stepsToMove; i++) {
      digitalWrite(_step_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(_step_pin, LOW);
      delayMicroseconds(1000000 / (_speed * _stepsPerRevolution * getMicrosteppingMultiplier(getMicrosteppingMode())));
    }    
}

//speed is revolutions / second
void Stepper::setMotorSpeed(float speed) {
	_speed = speed;
}