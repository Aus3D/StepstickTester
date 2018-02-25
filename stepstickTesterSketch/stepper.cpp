#include "Arduino.h"
#include "stepper.h"

const driverType drv8825 = {
	.microsteps = {
	{0b000, 1},
	{0b001, 2},
	{0b010, 4},
	{0b011, 8},
	{0b100, 16},
	{0b101, 32},
	{0b110, 32},
	{0b111, 32}},
	.microsteppingModes = 6,
	.enable_inverted = true,
	.dir_inverted = false,
  "DRV8825"
};

const driverType a4988 = {
	.microsteps = {
	{0b000, 1},
	{0b001, 2},
	{0b010, 4},
	{0b011, 8},
	{0b111, 16}},
	.microsteppingModes = 5,
	.enable_inverted = true,
	.dir_inverted = false,
  "A4988"
};

const driverList drivers = {{a4988, drv8825}, DRIVER_TYPE_COUNT};

Stepper::Stepper(int STEP_PIN, int DIR_PIN, int EN_PIN, int MS1_PIN, int MS2_PIN, int MS3_PIN, int type)
{
	_step_pin = STEP_PIN;
	_dir_pin = DIR_PIN;
	_en_pin = EN_PIN;
	_ms1_pin = MS1_PIN;
	_ms2_pin = MS2_PIN;
	_ms3_pin = MS3_PIN;
	_microsteppingMode = 0;
  setDriverType(type);
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

		digitalWrite(_ms1_pin, bitRead(_driver.microsteps[mode].pin_states,0));
		digitalWrite(_ms2_pin, bitRead(_driver.microsteps[mode].pin_states,1));
		digitalWrite(_ms3_pin, bitRead(_driver.microsteps[mode].pin_states,2));
	}
}

int Stepper::getMicrosteppingMultiplier(int mode) {
	return _driver.microsteps[mode].step_multiplier;
}

void Stepper::moveMotor(float rotations) {
	int stepsToMove = _stepsPerRevolution * rotations * getMicrosteppingMultiplier(getMicrosteppingMode());
  int stepDelay   = 1000000 / (_speed * _stepsPerRevolution * getMicrosteppingMultiplier(getMicrosteppingMode()));

    for(int i = 0; i < stepsToMove; i++) {
      digitalWrite(_step_pin, HIGH);
      delayMicroseconds(2);
      digitalWrite(_step_pin, LOW);
      delayMicroseconds(stepDelay);
    }    
}

//speed is revolutions / second
void Stepper::setMotorSpeed(float speed) {
	_speed = speed;
}

void Stepper::setDriverType(int type) {
  _driver_type_num = type;
  _driver = drivers.drivers[_driver_type_num];
}

int Stepper::getDriverType() {
  return _driver_type_num;
}

char * Stepper::getDriverTypeName() {
  return _driver.driver_name;;
}

