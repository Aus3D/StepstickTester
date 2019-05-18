#include "stepper.h"
#include "Arduino.h"

testProcedure drv8825_test = {
  .functions = {
    testFunction_testInputVoltage,
    testFunction_test5V,
    testFunction_test12V,
    testFunction_testStepperMotion,
    testFunction_powerDown,
    testFunction_test5V_down
  },
  .functionCount = 6
};

testProcedure tmc2100_test = {
  .functions = {
    testFunction_testInputVoltage,
    testFunction_test5V,
    testFunction_test12V,
    testFunction_testStepperMotion,
    testFunction_powerDown
  },
  .functionCount = 5
};

const driverType drv8825 = {
	.microsteps = {
	{{0,0,0}, 1},
	{{0,0,1}, 2},
	{{0,1,0}, 4},
	{{0,1,1}, 8},
	{{1,0,0}, 16},
	{{1,0,1}, 32},
	{{1,1,0}, 32},
	{{1,1,1}, 32}},
	.microsteppingModes = 6,
	.enable_inverted = true,
	.dir_inverted = false,
  drv8825_test,
  "DRV8825"
};

const driverType a4988 = {
	.microsteps = {
	{{0,0,0}, 1},
	{{0,0,1}, 2},
	{{0,1,0}, 4},
	{{0,1,1}, 8},
	{{1,1,1}, 16}},
	.microsteppingModes = 5,
	.enable_inverted = true,
	.dir_inverted = false,
  drv8825_test,
  "A4988"
};

const driverType tmc2100 = {
  .microsteps = {
  {{0,0,0}, 16}},   //spreadCycle
  .microsteppingModes = 1,
  .enable_inverted = true,
  .dir_inverted = true,
  tmc2100_test,
  "TMC2100"
};


const driverType tmc2130 = {
	.microsteps = {
	{{0,0,0}, 1},		//spreadCycle
	{{0,0,1}, 2},		//spreadCycle
	{{0,0,2}, 2},		//spreadCycle (256)
	{{0,1,0}, 4},		//spreadCycle
	{{0,1,1}, 16},		//spreadCycle
	{{0,1,2}, 4},		//spreadCycle (256)
	{{0,2,0}, 16},		//spreadCycle (256)
	{{0,2,1}, 4},		//stealthChop (256)
	{{0,2,2}, 16}},		//stealthChop (256)
	.microsteppingModes = 9,
	.enable_inverted = true,
	.dir_inverted = false,
  tmc2100_test,
  "TMC2130"
};

const driverList drivers = {{a4988, drv8825, tmc2100, tmc2130}, DRIVER_TYPE_COUNT};

Stepper::Stepper(int STEP_PIN, int DIR_PIN, int EN_PIN, int MS1_PIN, int MS2_PIN, int MS3_PIN, int type)
{
	_step_pin = STEP_PIN;
	_dir_pin = DIR_PIN;
	_en_pin = EN_PIN;
	_ms_pin[2] = MS1_PIN;
	_ms_pin[1] = MS2_PIN;
	_ms_pin[0] = MS3_PIN;
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

		for(int i = 0; i < 3; i++) {
			if(_driver.microsteps[mode].pin_state[i] == 2) {
				pinMode(_ms_pin[i], INPUT);
			} else {
				pinMode(_ms_pin[i], OUTPUT);
				digitalWrite(_ms_pin[i], _driver.microsteps[mode].pin_state[i]);
			}
		}
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

driverType Stepper::getDriver() {
	return _driver;
}

static char * Stepper::getDriverTypeName(int type) {
  if(type < 0) {
    type = 0;
  } else if (type >= DRIVER_TYPE_COUNT) {
    type = DRIVER_TYPE_COUNT-1;
  }
  return drivers.drivers[type].driver_name;
}
