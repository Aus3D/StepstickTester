#ifndef CONFIG_H
#define CONFIG_H

/////////////////////////////////////////////////////////////////////
// GENERAL CONFIG
/////////////////////////////////////////////////////////////////////
#define HAVE_ROTATIONAL_ENCODER true    //if we don't have an encoder, the user must watch the motor and verify it behaves correctly
#define ENCODER_INVERT

#define BOARD_V01_METROMINI

/////////////////////////////////////////////////////////////////////
// PINOUT
/////////////////////////////////////////////////////////////////////

#ifdef BOARD_V01_METROMINI
	#define STEPPER_STEP  2
	#define STEPPER_EN    3
	#define STEPPER_DIR   4
	#define STEPPER_MS1   5
	#define STEPPER_MS2   6
	#define STEPPER_MS3   7

	#define POWER_MOTOR_SUPPLY_EN     13
	#define POWER_MOTOR_SUPPLY_TEST   A3
	#define POWER_MOTOR_LOGIC_EN      A0
	#define POWER_MOTOR_LOGIC_TEST    A2
	#define POWER_BOARD_INPUT_TEST    A1

	#define TEST_BUTTON   8
	#define LED_GREEN     11
	#define LED_RED       12
	#define BUZZER        9

	#define LCD_RST       10
#endif

/////////////////////////////////////////////////////////////////////
// BUZZER
/////////////////////////////////////////////////////////////////////
#define BUZZER_PASSED_FREQ  440
#define BUZZER_FAILED_FREQ  98
#define BUZZER_PASSFAIL_DUR 500
#define BUZZER_TICK_FREQ    300
#define BUZZER_TICK_DUR     10

/////////////////////////////////////////////////////////////////////
// VOLTAGE MEASUREMENT
/////////////////////////////////////////////////////////////////////
#define INPUT_RAIL_DIV_R1 4700
#define INPUT_RAIL_DIV_R2 1000
#define INPUT_RAIL_THRESHOLD 6    //minimum voltage input power must be to pass test

#define LOGIC_RAIL_DIV_R1 4700
#define LOGIC_RAIL_DIV_R2 1000
#define LOGIC_RAIL_THRESHOLD 4.5  //minimum voltage stepper driver logic rail can be to pass test

#define SUPPLY_RAIL_DIV_R1 4700
#define SUPPLY_RAIL_DIV_R2 1000
#define SUPPLY_RAIL_THRESHOLD 1    //minimum amount below input rail stepper supply rail can be to pass test

#define RAIL_SHORT_WAIT_TIME 250
#define POWER_DOWN_TIMEOUT	 500
#define POWER_DOWN_THRESHOLD 1	//voltage below which the rail is considered powered-down

/////////////////////////////////////////////////////////////////////
// DRIVER TEST SETTINGS
/////////////////////////////////////////////////////////////////////
#define MOTOR_TEST_ROTATE_DEGREES     90  //we rotate back or forwards 90 degrees
#define MOTOR_TEST_ANGULAR_TOLERANCE  5   //number of degrees forward/reverse test can be off by
#define MOTOR_TEST_REST_TIME          100 //time (ms) to wait between test motions
#define MOTOR_TEST_SPEED              6   //motor speed (higher is faster)


#endif //CONFIG_H
