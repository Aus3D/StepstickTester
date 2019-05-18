#ifndef TESTS_H
#define TESTS_H

#define TEST_FAILED 	0
#define TEST_PASSED 	1
#define TEST_SKIPPED 	2

int testFunction_test5V(int prevStatus);
int testFunction_test12V(int prevStatus);
int testFunction_testInputVoltage(int prevStatus);
int testFunction_testStepperMotion(int prevStatus);
int testFunction_powerDown(int prevStatus);

#endif //TESTS_H