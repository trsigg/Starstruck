#include "coreIncludes.c"
#include "motorGroup.c"
#include "PID.c"

typedef struct {
	motorGroup rightLift, leftLift;
	int absMin, min, middle, max, absMax;
	PID scissorPID;
} liftGroup;

void initializeLift(liftGroup *lift, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed=15, int upPower=127, int downPower=-127) {
	configureButtonInput(lift->leftLift, posBtn, negBtn, stillSpeed, upPower, downPower);
}

void setLiftMotors(liftGroup *lift, int numLeftMotors, int numRightMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1, tMotor motor7=port1, tMotor motor8=port1, tMotor motor9=port1, tMotor motor10=port1, tMotor motor11=port1, tMotor motor12=port1) {
	tMotor inputMotors[12] = { motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10, motor11, motor12 };
	tMotor leftMotors[6] = { motor1, motor2, motor3, motor4, motor5, motor6 };
	tMotor rightMotors[6] = { motor1, motor2, motor3, motor4, motor5, motor6 };

	for (int i=0; i<numLeftMotors; i++)
		leftMotors[i] = inputMotors[i];

	for (int i=numLeftMotors; i < (numLeftMotors + numRightMotors); i++)
		rightMotors[i - numLeftMotors] = inputMotors[i];

	initializeGroup(lift->leftLift, numLeftMotors, leftMotors[0], leftMotors[1], leftMotors[2], leftMotors[3], leftMotors[4], leftMotors[5]);
	initializeGroup(lift->rightLift, numrightMotors, rightMotors[0], rightMotors[1], rightMotors[2], rightMotors[3], rightMotors[4], rightMotors[5]);
}

void liftRuntime(liftGroup *lift) {
	int power = takeInput(lift->leftLift, false);
}
