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

void setLiftMotors(liftGroup *lift, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1, tMotor motor7=port1, tMotor motor8=port1, tMotor motor9=port1, tMotor motor10=port1, tMotor motor11=port1, tMotor motor12=port1) {
	tMotor motors[12] = { motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10, motor11, motor12 };

	int numSideMotors = numMotors / 2;

	initializeGroup(lift->leftLift, numSideMotors, motors[0], motors[1], motors[2], motors[3], motors[4], motors[5]);
	initializeGroup(lift->rightLift, numSideMotors, motors[numSideMotors], motors[numSideMotors + 1], motors[numSideMotors + 2], motors[numSideMotors + 3], motors[numSideMotors + 4], motors[numSideMotors + 5]);
}

void liftRuntime(liftGroup *lift) {
	int power = takeInput(lift->leftLift, false);
}
