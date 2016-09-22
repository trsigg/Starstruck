#include "coreIncludes.c"
#include "motorGroup.c"

typedef struct {
	motorGroup rightLift, leftLift;
	tSensors encoder_L, encoder_R, potentiometer_L, potentiometer_R;
	bool hasEnc_L, hasEnc_R, hasPot_L, hasPot_R;
} liftGroup;

void initializeLift(liftGroup *lift, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed=15, int upPower=127, int downPower=-127) {
	configureButtonInput(lift->leftLift, posBtn, negBtn, stillSpeed, upPower, downPower);
	configureButtonInput(lift->rightLift, posBtn, negBtn, stillSpeed, upPower, downPower);
}

void setLeftLiftMotors(liftGroup *lift, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1) {
	initializeGroup(lift->leftLift, numMotors, motor1, motor2, motor3, motor4, motor5, motor6);
}

void setRightLiftMotors(liftGroup *lift, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1) {
	initializeGroup(lift->rightLift, numMotors, motor1, motor2, motor3, motor4, motor5, motor6);
}

void liftRuntime(liftGroup *lift) {
	int power = takeInput(lift->leftLift, false);

}
