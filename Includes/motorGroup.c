#define numTargets 4

#include "timer.c"
#include "PID.c"

enum controlType { NONE, BUTTON, JOYSTICK };

typedef struct {
	tMotor motors[12];
	int numMotors;
	controlType controlType; //true if controlled by button, false if by joystick
	TVexJoysticks posInput, negInput; //inputs. NegInput only assigned if using button control
	//button control
	int upPower, downPower, stillSpeed;
	//execute maneuver
	int targetPos, endPower, maneuverPower, maneuverTimeout;
	bool forward, maneuverExecuting; //forward: whether target is forwad from initial group position
	long maneuverTimer;
	//maintainPos
	PID posPID;	//PID controller which maintains position
	bool activelyMaintining;	//whether position is being maintained
	//joystick control
	int deadband; //range of motor values around 0 for which motors are not engaged
	bool isRamped; //whether group is ramped
	int msPerPowerChange; //if ramping, time between motor power changes, calculated using maxAcc100ms
	float powMap; //degree of polynomial to which inputs are mapped (1 for linear)
	float coeff; //factor by which motor powers are multiplied
	long lastUpdated; //ramping
	int absMin, absMax; //extreme  positions of motorGroup
	bool hasAbsMin, hasAbsMax;
	int maxPowerAtAbs, defPowerAtAbs; //maximum power at absolute position (pushing down from minimum or up from maximum) and default power if this is exceeded
	//sensors
	bool hasEncoder, hasPotentiometer;
	bool encoderReversed, potentiometerReversed;
	bool potentiometerDefault; //whether potentiometer (as opposed to encoder) is default sensor for position measurements
	tSensors encoder, potentiometer;
	//position targets
	int targets[numTargets];
	TVexJoysticks targetButtons[numTargets];
	int targetTimeout, targetPower;
} motorGroup;

//#region initialization
void initializeGroup(motorGroup *group, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1, tMotor motor7=port1, tMotor motor8=port1, tMotor motor9=port1, tMotor motor10=port1, tMotor motor11=port1, tMotor motor12=port1) { //look, I know this is stupid.  But arrays in ROBOTC */really/* suck
	tMotor motors[12] = { motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10, motor11, motor12 };
	for (int i=0; i<numMotors; i++)
		group->motors[i] = motors[i];

	for (int i=0; i<numTargets; i++)
		group->targets[i] = -1;

	group->numMotors = numMotors;
	group->maneuverExecuting = false;
}

void configureButtonInput(motorGroup *group, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->controlType = BUTTON;
	group->posInput = posBtn;
	group->negInput = negBtn;
	group->stillSpeed = stillSpeed;
	group->upPower = upPower;
	group->downPower = downPower;
}

void configureJoystickInput(motorGroup *group, TVexJoysticks joystick, int deadband=10, bool isRamped=false, int maxAcc100ms=60, float powMap=1, int maxPow=127) {
	group->controlType = JOYSTICK;
	group->posInput = joystick;
	group->deadband = deadband;
	group->isRamped = isRamped;
	group->msPerPowerChange = 100 / maxAcc100ms;
	group->powMap = powMap;
	group->coeff = maxPow /  127.0;
	group->lastUpdated = nPgmTime;
}
//#endregion

//#region sensors
void addSensor(motorGroup *group, tSensors sensor, bool reversed=false, bool setAsDefault=true) {
	switch (SensorType[sensor]) {
		case sensorPotentiometer:
			group->hasPotentiometer = true;
			group->potentiometer = sensor;
			group->potentiometerReversed = reversed;
			if (setAsDefault) group->potentiometerDefault = true;
			break;
		case sensorQuadEncoder:
			group->hasEncoder = true;
			group->encoder = sensor;
			group->encoderReversed = reversed;
			SensorValue[sensor] = 0;
			if (setAsDefault) group->potentiometerDefault = false;
			break;
	}
}

int encoderVal(motorGroup *group) {
	if (group->hasEncoder) {
		return SensorValue[group->encoder] * (group->encoderReversed ?  -1 : 1);
	} else {
		return 0;
	}
}

int potentiometerVal(motorGroup *group) {
	if (group->hasPotentiometer) {
		return (group->potentiometerReversed ? 4096-SensorValue[group->potentiometer] : SensorValue[group->potentiometer]);
	} else {
		return 0;
	}
}

int getPosition(motorGroup *group) {
	if (group->hasPotentiometer && group->hasEncoder) {
		return group->potentiometerDefault ? potentiometerVal(group) : encoderVal(group);
	} else {
		return (group->hasEncoder ? encoderVal(group) : potentiometerVal(group));
	}
}

void resetEncoder(motorGroup *group, int resetVal=0) {
	SensorValue[group->encoder] = resetVal * (group->encoderReversed ?  -1 : 1);
}
//#endregion

//#region position limiting
void setAbsMax(motorGroup *group, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMax = max;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsMin(motorGroup *group, int min, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->hasAbsMin = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsolutes(motorGroup *group, int min, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->absMax = max;
	group->hasAbsMin = true;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}
//#endregion

//#region motor targets
void createTarget(motorGroup *group, int position, TVexJoysticks btn, int power=127, int timeout=10) {
	for (int i=0; i<numTargets; i++) {
		if (group->targets[i] == -1) {
			group->targets[i] = position;
			group->targetButtons[i] = btn;
			break;
		}
	}

	group->targetPower = power;
	group->targetTimeout = timeout;
}
//#endregion

int setPower(motorGroup *group, int power, bool overrideAbsolutes=false) {
	if (!overrideAbsolutes) {
		if (group->hasAbsMin && getPosition(group) <= group->absMin && power < -group->maxPowerAtAbs)
			power = -group->defPowerAtAbs;

		if (group->hasAbsMax && getPosition(group) >= group->absMax && power > group->maxPowerAtAbs)
			power = group->defPowerAtAbs;
	}

	for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;

	return power;
}

//#region position movement
	//#subregion maintainPos
	void setTargetingPIDconsts(motorGroup *group, float kP, float kI, float kD, int minSampleTime=0, int integralMin=-127, int integralMax=127) {
		initializePID(group->posPID, 0, kP, kI, kD, minSampleTime, integralMin, integralMax);
	}

	void setTargetPosition(motorGroup *group, int position) {
		changeTarget(group->posPID, position);
		group->activelyMaintining = true;
	}

	void maintainTargetPos(motorGroup *group) {
		if (group->activelyMaintining && group->posPID.kP != 0) {
			setPower(group, PID_runtime(group->posPID, getPosition(group)));
		}
	}

	void stopTargeting(motorGroup *group) { group->activelyMaintining = false; }

	bool errorLessThan(motorGroup *group, int errorMargin) {	//returns true if PID error is less than specified margin
		return abs(group->posPID.target - getPosition(group)) < errorMargin;
	}
	//#endsubregion

int moveTowardPosition(motorGroup *group, int position, int power=127) {
	return setPower(group, power * sgn(position - getPosition(group)));
}

void executeManeuver(motorGroup *group) {
	if (group->maneuverExecuting) {
		if (group->forward == (getPosition(group) < group->targetPos)) {
			group->maneuverTimer = resetTimer();
			setPower(group, group->maneuverPower);
		}

		if (time(group->maneuverTimer) > group->maneuverTimeout) {
			group->maneuverExecuting = false;
			setPower(group, group->endPower);
		}
	}
}

void createManeuver(motorGroup *group, int position, int endPower=0, int maneuverPower=127, int timeout=10) {
	group->targetPos = position;
	group->endPower = endPower;
	group->forward = group->targetPos > getPosition(group);
	group->maneuverPower = abs(maneuverPower) * (group->forward ? 1 : -1);
	group->maneuverExecuting = true;
	group->maneuverTimeout = timeout;
	group->maneuverTimer = resetTimer();

	setPower(group, maneuverPower);
}

void goToPosition(motorGroup *group, int position, int endPower=0, int maneuverPower=127, int timeout=100) {
	long posTimer = resetTimer();
	int displacementSign = sgn(position - getPosition(group));
	setPower(group, displacementSign*maneuverPower);

	while (time(posTimer) < timeout) {
		if (sgn(position - getPosition(group)) == displacementSign) posTimer = resetTimer();
	}

	setPower(group, endPower);
}
//#endregion

//#region user input
void getTargetInput(motorGroup *group) {
	for (int i=0; i<numTargets; i++) {
		if (group->targets[i] == -1) {
			break;
		} else if (vexRT[group->targetButtons[i]] == 1) {
			createManeuver(group, group->targets[i], group->stillSpeed, group->targetPower, group->targetTimeout);
		}
	}
}

int handleButtonInput(motorGroup *group) {
	if (vexRT[group->posInput] == 1) {
		group->maneuverExecuting = false;
		return group->upPower;
	} else if (vexRT[group->negInput] == 1) {
		group->maneuverExecuting = false;
		return group->downPower;
	} else {
		getTargetInput(group);

		executeManeuver(group);

		if (group->maneuverExecuting)
			return group->maneuverPower;
		else
			return group->stillSpeed;
	}
}

int handleJoystickInput(motorGroup *group) {
	int input = vexRT[group->posInput];
	int power = sgn(input) * group->coeff * abs(pow(input / 127.0, group->powMap)) * 127;

	if (abs(power) < group->deadband) power = 0;

	//handle ramping
	if (group->isRamped) {
		long now = nPgmTime;
		int elapsed = now - group->lastUpdated;
		int currentPower = motor[ group->motors[0] ];

		if (elapsed >= group->msPerPowerChange) {
			group->lastUpdated = now;

			if (abs(power) > abs(currentPower)) {	//only ramp up in absolute value
				int maxDiff = elapsed / group->msPerPowerChange;

				if (abs(currentPower - power) > maxDiff) {
					group->lastUpdated = now - (elapsed % group->msPerPowerChange);
					return (power>currentPower ? currentPower+maxDiff : currentPower-maxDiff);
				}
			}
		} else {
			return currentPower;
		}
	}

	return power;
}

int takeInput(motorGroup *group, bool setMotors=true) {
	int power = 0;

	switch (group->controlType) {
		case BUTTON:
			power = handleButtonInput(group);
			break;
		case JOYSTICK:
			power = handleJoystickInput(group);
			break;
	}

	if (setMotors) setPower(group, power);

	return power;
}
//#endregion
