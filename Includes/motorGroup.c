#define numTargets 4

#include "timer.c"

#define DEFAULT_TIMEOUT 15

enum controlType { NONE, BUTTON, JOYSTICK };

typedef struct {
	tMotor motors[12];
	int numMotors;
	controlType controlType; //true if controlled by button, false if by joystick
	TVexJoysticks posInput, negInput; //inputs. NegInput only assigned if using button control
	//button control
	int upPower, downPower, stillSpeed;
	//execute maneuver
	int targetPos, endPower, maneuverPower;
	bool forward, maneuverExecuting; //forward: whether target is forwad from initial group position
	//joystick control
	int deadband; //range of motor values around 0 for which motors are not engaged
	bool isRamped; //whether group is ramped
	int msPerPowerChange; //if ramping, time between motor power changes, calculated using maxAcc100ms
	float powMap; //degree of polynomial to which inputs are mapped (1 for linear)
	float coeff; //factor by which motor powers are multiplied
	long lastUpdated; //ramping
	int absMin, absMax; //extreme potentiometer positions of motorGroup
	int maxPowerAtAbs, defPowerAtAbs; //maximum power at absolute position (pushing down from minimum or up from maximum) and default power if this is exceeded
	//sensors
	bool hasEncoder, hasPotentiometer;
	bool encoderReversed, potentiometerReversed;
	bool potentiometerDefault; //whether potentiometer (as opposed to encoder) is default sensor for position measurements
	tSensors encoder, potentiometer;
	//position targets
	int targets[numTargets];
	TVexJoysticks targetButtons[numTargets];
	int targetIndex;
	int prevPos;
	int timeout;
	long timer;
} motorGroup;

//#region initialization
void initializeGroup(motorGroup *group, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1, tMotor motor7=port1, tMotor motor8=port1, tMotor motor9=port1, tMotor motor10=port1, tMotor motor11=port1, tMotor motor12=port1) { //look, I know this is stupid.  But arrays in ROBOTC */really/* suck
	tMotor motors[12] = { motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10, motor11, motor12 };
	for (int i=0; i<numMotors; i++)
		group->motors[i] = motors[i];

	for (int i=0; i<numTargets; i++)
		group->targets[i] = -1;

	group->absMin = -1;
	group->absMax = 4097;

	group->numMotors = numMotors;
	group->targetIndex = -1;
	group->timeout = DEFAULT_TIMEOUT;
}

void configureButtonInput(motorGroup *group, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->controlType = BUTTON;
	group->posInput = posBtn;
	group->negInput = negBtn;
	group->stillSpeed = stillSpeed;
	group->upPower = upPower;
	group->downPower = downPower;
}

void configureJoystickInput(motorGroup *group, TVexJoysticks joystick, int deadband=10, bool isRamped=false, int maxAcc100ms=20, float powMap=1, int maxPow=127) {
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
			if (setAsDefault) group->potentiometerDefault = false;
			break;
	}
}

int encoderVal(motorGroup *group) {
	if (group->hasEncoder) {
		return (group->encoderReversed ?  -SensorValue[group->encoder] : SensorValue[group->encoder]);
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
	SensorValue[group->encoder] = resetVal;
}
//#endregion

//#region position limiting
void setAbsolutes(motorGroup *group, int min, int max=4097, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->absMax = max;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}
//#endregion

//#region motor targets
void createTarget(motorGroup *group, int position, TVexJoysticks btn) {
	if (group->hasPotentiometer) {
		for (int i=0; i<numTargets; i++) {
			if (group->targets[i] == -1) {
				group->targets[i] = position;
				group->targetButtons[i] = btn;
				break;
			}
		}
	}
}
//#endregion

void setPower(motorGroup *group, int power, bool overrideAbsolutes=false) {
	if (((getPosition(group) <= group->absMin && power < -group->maxPowerAtAbs) || (getPosition(group) >= group->absMax && power > group->maxPowerAtAbs) && !overrideAbsolutes)) //moving below absMin or above absMax and overrideAbsolutes is false TODO: make more efficient (a la wrist code)
		power = group->defPowerAtAbs * sgn(power);

	for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;
}

//#region position movement
void moveTowardPosition(motorGroup *group, int position, int power=127) {
	setPower(group, power * sgn(position - getPosition(group)));
}

void executeManeuver(motorGroup *group) {
	if (group->maneuverExecuting) {
		if (group->forward == (getPosition(group) > group->targetPos)) { //whether maneuver is finished
			setPower(group, group->endPower);
			group->maneuverExecuting = false;
		} else {
			setPower(group, (group->forward ? group->maneuverPower : -group->maneuverPower));
		}
	}
}

void createManeuver(motorGroup *group, int position, int endPower=0, int maneuverPower=127) {
	group->targetPos = position;
	group->endPower = endPower;
	group->maneuverPower = maneuverPower;
	group->forward = group->targetPos > getPosition(group);
	group->maneuverExecuting = true;
}

void goToPosition(motorGroup *group, int position, int endPower=0, int maneuverPower=127) {
	int displacementSign = sgn(position - getPosition(group));
	setPower(group, displacementSign*maneuverPower);
	while (sgn(position - getPosition(group)) == displacementSign);
	setPower(group, endPower);
}
//#endregion

//#region user input
void getTargetInput(motorGroup *group) {
	for (int i=0; i<numTargets; i++) {
		if (group->targets[i] == -1) {
			break;
		} else if (vexRT[group->targetButtons[i]] == 1) {
			group->timer = resetTimer();
			group->targetIndex = i;
			group->prevPos = getPosition(group);
		}
	}
}

int handleTargets(motorGroup *group) {
	int power = 0;

	getTargetInput(group);

	if (group->targetIndex == -1) {
		power = group->stillSpeed;
	} else {
		//go to target
		int newPos = getPosition(group);
		int target = group->targets[group->targetIndex];

		if (sgn(group->prevPos - target) == sgn(newPos - target)) {
			power = newPos>target ? -127 : 127;
			group->prevPos = newPos;
			group->timer = resetTimer();
		} else if (time(group->timer) > group->timeout) {
			group->targetIndex = -1;
			power = 0;
		}
	}

	return power;
}

int handleButtonInput(motorGroup *group) {
	int power = 0;

	if (vexRT[group->posInput] == 1) {
		power = group->upPower;
		group->targetIndex = -1;
	} else if (vexRT[group->negInput] == 1) {
		power = group->downPower;
		group->targetIndex = -1;
	} else {
		power = handleTargets(group);
	}

	if ((getPosition(group) < group->absMin && power<0) || (getPosition(group) > group->absMax && power>0))
		power = group->stillSpeed;

	return power;
}

int handleJoystickInput(motorGroup *group) {
	int power = 0;

	int input = vexRT[group->posInput];
	power = sgn(input) * group->coeff * abs(pow(input, group->powMap)) / pow(127, group->powMap-1);

	if (abs(power) < group->deadband) power = 0;

	//handle ramping
	if (group->isRamped) {
		long now = nPgmTime;
		int elapsed = now - group->lastUpdated;
		int currentPower = motor[ group->motors[0] ];

		if (elapsed > group->msPerPowerChange) {
			int maxDiff = elapsed / group->msPerPowerChange;

			if (abs(currentPower - power) < maxDiff) {
				group->lastUpdated = now;
			} else {
				power = (power>currentPower ? currentPower+maxDiff : currentPower-maxDiff);
				group->lastUpdated = now - (elapsed % group->msPerPowerChange);
			}
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
