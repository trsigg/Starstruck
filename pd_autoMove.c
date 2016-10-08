//TODO: change ramping to be handled by rampHandler
//TODO: add #defined constants to defaults lists

#define autoDrive drive

#include "coreIncludes.c"
#include "parallelDrive.c"
#include "PID.c"
#include "rampHandler.c"
#include "timer.c"

//turn defaults
#define TURN_BRAKE_DURATION 100 //maximum duration of braking at end of turn
angleType defAngleType = DEGREES;
bool defTurnRunAsTask = false;
int defTurnInts[5] = { 40, 100, -100, 100, 20 }; //initialPower, maxPower, finalPower, waitAtEnd, brakePower
//end turn defaults

enum correctionType { NONE, GYRO, ENCODER, AUTO };

//drive defaults
#define DRIVE_BRAKE_POWER 30 //power used during driveStraight braking
#define DRIVE_BRAKE_DURATION 100 //maximum duration of braking at end of driveStraight
correctionType defCorrectionType = AUTO;
bool defDriveBools[2] = { false, false }; //runAsTask, rawValue
int defDriveInts[6] = { 40, 120, 0, 1000, 100, 50 }; //initialPower, maxPower, finalPower, timeout, waitAtEnd, sampleTime
float defDriveFloats[4] = { 0.25, 0.25, 0.25, 3 }; //kP, kI, kD, minSpeed
//end drive defaults


parallel_drive autoDrive;

//turning region
typedef struct {
	float angle; //positive for clockwise, negative for counterclockwise
	rampHandler ramper; //used for ramping motor powers
	int waitAtEnd; //delay after finishing turn (default 100ms for braking)
	int brakePower; //the motor power while braking
	bool isTurning; //whether turn is executing (useful for running as task)
	int direction; //sign of angle
} turnStruct;

turnStruct turnData;

bool turnIsComplete() {
	return abs(gyroVal(autoDrive, DEGREES)) >= abs(turnData.angle);
}

void turnRuntime() {
	int gyro = abs(gyroVal(autoDrive, DEGREES));
	int power = turnData.a*pow(gyro, 2) +  turnData.b*gyro + turnData.c;

	setDrivePower(autoDrive, turnData.direction*power, -turnData.direction*power);
}

void turnEnd() {
	//brake
	setDrivePower(autoDrive, -turnData.direction * turnData.brakePower, turnData.direction * turnData.brakePower);
	int brakeDelay = limit(0, TURN_BRAKE_DURATION, turnData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(autoDrive, 0, 0);

	wait1Msec(turnData.waitAtEnd>TURN_BRAKE_DURATION ? turnData.waitAtEnd-TURN_BRAKE_DURATION : 0);
	turnData.isTurning = false;
}

task turnTask() {
	while (!turnIsComplete()) {
		turnRuntime();
		EndTimeSlice();
	}
	turnEnd();
}

void turn(parallel_drive &drive, float angle, angleType angleType=defAngleType, bool runAsTask=defTurnRunAsTask, int initialPower=defTurnInts[0], int maxPower=defTurnInts[1], int finalPower=defTurnInts[2], int waitAtEnd=defTurnInts[3], int brakePower=defTurnInts[4]) {
	//initialize variables
	turnData.angle = abs(angle);
	turnData.a = a;
	turnData.b = b;
	turnData.c = c;
	turnData.direction = sgn(angle);
	turnData.waitAtEnd = waitAtEnd;
	turnData.isTurning = true;

	resetGyro(autoDrive);

	if (runAsTask) {
		startTask(turnTask);
	}
	else {
		while (!turnIsComplete()) { turnRuntime(); }
		turnEnd();
	}
}

void turn(parallel_drive &drive, float angle, angleType angleType=defAngleType, bool runAsTask=defTurnRunAsTask, int initialPower=defTurnInts[0], int maxPower=defTurnInts[1], int finalPower=defTurnInts[2], int waitAtEnd=defTurnInts[3], int brakePower=defTurnInts[4]) {
	angle = convertAngle(angle, DEGREES, angleType);
	float a = (pow(angle, 2) * (finalPower+initialPower-2*maxPower) - 2*sqrt(pow(angle, 4) * (finalPower-maxPower) * (initialPower-maxPower))) / pow(angle, 4);
	float b = ((finalPower-initialPower)/angle - a*angle) * sgn(angle);

	_turn_(drive, angle, a, b, initialPower, runAsTask, waitAtEnd, brakePower);
}

void setTurnDefaults(angleType angleType, bool runAsTask=defTurnRunAsTask, int initialPower=defTurnInts[0], int maxPower=defTurnInts[1], int finalPower=defTurnInts[2], int waitAtEnd=defTurnInts[3], int brakePower=defTurnInts[4]) {
	defAngleType = angleType;
	defTurnRunAsTask = runAsTask;

	int defInts[5] = { initialPower, maxPower, finalPower, waitAtEnd, brakePower };

	defTurnInts = defInts;
}
//end turning region


//driveStraight region
typedef struct {
	float distance;
	bool rawValue; //whether distance is measured in encoder clicks or inches
	float minSpeed; //minimum speed during maneuver to prevent timeout (distance per 100ms)
	int timeout; //amount of time after which a drive action sensing lower speed than minSpeed ceases (ms)
	int sampleTime; //time between motor power adjustments
	int waitAtEnd; //duration of pause at end of driving
	int brakePower; // motor power during braking
	correctionType correctionType; //which sensor inputs are used for correction
	bool isDriving; //whether driving action is being executed (true if driving, false othrewise)
	//interal variables
	PID pid;
	float a, b, c; //constants in equation for determining motor power
	float totalDist; //distance traveled so far
	int direction; //sign of distance
	long timer; //for tracking timeout
} driveStruct;

driveStruct driveData;

bool drivingComplete() {
	return abs(driveData.totalDist)>driveData.distance  || time(driveData.timer)>driveData.timeout;
}

void driveStraightRuntime() {
	float leftDist = driveEncoderVal(autoDrive, LEFT, driveData.rawValue);
	float rightDist = driveEncoderVal(autoDrive, RIGHT, driveData.rawValue);
	driveData.totalDist += (leftDist + rightDist) / 2;

	//calculate error value and correction coefficient
	float error;

	switch (driveData.correctionType) {
		case ENCODER:
			driveData.error = rightDist - leftDist;
			break;
		case GYRO:
			error = sin(gyroVal(autoDrive, RADIANS)); //not sure if this is the right approach, but...
			break;
		default:
			error = 0;
	}

	float slaveCoeff = 1 + PID_runtime(driveData.pid, error);

	int power = driveData.a*pow(driveData.totalDist, 2) + driveData.b*driveData.totalDist + driveData.c;

	setDrivePower(autoDrive, slaveCoeff*driveData.direction*power, driveData.direction*power);

	if (driveEncoderVal(autoDrive) > driveData.minSpeed) driveData.timer = resetTimer(); //track timeout state
	resetDriveEncoders(autoDrive);
}

void driveStraightEnd() {
	//brake
	setDrivePower(autoDrive, -driveData.direction * DRIVE_BRAKE_POWER, -driveData.direction * DRIVE_BRAKE_POWER);
	int brakeDelay = limit(0, DRIVE_BRAKE_DURATION, driveData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(autoDrive, 0, 0);

	wait1Msec(driveData.waitAtEnd>DRIVE_BRAKE_DURATION ? driveData.waitAtEnd-DRIVE_BRAKE_DURATION : 0);
	driveData.isDriving = false;
}

task driveStraightTask() {
	while (!drivingComplete()) {
		driveStraightRuntime();

		wait1Msec(driveData.sampleTime);
	}
	driveStraightEnd();
}

void setCorrectionType(correctionType type) {
	if (type==GYRO && autoDrive.hasGyro) {
		driveData.correctionType = GYRO;
	} else if (type==ENCODER && autoDrive.leftDrive.hasEncoder && autoDrive.rightDrive.hasEncoder) {
		driveData.correctionType = ENCODER;
	} else {
		driveData.correctionType = NONE;
	}
}

void _driveStraight_(parallel_drive &drive, float distance, float a, float b, float c, bool runAsTask=false, float kP=0.25, float kI=0.25, float kD=0.25, correctionType correctionType=AUTO, bool rawValue=false, float minSpeed=3, int timeout=800, int waitAtEnd=250, int sampleTime=50) {
	//initialize variables
	driveData.distance = abs(distance);
	driveData.a = a;
	driveData.b = b;
	driveData.c = c;
	driveData.direction = sgn(distance);
	driveData.rawValue = rawValue;
	driveData.minSpeed = minSpeed * sampleTime / 1000;
	driveData.timeout = timeout;
	driveData.waitAtEnd = waitAtEnd;
	driveData.sampleTime = sampleTime;
	driveData.isDriving = true;
	initializePID(driveData.pid, 0, kP, kI, kD);

	driveData.totalDist = 0;
	driveData.error = 0;

	if (correctionType == AUTO) {
		setCorrectionType(ENCODER);

		if (driveData.correctionType == NONE) {
			setCorrectionType(GYRO);
			resetGyro(autoDrive);
		}
	} else {
		setCorrectionType(correctionType);
	}

	//initialize sensors
	resetDriveEncoders(autoDrive);

	driveData.timer = resetTimer();

	if (runAsTask) {
		startTask(driveStraightTask);
	} else { //runs as function
		while (!drivingComplete()) {
			driveStraightRuntime();
			wait1Msec(driveData.sampleTime);
		}
		driveStraightEnd();
	}
}

void driveStraight(parallel_drive &drive, float distance, bool runAsTask=defDriveBools[0], int initialPower=defDriveInts[0], int maxPower=defDriveInts[1], int finalPower=defDriveInts[2], float kP=defDriveFloats[0], float kI=defDriveFloats[1], float kD=defDriveFloats[2], correctionType correctionType=defCorrectionType, bool rawValue=defDriveBools[1], float minSpeed=defDriveFloats[3], int timeout=defDriveInts[3], int waitAtEnd=defDriveInts[4], int sampleTime=defDriveInts[5]) {
	float a = (pow(distance, 2) * (finalPower+initialPower-2*maxPower) - 2*sqrt(pow(distance, 4) * (finalPower-maxPower) * (initialPower-maxPower))) / pow(distance, 4);
	float b = ((finalPower-initialPower)/distance - a*distance) * sgn(distance);

	_driveStraight_(drive, distance, a, b, initialPower, runAsTask, kP, kI, kD, correctionType, rawValue, minSpeed, timeout, waitAtEnd, sampleTime);
}

void setDriveDefaults(bool runAsTask, int initialPower=defDriveInts[0], int maxPower=defDriveInts[1], int finalPower=defDriveInts[2], float kP=defDriveFloats[0], float kI=defDriveFloats[1], float kD=defDriveFloats[2], correctionType correctionType=defCorrectionType, bool rawValue=defDriveBools[1], float minSpeed=defDriveFloats[3], int timeout=defDriveInts[3], int waitAtEnd=defDriveInts[4], int sampleTime=defDriveInts[5]) {
	defCorrectionType = correctionType;

	bool defBools[2] = { runAsTask, rawValue };
	int defInts[6] = { initialPower, maxPower, finalPower, timeout, waitAtEnd, sampleTime };
	float defFloats[4] = { kP, kI, kD, minSpeed };

	defDriveBools = defBools;
	defDriveInts = defInts;
	defDriveFloats = defFloats;
}
//end driveStraight region
