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
int defTurnInts[5] = { 40, 100, -80, 100, 20 }; //initialPower, maxPower/kP, finalPower/kD, waitAtEnd, brakePower
//end turn defaults

enum correctionType { NONE, GYRO, ENCODER, AUTO };

//drive defaults
#define DRIVE_BRAKE_POWER 30 //power used during driveStraight braking
#define DRIVE_BRAKE_DURATION 100 //maximum duration of braking at end of driveStraight
correctionType defCorrectionType = AUTO;
bool defDriveBools[2] = { false, false }; //runAsTask, rawValue
int defDriveInts[6] = { 40, 120, 0, 1000, 100, 50 }; //initialPower, maxPower/kP, finalPower/kD, timeout, waitAtEnd, sampleTime
float defDriveFloats[4] = { 0, 0, 0, 3 }; //kP, kI, kD, minSpeed
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
	int power = rampRuntime( turnData.ramper, abs(gyroVal(autoDrive, DEGREES)) );

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

void turn(float angle, int in1=defTurnInts[0], int in2=defTurnInts[1], int in3=defTurnInts[2], bool runAsTask=defTurnRunAsTask, angleType angleType=defAngleType, int waitAtEnd=defTurnInts[3], int brakePower=defTurnInts[4]) { //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
	//initialize variables
	turnData.angle = abs(angle);
	initializeRampHandler(turnData.ramper, angle, in1, in2, in3);
	turnData.direction = sgn(angle);
	turnData.waitAtEnd = waitAtEnd;
	turnData.isTurning = true;

	resetGyro(autoDrive);

	if (runAsTask) {
		startTask(turnTask);
	}
	else {
		while (!turnIsComplete())
			turnRuntime();
		turnEnd();
	}
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
	rampHandler ramper;
	float leftDist, rightDist, totalDist; //distance traveled by each side of the drive (and their average)
	int direction; //sign of distance
	long timer; //for tracking timeout
} driveStruct;

driveStruct driveData;

bool drivingComplete() {
	return abs(driveData.totalDist)>driveData.distance  || time(driveData.timer)>driveData.timeout;
}

void driveStraightRuntime() {
	driveData.leftDist += driveEncoderVal(autoDrive, LEFT, driveData.rawValue);
	driveData.rightDist += driveEncoderVal(autoDrive, RIGHT, driveData.rawValue);
	driveData.totalDist = (driveData.leftDist + driveData.rightDist) / 2;

	if (driveEncoderVal(autoDrive) > driveData.minSpeed) driveData.timer = resetTimer(); //track timeout state
	resetDriveEncoders(autoDrive);

	//calculate error value and correction coefficient
	float error;

	switch (driveData.correctionType) {
		case ENCODER:
			error = driveData.rightDist - driveData.leftDist;
			break;
		case GYRO:
			error = sin(gyroVal(autoDrive, RADIANS)); //not sure if this is the right approach, but...
			break;
		default:
			error = 0;
	}

	float slaveCoeff = 1 + PID_runtime(driveData.pid, error);

	int power = rampRuntime(driveData.ramper, driveData.totalDist);

	setDrivePower(autoDrive, slaveCoeff*driveData.direction*power, driveData.direction*power);
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
		while (abs(gyroVal(autoDrive)) > 10) resetGyro(autoDrive); //I'm horrible, I know
	} else if (type==ENCODER && autoDrive.leftDrive.hasEncoder && autoDrive.rightDrive.hasEncoder) {
		driveData.correctionType = ENCODER;
	} else {
		driveData.correctionType = NONE;
	}
}

void driveStraight(float distance, bool runAsTask=defDriveBools[0], int in1=defDriveInts[0], int in2=defDriveInts[1], int in3=defDriveInts[2], float kP=defDriveFloats[0], float kI=defDriveFloats[1], float kD=defDriveFloats[2], correctionType correctionType=defCorrectionType, bool rawValue=defDriveBools[1], float minSpeed=defDriveFloats[3], int timeout=defDriveInts[3], int waitAtEnd=defDriveInts[4], int sampleTime=defDriveInts[5]) { //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
	//initialize variables
	driveData.distance = abs(distance);
	initializeRampHandler(driveData.ramper, distance, in1, in2, in3);
	driveData.direction = sgn(distance);
	driveData.rawValue = rawValue;
	driveData.minSpeed = minSpeed * sampleTime / 1000;
	driveData.timeout = timeout;
	driveData.waitAtEnd = waitAtEnd;
	driveData.sampleTime = sampleTime;
	driveData.isDriving = true;
	initializePID(driveData.pid, 0, kP, kI, kD);

	driveData.leftDist = 0;
	driveData.rightDist = 0;
	driveData.totalDist = 0;

	if (correctionType == AUTO) {
		setCorrectionType(ENCODER);

		if (driveData.correctionType == NONE) {
			setCorrectionType(GYRO);
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
