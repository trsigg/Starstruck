//TODO: change ramping to be handled by rampHandler
//TODO: add #defined constants to defaults lists

#define autoDrive drive

#include "coreIncludes.c"
#include "parallelDrive.c"
#include "PID.c"
#include "rampHandler.c"
#include "timer.c"

//turn defaults
angleType defAngleType = DEGREES;
bool defTurnBools[2] = { false, true }; //runAsTask, useGyro
int defTurnInts[3] = { 20, 100, 100 }; //brakePower, waitAtEnd, brakeDuration
float defTurnFloats[3] = { 40, 100, -80 }; //initialPower, maxPower/kP, finalPower/kD
//end turn defaults

enum correctionType { NONE, GYRO, ENCODER, AUTO };

//drive defaults
#define DRIVE_BRAKE_DURATION 100
correctionType defCorrectionType = AUTO;
bool defDriveBools[2] = { false, false }; //runAsTask, rawValue
int defDriveInts[4] = { 1000, 30, 100, 50 }; //timeout, brakePower, waitAtEnd, sampleTime
float defDriveFloats[7] = { 40, 120, -15, 0, 0, 0, 40 }; //initialPower, maxPower/kP_r, finalPower/kD_r, kP_c, kI_c, kD_c, minSpeed
//end drive defaults


parallel_drive autoDrive;

//turning region
typedef struct {
	float angle; //positive for clockwise, negative for counterclockwise
	rampHandler ramper; //used for ramping motor powers
	int waitAtEnd; //delay after finishing turn (default 100ms for braking)
	int brakeDuration; //maximum duration of braking at end of turn
	int brakePower; //the motor power while braking
	bool usingGyro; //whether to use encoders or gyro for turns
	bool isTurning; //whether turn is executing (useful for running as task)
	int direction; //sign of angle
} turnStruct;

turnStruct turnData;

bool turnIsComplete() {
	if (turnData.usingGyro) {
		return abs(gyroVal(autoDrive, DEGREES)) >= turnData.angle;
	} else {
		return abs(driveEncoderVal(autoDrive)) >= turnData.angle;
	}
}

void turnRuntime() {
	int power = rampRuntime( turnData.ramper, abs(gyroVal(autoDrive, DEGREES)) );

	setDrivePower(autoDrive, turnData.direction*power, -turnData.direction*power);
}

void turnEnd() {
	//brake
	setDrivePower(autoDrive, -turnData.direction * turnData.brakePower, turnData.direction * turnData.brakePower);
	int brakeDelay = limit(0, turnData.brakeDuration, turnData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(autoDrive, 0, 0);

	wait1Msec(turnData.waitAtEnd>turnData.brakeDuration ? turnData.waitAtEnd-turnData.brakeDuration : 0);
	turnData.isTurning = false;
}

task turnTask() {
	while (!turnIsComplete()) {
		turnRuntime();
		EndTimeSlice();
	}
	turnEnd();
}

void turn(float angle, float in1=defTurnFloats[0], float in2=defTurnFloats[1], float in3=defTurnFloats[2], bool runAsTask=defTurnBools[0], angleType angleType=defAngleType, bool useGyro=defTurnBools[1], int brakePower=defTurnInts[0], int waitAtEnd=defTurnInts[1], int brakeDuration=defTurnInts[2]) { //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
	//initialize variables
	float formattedAngle = convertAngle(abs(angle), DEGREES, angleType);
	turnData.angle = (useGyro ? formattedAngle : 2*PI*autoDrive.width*formattedAngle/360.);
	initializeRampHandler(turnData.ramper, angle, in1, in2, in3);
	turnData.direction = sgn(angle);
	turnData.waitAtEnd = waitAtEnd;
	turnData.brakeDuration = brakeDuration;
	turnData.usingGyro = useGyro;
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
//end turning region


//driveStraight region
typedef struct {
	float distance;
	bool rawValue; //whether distance is measured in encoder clicks or inches
	float minSpeed; //minimum speed during maneuver to prevent timeout (distance per 100ms)
	int timeout; //amount of time after which a drive action sensing lower speed than minSpeed ceases (ms)
	int sampleTime; //time between motor power adjustments
	int waitAtEnd; //duration of pause at end of driving
	int brakeDuration; //maximum duration of braking at end of turn
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
	setDrivePower(autoDrive, -driveData.direction * driveData.brakePower, -driveData.direction * driveData.brakePower);
	int brakeDelay = limit(0, driveData.brakePower, driveData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(autoDrive, 0, 0);

	wait1Msec(driveData.waitAtEnd>driveData.brakeDuration ? driveData.waitAtEnd-driveData.brakeDuration : 0);
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

void driveStraight(float distance, bool runAsTask=defDriveBools[0], float in1=defDriveFloats[0], float in2=defDriveFloats[1], float in3=defDriveFloats[2], float kP=defDriveFloats[3], float kI=defDriveFloats[4], float kD=defDriveFloats[5], correctionType correctionType=defCorrectionType, bool rawValue=defDriveBools[1], float minSpeed=defDriveFloats[3], int timeout=defDriveInts[0], int brakePower=defDriveInts[1], int waitAtEnd=defDriveInts[2], int sampleTime=defDriveInts[3]) { //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
	//initialize variables
	driveData.distance = abs(distance);
	initializeRampHandler(driveData.ramper, distance, in1, in2, in3);
	driveData.direction = sgn(distance);
	driveData.rawValue = rawValue;
	driveData.minSpeed = minSpeed * sampleTime / 1000;
	driveData.timeout = timeout;
	driveData.waitAtEnd = waitAtEnd;
	driveData.sampleTime = sampleTime;
	driveData.brakeDuration = DRIVE_BRAKE_DURATION;
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
//end driveStraight region
