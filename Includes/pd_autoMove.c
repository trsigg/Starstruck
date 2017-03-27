#define autoDrive drive

#include "coreIncludes.c"
#include "parallelDrive.c"
#include "PID.c"
#include "rampHandler.c"
#include "timer.c"

enum correctionType { NONE, GYRO, ENCODER, AUTO };

parallel_drive autoDrive;

//#region defaults
typedef struct {
	angleType defAngleType;
	bool runAsTask;
	bool useGyro;
	bool reversed;	//reverses all turns (for mirroring auton routines)
	int timeout;
	int brakePower;
	int waitAtEnd;
	int brakeDuration;
	float error;
	float rampConst1, rampConst2, rampConst3, rampConst4; // initialPower/kP; maxPower/kD; finalPower/error; 0/timeout
} turnDefsStruct;

typedef struct {
	correctionType defCorrectionType;
	bool runAsTask;
	bool rawValue;
	int brakeDuration;
	int pdTimeout;
	int timeout;
	int brakePower, brakeDuration;
	int waitAtEnd;
	int sampleTime;
	float rampConst1, rampConst2, rampConst3, rampConst4;
	float kP_c, kI_c, kD_c; //correction PID constants
	float minSpeed;
} driveDefsStruct;

turnDefsStruct turnDefaults;
driveDefsStruct driveDefaults;

void initializeAutoMovement() {
	//turning
	turnDefaults.defAngleType = DEGREES;
	turnDefaults.runAsTask = false;
	turnDefaults.useGyro = true;
	turnDefaults.reversed = false;
	turnDefaults.brakePower = 20;
	turnDefaults.waitAtEnd = 100;
	turnDefaults.brakeDuration = 100;
	turnDefaults.error = 2;
	turnDefaults.rampConst1 = 40;		// initialPower/kP
	turnDefaults.rampConst2 = 100;	// maxPower/kD
	turnDefaults.rampConst3 = -40;	// finalPower/error
	turnDefaults.rampConst4 = 0;		// 0/timeout

	//driving
	driveDefaults.defCorrectionType = AUTO;
	driveDefaults.runAsTask = false;
	driveDefaults.rawValue = false;
	driveDefaults.brakeDuration = 100;
	driveDefaults.timeout = 1000;
	driveDefaults.brakePower = 30;
	driveDefaults.waitAtEnd = 100;
	driveDefaults.sampleTime = 50;
	driveDefaults.rampConst1 = 40;
	driveDefaults.rampConst2 = 120;
	driveDefaults.rampConst3 = -15;
	driveDefaults.rampConst4 = 0;
	driveDefaults.kP_c = .55;
	driveDefaults.kI_c = 0.007;
	driveDefaults.kD_c = 0.15;
	driveDefaults.minSpeed = 10;
}
//#endregion

//#region turning
typedef struct {
	float angle; //positive for clockwise, negative for counterclockwise
	rampHandler ramper; //used for ramping motor powers
	float error;	//allowable deviation from target value
	int timeout;	//time robot is required to be within <error> of the target before continuing
	long timer;	//tracks timeout state - TODO: movement timer, completion timer
	int waitAtEnd; //delay after finishing turn (default 100ms for braking)
	int brakeDuration; //maximum duration of braking at end of turn
	int brakePower; //the motor power while braking
	bool usingGyro; //whether to use encoders or gyro for turns
	bool isTurning; //whether turn is executing (useful for running as task)
	int direction; //sign of angle
} turnStruct;

turnStruct turnData;

float turnProgress() {
	return turnData.usingGyro ? gyroVal(autoDrive, DEGREES) : driveEncoderVal(autoDrive);
}

bool turnIsComplete() {
	if (ramper->algorithm == PID)
		return time(turnData.timer) > turnData.timeout;
	else	//algorithm is QUAD
		return abs(turnProgress()) >= turnData.angle;
}

void turnRuntime() {
	float progress = turnProgress();

	int power = rampRuntime(turnData.ramper, abs(progress()));

	setDrivePower(autoDrive, turnData.direction*power, -turnData.direction*power);

	if (abs(progress - turnData.angle) > turnData.error)	//track timeout state
		turnData.timer = resetTimer;
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

void turn(float angle, bool runAsTask=turnDefaults.runAsTask, float in1=turnDefaults.rampConst1, float in2=turnDefaults.rampConst2, float in3=turnDefaults.rampConst3, int in4=turnDefaults.rampConst4, angleType angleType=turnDefaults.defAngleType, bool useGyro=turnDefaults.useGyro, int brakePower=turnDefaults.brakePower, int waitAtEnd=turnDefaults.waitAtEnd, int brakeDuration=turnDefaults.brakeDuration) { //for PD, in1=kP, in2=kD, in3=error, in4=timeout; for quad ramping, in1=initial, in2=maximum, in3=final, and in4=0
	//initialize variables
	if (turnDefaults.reversed) angle *= -1;
	float formattedAngle = convertAngle(abs(angle), DEGREES, angleType);
	turnData.angle = (useGyro ? formattedAngle : PI*autoDrive.width*formattedAngle/360.);
	turnData.direction = sgn(angle);
	turnData.waitAtEnd = waitAtEnd;
	turnData.brakeDuration = brakeDuration;
	turnData.usingGyro = useGyro;
	turnData.isTurning = true;

	if (in4 == 0) {
		initializeRampHandler(turnData.ramper, formattedAngle, in1, 0, in2);
		turnData.error = in3;
		turnData.timeout = in4;
		turnData.timer = resetTimer();
	} else {
		initializeRampHandler(turnData.ramper, formattedAngle, in1, in2, in3);
	}

	resetGyro(autoDrive);

	if (runAsTask) {
		startTask(turnTask);
	}
	else {
		while (!turnIsComplete()) {
			turnRuntime();
			EndTimeSlice();
		}
		turnEnd();
	}
}
//#endregion

//#region driveStraight
typedef struct {
	float distance;
	bool rawValue; //whether distance is measured in encoder clicks or inches
	float minSpeed; //minimum speed during maneuver to prevent timeout (distance per 100ms)
	float error;	//allowable deviation from target value
	int pdTimeout;	//time robot is required to be within <error> of the target before continuing
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
	long pdTimer;	//for tracking pd timeout
	long timer; //for tracking timeout (time without movement)
} driveStruct;

driveStruct driveData;

bool drivingComplete() {
	return driveData.totalDist>driveData.distance  || time(driveData.timer)>driveData.timeout;
}

void driveStraightRuntime() {
	driveData.leftDist += abs(driveEncoderVal(autoDrive, LEFT, driveData.rawValue));
	driveData.rightDist += abs(driveEncoderVal(autoDrive, RIGHT, driveData.rawValue));
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

	int power = rampRuntime(driveData.ramper, driveData.totalDist);

	float correctionPercent = 1 + PID_runtime(driveData.pid, error);
	float rightPower = power * correctionPercent;
	float leftPower = power;

	if (rightPower > 127) {
		rightPower = 127;
		leftPower = 127 / (correctionPercent);
	}

	setDrivePower(autoDrive, driveData.direction*leftPower, driveData.direction*rightPower);
}

void driveStraightEnd() {
	//brake
	setDrivePower(autoDrive, -driveData.direction * driveData.brakePower, -driveData.direction * driveData.brakePower);
	int brakeDelay = limit(0, driveData.brakeDuration, driveData.waitAtEnd);
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

void driveStraight(float distance, bool runAsTask=driveDefaults.runAsTask, float in1=driveDefaults.rampConst1, float in2=driveDefaults.rampConst2, float in3=driveDefaults.rampConst3, float kP=driveDefaults.kP_c, float kI=driveDefaults.kI_c, float kD=driveDefaults.kD_c, correctionType correctionType=driveDefaults.defCorrectionType, bool rawValue=driveDefaults.rawValue, float minSpeed=driveDefaults.minSpeed, int timeout=driveDefaults.timeout, int brakePower=driveDefaults.brakePower, int waitAtEnd=driveDefaults.waitAtEnd, int sampleTime=driveDefaults.sampleTime) { //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
	//initialize variables
	driveData.distance = abs(distance);
	initializeRampHandler(driveData.ramper, distance, in1, in2, in3);
	driveData.direction = sgn(distance);
	driveData.rawValue = rawValue;
	driveData.minSpeed = minSpeed * sampleTime / 1000;
	driveData.timeout = timeout;
	driveData.waitAtEnd = waitAtEnd;
	driveData.sampleTime = sampleTime;
	driveData.brakeDuration = driveDefaults.brakeDuration;
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
//#endregion
