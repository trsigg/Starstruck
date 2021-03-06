#pragma config(Sensor, in1,    hyro,           sensorGyro)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    clawPotR,       sensorPotentiometer)
#pragma config(Sensor, in4,    clawPotL,       sensorPotentiometer)
#pragma config(Sensor, in5,    sidePot,        sensorPotentiometer)
#pragma config(Sensor, in6,    modePot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEnc,        sensorQuadEncoder)
#pragma config(Motor,  port1,           rbd,           tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rfd,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           lift1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           lift2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           lift3,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           clawMotor,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           lift4,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           lift5,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           lfd,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          lbd,           tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\Includes\pd_autoMove.c"
//#endregion

//#region buttons
#define autoDumpOnBtn			Btn8U	//claw
#define autoDumpOffBtn		Btn8D
#define openClawBtn				Btn6U
#define closeClawBtn			Btn6D
#define hyperExtendBtn		Btn7D
#define liftUpBtn					Btn5U //lift
#define liftDownBtn				Btn5D
//#endregion

//#region enums
enum liftState { BOTTOM, MIDDLE, TOP, THROW, MAX };
enum clawState { CLOSED, OPEN, HYPEREXTENDED };
//#endregion

//#region positions
int liftPositions[5] = { 1000, 1650, 2425, 2425, 2950 };	//same order as corresponding enums
int clawPositions[3] = { 400, 1150, 2000 };
//#endregion

//#region constants
#define liftStillSpeed 10	//still speeds
#define liftErrorMargin 150	//margins of error
#define clawErrorMargin 150
#define maxStationarySpeed	100	//max error decrease in claw PID error (per second) where claw is considered not to be moving
#define fenceToWallDist 28	//distances
#define clawDiff 0					//difference between claw potentiometers when at the same angle (left - right)
#define liftDriftDist	300	//estimated distance lift drifts after button is released
//#endregion

//#region config
#define straightToCube true
#define blocking false
#define driverPID false
//#endregion

//#region timers
#define autonTimer T1
#define movementTimer T2
//#region

//#region globals
bool autoDumping = true;
int autoSign; //for autonomous, positive if robot is left of pillow
clawState currentState;

motorGroup lift;
motorGroup rightClaw;
motorGroup leftClaw;
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeAutoMovement();

	turnDefaults.rampConst1 = 60;
	turnDefaults.rampConst2 = 127;
	turnDefaults.rampConst3 = -10;

	driveDefaults.rampConst1 = 60;
	driveDefaults.rampConst2 = 127;
	driveDefaults.rampConst3 = -20;

	//configure drive
	initializeDrive(drive);
	setDriveMotors(drive, 4, lfd, lbd, rfd, rbd);
	attachEncoder(drive, leftEnc, LEFT);
	attachEncoder(drive, rightEnc, RIGHT, false, 2.75);
	attachGyro(drive, hyro);

	//configure lift
	initializeGroup(lift, 3, lift1, lift2, lift3);
	configureButtonInput(lift, liftUpBtn, liftDownBtn, liftStillSpeed);
	addSensor(lift, liftPot);

	//configure claw sides
	initializeGroup(rightClaw, 1, clawMotor);
	setTargetingPIDconsts(rightClaw, 0.2, 0, 0.7, 25);
	addSensor(rightClaw, clawPotR);

	initializeGroup(leftClaw, 1, clawMotor);
	setTargetingPIDconsts(leftClaw, 0.2, 0, 0.7, 25);
	addSensor(leftClaw, clawPotL);
}

void inactivateTargets() {
	stopTargeting(lift);
	stopTargeting(rightClaw);
	stopTargeting(leftClaw);
}

//#region lift
void setLiftState(liftState state) {
	setTargetPosition(lift, liftPositions[state]);
}

void setLiftPIDmode(bool auto) {
	if (auto)
		setTargetingPIDconsts(lift, 0.9, 0.005, 50, 25);
	else
		setTargetingPIDconsts(lift, 0.2, 0.0001, 5, 25);
}

void liftControl() {
	if (driverPID) {
		lift.stillSpeed = liftStillSpeed * (getPosition(lift)<liftPositions[MIDDLE] ? -1 : 1);

		takeInput(lift);
	} else {
		if (vexRT[liftUpBtn] == 1) {
			setPower(lift, 127);
			setTargetPosition(lift, limit(getPosition(lift)+liftDriftDist, liftPositions[BOTTOM], liftPositions[MAX]));
		} else if (vexRT[liftDownBtn] == 1) {
			setPower(lift, -127);
			setTargetPosition(lift, limit(getPosition(lift)-liftDriftDist, liftPositions[BOTTOM], liftPositions[MAX]));
		} else {
			maintainTargetPos(lift);
		}
	}
}
//#endregion

//#region claw
void executeClawPIDs() {
	maintainTargetPos(leftClaw);
	maintainTargetPos(rightClaw);
}

void setClawState(clawState state) {
	setTargetPosition(leftClaw, clawPositions[state]+clawDiff);
	setTargetPosition(rightClaw, clawPositions[state]);
	currentState = state;
}

void setClawTargets(int targetPos) {
	setTargetPosition(leftClaw, targetPos+clawDiff);
	setTargetPosition(rightClaw, targetPos);
}

void clawControl() {
	if (vexRT[openClawBtn]==1 && currentState!=OPEN)
		setClawState(OPEN);
	else if (vexRT[closeClawBtn]==1 && currentState!=CLOSED)
		setClawState(CLOSED);
	else if (vexRT[hyperExtendBtn]==1 && currentState!=HYPEREXTENDED)
		setClawState(HYPEREXTENDED);
	else if (getPosition(lift)>liftPositions[THROW] && currentState!=OPEN && autoDumping)
		setClawState(OPEN);

	executeClawPIDs();

	if (vexRT[autoDumpOnBtn] == 1)
		autoDumping = true;
	else if (vexRT[autoDumpOffBtn] == 1)
		autoDumping = false;
}
//#endregion

//#region autonomous
task maneuvers() {
	while (true) {
		executeClawPIDs();

		maintainTargetPos(lift);
	}
}

bool clawNotMoving(motorGroup *claw, int maxSpeed=maxStationarySpeed) {
	float speed = (abs(claw->posPID.target - getPosition(claw)) - abs(claw->posPID.prevError)) * 1000 / claw->posPID.minSampleTime;
	return speed < maxSpeed;
}

bool clawIsClosed() {
	return getPosition(rightClaw)<clawPositions[OPEN] && clawNotMoving(rightClaw, maxStationarySpeed)
					&& getPosition(leftClaw)<clawPositions[OPEN]+clawDiff && clawNotMoving(leftClaw, maxStationarySpeed);
}

bool liftNotReady() { return errorLessThan(lift, liftErrorMargin); }

bool clawNotReady() {
	if (rightClaw.posPID.target < clawPositions[OPEN])	//assuming this is representative of both claw sides
		return !clawIsClosed();
	else
		return !(errorLessThan(rightClaw, clawErrorMargin) || errorLessThan(leftClaw, clawErrorMargin));
}

void waitForMovementToFinish(bool waitForClaw=true, bool waitForLift=true, bool waitForDrive=true, int timeout=75) {
	clearTimer(movementTimer);

	while (time1(movementTimer) < timeout)
		if ((liftNotReady() && waitForLift) || (clawNotReady() && waitForClaw))
			clearTimer(movementTimer);

	while (turnData.isTurning || driveData.isDriving);
}

void liftTo(liftState state, int timeout=75) {
	setLiftState(state);
	clearTimer(movementTimer);

	while (time1(movementTimer) < timeout)
		if (liftNotReady())
			clearTimer(movementTimer);
}

void moveClawTo(clawState state, int timeout=75) {
	setClawState(state);
	clearTimer(movementTimer);

	while (time1(movementTimer) < timeout)
		if (clawNotReady())
			clearTimer(movementTimer);
}

void turnDriveDump (int angle, int dist, int distCutoff=0, float turnConst1=turnDefaults.rampConst1, float turnConst2=turnDefaults.rampConst2, float turnConst3=turnDefaults.rampConst3) {
	if (angle != 0) { //turning
		if (dist != 0) { //turning & driving
			if (lift.posPID.target < liftPositions[MIDDLE])
				liftTo(MIDDLE); //lift up so claw doesn't drag on ground
			turn(angle, false, turnConst1, turnConst2, turnConst3); //turn
		} else { //turning but not driving
			turn(angle, true, turnConst1, turnConst2, turnConst3); //turn
			while (abs(turnProgress()) < distCutoff); //wait to throw
		}
	}

	if (dist != 0) { //driving
		driveStraight(dist, true); //drive
		while (driveData.totalDist < distCutoff); //wait to throw
	}

	setLiftState(MAX);
	while (getPosition(lift) < liftPositions[THROW]);
	setClawState(OPEN);
	waitForMovementToFinish();
}

void grabNdump(int delayDuration, int dist=fenceToWallDist, int closeTimeout=500) {
	wait1Msec(delayDuration); //wait for objects to be dropped
	moveClawTo(CLOSED, closeTimeout);
	turnDriveDump(0, -dist); //dump pillow
}

void driveToWall(int distance=fenceToWallDist) {
	liftTo(BOTTOM);
	driveStraight(distance);
}

void ramToRealign(int duration=500) {
	liftTo(BOTTOM);

	setDrivePower(drive, -127, -127); //realign using wall
	wait1Msec(duration);
}

void initialPillow(bool liftToMid=false) {
	liftTo(MIDDLE);
	if (liftToMid)
		setTargetPosition(lift, liftPositions[MIDDLE]-65);
	else
		liftTo(BOTTOM);

	if (straightToCube) {
		driveStraight(18);
	} else {
		//open claw and drive away from wall
		setClawState(OPEN);
		driveStraight(5, true);
		while (driveData.isDriving);

		//drive to central pillow
		turn(autoSign * -47, true);
		waitForMovementToFinish();
		driveStraight(14);
	}

	moveClawTo(CLOSED, 100); //clamp pillow
}

task skillz() {
	setClawState(OPEN);
	driveStraight(-11, true);
	waitForMovementToFinish();

	grabNdump(3000);

	for (int i=0; i<2; i++) { //dump preload pillows
		ramToRealign();

		driveStraight(fenceToWallDist);

		grabNdump(500);
	}

	ramToRealign();

	//get and dump front center jacks
	setTargetPosition(lift, liftPositions[MIDDLE]+10);
	setClawTargets(clawPositions[CLOSED]-75);
	driveStraight(4.75, true);
	waitForMovementToFinish(false);
	turn(-80, true, 60, 127, -10);
	waitForMovementToFinish();
	liftTo(BOTTOM);
	driveStraight(29);
	moveClawTo(CLOSED);
	driveStraight(-6);
	turnDriveDump(67, 0);

	//get and dump pillow in center of field
	setLiftState(BOTTOM);
	driveStraight(-5, true);	//to align to central pillow
	waitForMovementToFinish();
	driveStraight(10);
	moveClawTo(CLOSED); //grab pillow
	turnDriveDump(0, -10);

	//grab and dump center back jacks
	driveStraight(fenceToWallDist+1.5, true);
	setClawState(HYPEREXTENDED);
	setLiftState(BOTTOM);
	waitForMovementToFinish();
	grabNdump(0, 30, 750);

	//get and dump pillow
	setLiftState(BOTTOM);
	turn(-13, true);
	waitForMovementToFinish();
	driveStraight(29);
	moveClawTo(CLOSED);
	turnDriveDump(40, -30, 10);

	ramToRealign();
	driveStraight(fenceToWallDist);
	grabNdump(500);

	ramToRealign();
	driveStraight(fenceToWallDist+3);
	grabNdump(500, 35);

	liftTo(BOTTOM);
}

task pillowAuton() {
	clearTimer(autonTimer);
	initialPillow(true);

	//go to fence and lift up
	setLiftState(TOP);
	driveStraight(8.5, true);
	while (driveData.isDriving);
	turn(autoSign * 37, true, 60, 127, -15); //turn to face fence
	while (turnData.isTurning);
	driveStraight(20, true); // drive up to wall
	waitForMovementToFinish();

	if (blocking)
		while (time1(autonTimer) < 12000);
	moveClawTo(OPEN); //release pillow
	wait1Msec(750); //wait for pillow to fall
	moveClawTo(CLOSED);
	driveStraight(-6); //back up
	moveClawTo(HYPEREXTENDED);

	//push jacks over
 	driveStraight(8);
 	moveClawTo(CLOSED);

 	setClawState(HYPEREXTENDED);

 	//drive to other wall and lift down
 	driveStraight(-10, true);
 	while (driveData.isDriving);
 	turn(autoSign * 74, true, 60, 127, -15);
 	waitForMovementToFinish();
 	setTargetPosition(lift, liftPositions[MIDDLE]+100);
 	driveStraight(33, true, driveDefaults.rampConst1, driveDefaults.rampConst2, -10);
 	waitForMovementToFinish();
 	turn(autoSign * -85, false, 40, 120, -40);
 	driveStraight(11);

 	goToPosition(lift, liftPositions[TOP]+50); //push jacks over
 	driveStraight(5);
 	moveClawTo(CLOSED);
}

task dumpyAuton() {
	initialPillow();

	liftTo(MIDDLE);
	driveStraight(8);

	turnDriveDump(autoSign * -97, -17, 7, 45, 120, -20);

	setLiftState(BOTTOM);
	setClawState(HYPEREXTENDED);
	waitForMovementToFinish();
	driveStraight(fenceToWallDist);
	grabNdump(0, 33, 750);
	driveToWall();
	grabNdump(0, 33, 750);

	liftTo(BOTTOM);
}

task oneSideAuton() {
	setTargetPosition(lift, liftPositions[TOP]-450); //lift to near top
	driveStraight(-5, true); //drive away from wall
	while(driveData.isDriving);

	turn(autoSign * 30, true);
	while (turnData.isTurning);

	driveStraight(-12, true);
	waitForMovementToFinish(false);

	turn(autoSign * -30, true); //turn toward wall
	waitForMovementToFinish();

	//knock off jacks
	driveStraight(-25);
	liftTo(MAX);
	moveClawTo(CLOSED);

	//pick up and dump back jacks
	setClawState(OPEN);
	setLiftState(BOTTOM);
	waitForMovementToFinish();
	//turn(autoSign * -3);
	driveStraight(29);
	moveClawTo(CLOSED);
	turnDriveDump(autoSign * 0, -35, 20);
}

task autonomous() {
	inactivateTargets();
	setLiftPIDmode(true);
	startTask(maneuvers);

	int sidePos = SensorValue[sidePot];
	int modePos = SensorValue[modePot];

	autoSign = (sidePos > 1830) ? 1 : -1;

	startTask(dumpyAuton);

	//start appropriate autonomous task
	/*if (1290<sidePos && sidePos<2470) {
		startTask(skillz);
	} else if (modePos < 385) {
		startTask(pillowAuton);
	} else if (modePos < 1765) {
		startTask(dumpyAuton);
	} else if (modePos > 3300) {
		startTask(oneSideAuton);
	}*/
}
//#endregion

task usercontrol() {
	inactivateTargets();
	setLiftPIDmode(false);

	while (true) {
		driveRuntime(drive);

		liftControl();

		clawControl();
	}
}
