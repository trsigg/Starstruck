#pragma config(Sensor, in1,    hyro,           sensorGyro)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in5,    modePot,        sensorPotentiometer)
#pragma config(Sensor, in6,    sidePot,        sensorPotentiometer)
#pragma config(Sensor, in7,    clawPotR,       sensorPotentiometer)
#pragma config(Sensor, in8,    clawPotL,       sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEnc,        sensorQuadEncoder)
#pragma config(Motor,  port1,           LDrive1,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           lift1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           rDrive1,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           rClaw,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           LDrive2,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           LClaw,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           rDrive2,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           lift3,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          rDrive3,       tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#region config
	//#subregion user control
//#define DRIVER_PID	//uncommented if using PID instead of still speeds during user control
	//#endsubregion
	//#subregion auton
#define dumpToSide false
#define straightToCube true
#define blocking false
#define agressiveClose false
	//#endsubregion
//#endregion

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\Includes\pd_autoMove.c"
//#endregion

//#region buttons
	//#subregion claw
#define autoDumpOnBtn			Btn8U
#define autoDumpOffBtn		Btn8D
#define clawForwardBtn		Btn7R	//directly sets claw power to clawDefPower
#define clawBackwardBtn		Btn7L	//directly sets claw power to -clawDefPower
#define clawNeutralBtn		Btn7U	//directly sets claw power to 0
#define openClawBtn				Btn6U
#define closeClawBtn			Btn6D
#define hyperExtendBtn		Btn7D
	//#endsubregion
	//#subregion lift
#define liftUpBtn		Btn5U
#define liftDownBtn	Btn5D
	//#endsubregion
//#endregion

//#region enums
enum liftState { BOTTOM, MIDDLE, TOP, THROW, MAX };
enum clawState { CLOSED, OPEN, HYPEREXTENDED };
//#endregion

//#region positions
int liftPositions[5] = { 1175, 1700, 2400, 2425, 2950 };	//same order as corresponding enums
int clawPositions[3] = { 370, 1200, 2000 };
//#endregion

//#region constants
#define liftStillSpeed 15
#define clawDefPower 80	//power used in manual control
#define liftErrorMargin 150	//margins of error
#define clawErrorMargin 100
#define maxStationarySpeed	100	//max error decrease in claw PID error (per second) where claw is considered not to be moving (CURRENTLY UNUSED)
#define fenceToWallDist 30
#define clawDiff 0					//difference between claw potentiometers when at the same angle (left - right)
#define liftDriftDist	300	//estimated distance lift drifts after button is released (for setting lift PID target during drive control)
//#endregion

//#region timers
#define autonTimer T1
#define movementTimer T2
//#region

//#region globals
bool autoDumping = true;
int autoSign; //for autonomous, positive if robot is left of pillow
clawState currentState;
int liftDirection;

motorGroup lift;
motorGroup rightClaw;
motorGroup leftClaw;
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeAutoMovement();

	turnDefaults.rampConst1 = 40;
	turnDefaults.rampConst2 = 127;
	turnDefaults.rampConst3 = -30;

	driveDefaults.rampConst1 = 50;
	driveDefaults.rampConst2 = 120;
	driveDefaults.rampConst3 = -20;

	//configure drive
	initializeDrive(drive, true);
	setDriveMotors(drive, 6, LDrive1, LDrive2, LDrive2, rDrive1, rDrive2, rDrive2);
	attachEncoder(drive, leftEnc, LEFT);
	attachEncoder(drive, rightEnc, RIGHT, false, 3.25);
	attachGyro(drive, hyro);

	//configure lift
	initializeGroup(lift, 3, lift1, lift2, lift3);
	configureButtonInput(lift, liftUpBtn, liftDownBtn, liftStillSpeed);
	addSensor(lift, liftPot);

	//configure claw sides
	initializeGroup(rightClaw, 1, rClaw);
	setTargetingPIDconsts(rightClaw, 0.2, 0, 0.7/25, 25);
	addSensor(rightClaw, clawPotR);

	initializeGroup(leftClaw, 1, LClaw);
	setTargetingPIDconsts(leftClaw, 0.2, 0, 0.7/25, 25);
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
		setTargetingPIDconsts(lift, 0.4, 0.001, 0.6, 25);
	else
		setTargetingPIDconsts(lift, 0.2, 0.001, 0.2, 25);
}

void liftControl() {
	#ifdef DRIVER_PID
		if (vexRT[liftUpBtn] == 1) {
			setPower(lift, 127);
			setTargetPosition(lift, limit(getPosition(lift)+liftDriftDist, liftPositions[BOTTOM], liftPositions[MAX]));
		} else if (vexRT[liftDownBtn] == 1) {
			setPower(lift, -127);
			setTargetPosition(lift, limit(getPosition(lift)-liftDriftDist, liftPositions[BOTTOM], liftPositions[MAX]));
		} else {
			maintainTargetPos(lift);
		}
	#else
		if (vexRT[liftUpBtn] == 1) {
			setPower(lift, 127);
			liftDirection = 1;
		} else if (vexRT[liftDownBtn] == 1) {
			setPower(lift, -127);
			liftDirection = -1;
		} else {
			setPower(lift, liftDirection * liftStillSpeed);
		}
	#endif
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

void setClawPower(int power, bool setTargets=true) {
	setPower(leftClaw, power);
	setPower(rightClaw, power);

	if (setTargets) setClawTargets(getPosition(rightClaw));
}

void clawControl() {
	if (vexRT[clawNeutralBtn] == 1) {
		setClawPower(0);
	} else if (vexRT[clawForwardBtn] == 1) {
		setClawPower(clawDefPower);
	} else	if (vexRT[clawBackwardBtn] == 1) {
		setClawPower(-clawDefPower);
	} else if (vexRT[openClawBtn]==1 && currentState!=OPEN)
		setClawState(OPEN);
	else if (vexRT[closeClawBtn]==1 && currentState!=CLOSED)
		setClawState(CLOSED);
	else if (vexRT[hyperExtendBtn]==1 && currentState!=HYPEREXTENDED)
		setClawState(HYPEREXTENDED);
	else if (getPosition(lift)>liftPositions[THROW] && currentState!=OPEN && autoDumping)
		setClawState(OPEN);
	else {
		executeClawPIDs();
	}

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

		EndTimeSlice();
	}
}

bool clawNotMoving(motorGroup *claw, int maxSpeed=maxStationarySpeed) {
	float speed = (abs(claw->posPID.target - getPosition(claw)) - abs(claw->posPID.prevError)) * 1000 / time(claw->posPID.lastUpdated);
	return speed < maxSpeed;
}

bool clawIsClosed(int maxSpeed=maxStationarySpeed) {
	return getPosition(rightClaw) < clawPositions[OPEN] /*&& clawNotMoving(rightClaw, maxSpeed)*/
					&& getPosition(leftClaw) < clawPositions[OPEN]+clawDiff /*&& clawNotMoving(leftClaw, maxSpeed)*/;
}

bool liftNotReady() { return !errorLessThan(lift, liftErrorMargin); }

bool clawNotReady() {
	if (rightClaw.posPID.target == clawPositions[CLOSED])	//assuming this is representative of both claw sides
		return !clawIsClosed();
	else
		return !(errorLessThan(rightClaw, clawErrorMargin) || errorLessThan(leftClaw, clawErrorMargin));
}

void waitForMovementToFinish(bool waitForClaw=true, bool waitForLift=true, bool waitForDrive=true, int timeout=75) {
	clearTimer(movementTimer);

	while (time1(movementTimer) < timeout) {
		if ((liftNotReady() && waitForLift) || (clawNotReady() && waitForClaw))
			clearTimer(movementTimer);
		EndTimeSlice();
	}

	while (turnData.isTurning || driveData.isDriving) EndTimeSlice();
}

void liftTo(liftState state, int timeout=75) {
	setLiftState(state);
	clearTimer(movementTimer);

	while (time1(movementTimer) < timeout) {
		if (liftNotReady())
			clearTimer(movementTimer);
		EndTimeSlice();
	}
}

void moveClawTo(clawState state, int timeout=75) {
	setClawState(state);
	clearTimer(movementTimer);

	while (time1(movementTimer) < timeout) {
		if (clawNotReady())
			clearTimer(movementTimer);
		EndTimeSlice();
	}
}

void turnDriveDump (int angle, int dist, int distCutoff=0, float turnConst1=turnDefaults.rampConst1, float turnConst2=turnDefaults.rampConst2, float turnConst3=turnDefaults.rampConst3) {
	if (angle != 0) { //turning
		if (dist != 0) { //turning & driving
			if (lift.posPID.target < liftPositions[MIDDLE])
				liftTo(MIDDLE); //lift up so claw doesn't drag on ground
			turn(angle, false, turnConst1, turnConst2, turnConst3); //turn
		} else { //turning but not driving
			turn(angle, true, turnConst1, turnConst2, turnConst3); //turn
			while (abs(turnProgress()) < distCutoff) EndTimeSlice(); //wait to throw
		}
	}

	if (dist != 0) { //driving
		driveStraight(dist, true); //drive
		while (driveData.totalDist<distCutoff && driveData.isDriving) EndTimeSlice(); //wait to throw
	}

	setLiftState(MAX);
	while (getPosition(lift) < liftPositions[THROW]) EndTimeSlice();
	setClawState(OPEN);
	waitForMovementToFinish();
}

void grabNdump(int delayDuration, int dist=fenceToWallDist, int closeTimeout=500) {
	wait1Msec(delayDuration); //wait for objects to be dropped
	moveClawTo(CLOSED, closeTimeout);
	turnDriveDump(0, -dist); //dump pillow
}

void ramToRealign(int duration=500, bool liftToBottom=true) {
	if (liftToBottom) liftTo(BOTTOM);

	setDrivePower(drive, -127, -127); //realign using wall
	wait1Msec(duration);
	setDrivePower(drive, 0, 0);
}

void initialPillow() {
	setLiftState(BOTTOM);
	if (agressiveClose) setClawState(OPEN);
	if (straightToCube) {
		driveStraight(24);//, true);
		//while (driveData.totalDist < 20);
		setClawState(OPEN);
		while (driveData.isDriving);
	} else {
		//open claw and drive away from wall
		driveStraight(6, true);
		while (driveData.isDriving) EndTimeSlice();

		//drive to central pillow
		turn(autoSign * -47, true);
		waitForMovementToFinish();
		driveStraight(17);
	}

	moveClawTo(CLOSED, 500); //clamp pillow
}

task skillz() {
	driveStraight(-13);
	setClawState(OPEN);
	liftTo(MIDDLE);
	liftTo(BOTTOM);

	grabNdump(3000);

	for (int i=0; i<2; i++) { //dump preload pillows
		ramToRealign();

		driveStraight(fenceToWallDist);

		grabNdump(500);
	}

	ramToRealign();

	//get and dump front center jacks
	setTargetPosition(lift, liftPositions[MIDDLE]+100);
	setClawTargets(clawPositions[OPEN]-100);
	ramToRealign(500, false);
	driveStraight(6, true);
	waitForMovementToFinish(false);
	turn(-58, true, 40, 90, -30);
	waitForMovementToFinish();
	liftTo(BOTTOM);
	driveStraight(42);
	moveClawTo(CLOSED);
	wait1Msec(500);
	setTargetPosition(lift, liftPositions[MIDDLE]+250);
	driveStraight(-6);
	turnDriveDump(68, -5, 0, 40, 95, -30);

	//get and dump pillow in center of field
	setLiftState(MIDDLE);
	ramToRealign(500, false);
	setLiftState(BOTTOM);
	waitForMovementToFinish();
	driveStraight(13);
	moveClawTo(CLOSED); //grab pillow
	turnDriveDump(0, -15);

	//grab and dump center back jacks
	setClawState(HYPEREXTENDED);
	ramToRealign();
	driveStraight(fenceToWallDist+3.5, true);
	waitForMovementToFinish();
	grabNdump(0, fenceToWallDist+10, 750);

	//get and dump right side pillow
	ramToRealign();
	driveStraight(3);
	turn(-25, true);
	waitForMovementToFinish();
	driveStraight(50);
	moveClawTo(CLOSED);
	turnDriveDump(25, -fenceToWallDist, 10);

	//get first right side jack
	ramToRealign();
	setLiftState(MIDDLE);
	driveStraight(5, true);
	waitForMovementToFinish();
	turn(-50);
	liftTo(BOTTOM);
	driveStraight(10);

	//get second side jack
	turn(60);
	driveStraight(fenceToWallDist);
	grabNdump(0);

	//for redundancy
	for (int i=0; i>=0; i+=5) {
		ramToRealign();
		driveStraight(fenceToWallDist + i);
		grabNdump(500);
	}
}

task pillowAuton() {
	clearTimer(autonTimer);
	initialPillow();

	//go to fence and lift up
	setLiftState(TOP);
	driveStraight(8, true, 40, 95, -30);
	while (driveData.isDriving) EndTimeSlice();
	wait1Msec(500);
	turn(autoSign * 49, true, 40, 100, -30); //turn to face fence
	while (turnData.isTurning) EndTimeSlice();
	driveStraight(30, true, 60, 127, -20); // drive up to wall
	waitForMovementToFinish();
	wait1Msec(750);

	if (blocking) {
		setDrivePower(drive, 15, 15);
		while (time1(autonTimer) < 15000) EndTimeSlice();
	}
	moveClawTo(OPEN); //release pillow
	wait1Msec(500); //wait for pillow to fall
	moveClawTo(CLOSED);
	driveStraight(-10.5); //back up
	moveClawTo(HYPEREXTENDED);

	//push jacks over
 	driveStraight(13);
 	moveClawTo(CLOSED);

 	setClawState(HYPEREXTENDED);

 	//drive to other wall
 	setTargetPosition(lift, liftPositions[TOP]+150);
 	driveStraight(-10, true);
 	while (driveData.isDriving) EndTimeSlice();
 	turn(autoSign * 70, true, 60, 127, -20);
 	waitForMovementToFinish();
 	driveStraight(50);
 	turn(autoSign * -40, false, 60, 127, -20);
 	driveStraight(11);

 	moveClawTo(CLOSED);
 	liftTo(MAX);
 	driveStraight(-2);
}

task dumpyAuton() {
	initialPillow();

	liftTo(MIDDLE);
	driveStraight(8.5, false, 40, 95, -20);

	turnDriveDump(autoSign * (dumpToSide ? -70 : -95), -24, 7, 45, 100, -20);
	if (dumpToSide) {
		driveStraight(24);
		turn(autoSign * -12.5);
	} else {
		ramToRealign();
	}

	setLiftState(BOTTOM);
	setClawState(HYPEREXTENDED);
	waitForMovementToFinish();
	driveStraight(fenceToWallDist + (dumpToSide ? -15 : 6));
	grabNdump(0, fenceToWallDist, 750);
	ramToRealign();
	driveStraight(fenceToWallDist);
	grabNdump(0, fenceToWallDist, 750);

	liftTo(BOTTOM);
}

task oneSideAuton() {
	setClawState(HYPEREXTENDED);
	setTargetPosition(lift, liftPositions[TOP]+175);
	driveStraight(57.5);
	moveClawTo(CLOSED);
	wait1Msec(500);
	driveStraight(-10);
	turn(autoSign * 113, true, 50, 127, -20);
	waitForMovementToFinish();

	setLiftState(BOTTOM);
	setClawState(OPEN);
	waitForMovementToFinish();
	driveStraight(fenceToWallDist);
	setClawState(CLOSED);
	while (getPosition(rightClaw) > 600);	//TODO: don't do this
	turnDriveDump(0, -fenceToWallDist-5, fenceToWallDist-20);
}

task autonomous() {
	inactivateTargets();
	setLiftPIDmode(true);
	startTask(maneuvers);

	int sidePos = SensorValue[sidePot];
	int modePos = SensorValue[modePot];

	autoSign = sgn(1580 - sidePos);

	//start appropriate autonomous task
	startTask(skillz);
	/*if (1105<sidePos && sidePos<2295) {
		startTask(skillz);
	} else if (modePos < 590) {
		startTask(pillowAuton);
	} else if (modePos < 1845) {
		startTask(dumpyAuton);
	} else if (modePos < 3475) {
		startTask(oneSideAuton);
	}*/

	while (true) EndTimeSlice();
}
//#endregion

task usercontrol() {
	inactivateTargets();
	setClawPower(0);
	setPower(lift, 0);
	setLiftPIDmode(false);

	while (true) {
		driveRuntime(drive);

		liftControl();

		clawControl();
	}
}