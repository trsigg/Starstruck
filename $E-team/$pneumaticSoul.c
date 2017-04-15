#pragma config(Sensor, in1,    hyro,           sensorGyro)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in5,    sidePot,        sensorPotentiometer)
#pragma config(Sensor, in6,    modePot,        sensorPotentiometer)
#pragma config(Sensor, in7,    clawPotL,       sensorPotentiometer)
#pragma config(Sensor, in8,    clawPotR,       sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEnc,        sensorQuadEncoder)
#pragma config(Motor,  port1,           rDrive1,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           lift1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           LDrive1,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           LClaw,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           rDrive2,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           lift2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           rClaw,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           LDrive2,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           lift3,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          LDrive3,       tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#region config
	//#subregion user control
//#define DRIVER_PID	//uncommented if using PID instead of still speeds during user control
	//#endsubregion
	//#subregion auton
#define straightToCube true	//whether initialPillow() drives straight to cube
#define agressiveClose false	//determines the point at which the claw begins to close in initialPillow()
#define skills true	//whether autonomous mode runs skills routine
	//#endsubregion
	//#subregion tuning
#define TUNING	//uncommented to use tuning function
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

//#region positions
enum liftState { BOTTOM, MIDDLE, TOP, THROW, MAX };
enum clawState { CLOSED, OPEN, HYPEREXTENDED };

int liftPositions[5] = { 1050, 1860, 2340, 2160, 2820 };	//same order as corresponding enums
int clawPositions[3] = { 350, 1285, 1900 };
//#endregion

#ifdef TUNING
	int targets[4] = { liftPositions[MIDDLE], clawPositions[OPEN], 0, 0 };	//lift, claw, turn, drive
	bool abortDrive = false;	//set to true to stop drive maneuver
	bool abortTuning = false;
#endif

//#region constants
#define liftStillSpeed 15
#define clawDefPower 80	//power used in manual control
#define liftErrorMargin 150	//margins of error
#define clawErrorMargin 100
#define maxStationarySpeed	100	//max error decrease in claw PID error (per second) where claw is considered not to be moving (CURRENTLY UNUSED)
#define fenceToWallDist 30
#define clawDiff 120					//difference between claw potentiometers when at the same angle (left - right)
#define liftDriftDist	300	//estimated distance lift drifts after button is released (for setting lift PID target during drive control)
//#endregion

//#region timers
#define autonTimer T1
#define movementTimer T2
//#region

//#region globals
bool autoDumping = false;
clawState currentState;
int liftDirection;
bool autonVariant = false; //whether using standard or variant auton routine

motorGroup lift;
motorGroup rightClaw;
motorGroup leftClaw;
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeAutoMovement();

	//configure drive
	initializeDrive(drive, true);
	setDriveMotors(drive, 6, LDrive1, LDrive2, LDrive3, rDrive1, rDrive2, rDrive2);
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
		setTargetingPIDconsts(lift, 0.4, 0.001, 0.4, 25);
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

void setClawTargets(int targetPos) {
	setTargetPosition(leftClaw, targetPos+clawDiff);
	setTargetPosition(rightClaw, targetPos);
}

void setClawState(clawState state) {
	setClawTargets(clawPositions[state]);
	currentState = state;
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

		if (lift.posPID.target==liftPositions[BOTTOM] && errorLessThan(lift, liftErrorMargin))
			setPower(lift, -liftStillSpeed);
		else
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

void ramToRealign(int duration=500) {
	setDrivePower(drive, -127, -127); //realign using wall
	wait1Msec(duration);
	setDrivePower(drive, 0, 0);
}

#ifdef TUNING
void tuning() {
	while (!abortTuning) {
		if (targets[0] != lift.posPID.target)
			setTargetPosition(lift, targets[0]);

		if (targets[1] != rightClaw.posPID.target)
			setClawTargets(targets[1]);

		if (abortDrive) {
			abortDrive = false;
			driveData.isDriving = false;
			turnData.isTurning = false;
			setDrivePower(drive, 0, 0);
		}

		if (!(driveData.isDriving || turnData.isTurning)) {
			if (targets[2] != 0) {
				turn(targets[2], true);
				targets[2] = 0;
			} else if (targets[3] != 0) {
				driveStraight(targets[3], true);
				targets[3] = 0;
			}
		}
	}
}
#endif

void initialPillow() {
	setLiftState(BOTTOM);

	if (agressiveClose)
		setClawState(OPEN);

	if (straightToCube) {
		driveStraight(26, true);
		while (driveData.totalDist < 15);
		setClawState(OPEN);
		while (driveData.isDriving);
	} else {
		//open claw and drive away from wall
		driveStraight(6, true);
		while (driveData.isDriving) EndTimeSlice();

		//drive to central pillow
		turn(-47, true);
		waitForMovementToFinish();
		driveStraight(17);
	}

	moveClawTo(CLOSED, 500); //clamp pillow
}

void initialSide(bool preloadFirst) {	//dumps preload and corner jack first if preloadFirst is true
  //back up
  driveStraight(-35, true);
  setLiftState(MIDDLE);
  setClawTargets(clawPositions[OPEN] - 350);
  waitForMovementToFinish(false);

  //get corner jacks
  setLiftState(BOTTOM);
  turn(-20);
  driveStraight(26);
  waitForMovementToFinish();
  moveClawTo(CLOSED);
  while (getPosition(rightClaw) > 600);

  //dump
  setLiftState(MIDDLE);
  driveStraight(-15);
  turnDriveDump(17, -25, 7);
}

task skillz() {
	driveStraight(-13);
	setClawState(OPEN);
	liftTo(MIDDLE);
	liftTo(BOTTOM);

	grabNdump(3000);

	for (int i=0; i<2; i++) { //dump preload pillows
		setLiftState(BOTTOM);
		ramToRealign();

		driveStraight(fenceToWallDist);

		grabNdump(500);
	}

	setLiftState(BOTTOM);
	ramToRealign();

	//get and dump front center jacks
	setTargetPosition(lift, liftPositions[MIDDLE]+100);	//TODO
	setClawTargets(clawPositions[OPEN]-100);
	ramToRealign();
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
	ramToRealign();
	setLiftState(BOTTOM);
	waitForMovementToFinish();
	driveStraight(13);
	moveClawTo(CLOSED); //grab pillow
	turnDriveDump(0, -15);

	//grab and dump center back jacks
	setClawState(HYPEREXTENDED);
	setLiftState(BOTTOM);
	ramToRealign();
	driveStraight(fenceToWallDist+3.5, true);
	waitForMovementToFinish();
	grabNdump(0, fenceToWallDist+10, 750);

	//get and dump right side pillow
	setLiftState(BOTTOM);
	ramToRealign();
	driveStraight(3);
	turn(-25, true);
	waitForMovementToFinish();
	driveStraight(50);
	moveClawTo(CLOSED);
	turnDriveDump(25, -fenceToWallDist, 10);

	//get first right side jack
	setLiftState(MIDDLE);
	ramToRealign();
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
		setLiftState(BOTTOM);
		ramToRealign();
		driveStraight(fenceToWallDist + i);
		grabNdump(500);
	}
}

task blockingAuton() {	//variant blocks for nearly entire autonomous period
	clearTimer(autonTimer);
	initialPillow();

	//go to fence and lift up
	setLiftState(TOP);
	driveStraight(12, false, 40, 95, -30);
	wait1Msec(100);
	turn(40, false, 40, 100, -30); //turn to face fence
	driveStraight(30, false, 60, 127, -20); // drive up to wall
	waitForMovementToFinish();

	if (autonVariant) {
		setDrivePower(drive, 15, 15);
		while (time1(autonTimer) < 14000) EndTimeSlice();
	}

	moveClawTo(OPEN); //release pillow
	wait1Msec(500); //wait for pillow to fall
	setClawState(CLOSED);
	driveStraight(-7, true); //back up
	while (driveData.totalDist < 5);
	setClawState(HYPEREXTENDED);
	waitForMovementToFinish();

	//push jacks over
 	driveStraight(10);
 	moveClawTo(OPEN);

 	setClawState(HYPEREXTENDED);

 	driveStraight(-5);
 	turn(135);
 	liftTo(BOTTOM);
 	driveStraight(fenceToWallDist + 2);
 	grabNdump(0);

 	liftTo(BOTTOM);
}

task dumpyAuton() {	//variant dumps to side
	initialPillow();

	liftTo(MIDDLE);
	driveStraight(11, false, 40, 95, -30);

	turnDriveDump((autonVariant ? -70 : -95), -24, 7, 45, 100, -20);
	if (autonVariant) {
		setLiftState(BOTTOM);
		driveStraight(24);
		turn(-10);
	} else {
		setLiftState(BOTTOM);
		ramToRealign();
	}

	setClawState(HYPEREXTENDED);
	waitForMovementToFinish();
	driveStraight(autonVariant ? 15 : fenceToWallDist+6);
	grabNdump(0, fenceToWallDist, 750);
	setLiftState(BOTTOM);
	ramToRealign();
	driveStraight(fenceToWallDist);
	grabNdump(0, fenceToWallDist, 750);

	liftTo(BOTTOM);
}

task oneSideAuton() {	//variant doesn't get center back jacks
  initialSide(false);

  if (!autonVariant) {
    //drive to back
    setClawTargets(clawPositions[OPEN] - 300);
    setLiftState(MIDDLE);
    ramToRealign();
    driveStraight(fenceToWallDist + 15);

    //get and dump back center jacks
    turn(50);
    setLiftState(BOTTOM);
    driveStraight(-4);
    waitForMovementToFinish();
    driveStraight(45);
    moveClawTo(CLOSED);
    while (getPosition(rightClaw) > 600);
    setLiftState(MIDDLE);
    driveStraight(-40);
    turnDriveDump(-40, -fenceToWallDist-15, 13);
  }
}

task fatAngel() {	//no variant
  initialSide(true);

  //center cube
  setLiftState(BOTTOM);
  turn(35);
  waitForMovementToFinish();
  driveStraight(30);
  moveClawTo(CLOSED);
  liftTo(MIDDLE);
  //driveStraight(5, false, 40, 100, -20);
  turnDriveDump(-43, -20, 5);

  //center back jacks
  setClawState(HYPEREXTENDED);
  liftTo(BOTTOM);
  driveStraight(fenceToWallDist + 5);
  moveClawTo(CLOSED);
  turnDriveDump(0, -fenceToWallDist-5, 1);
}

task autonomous() {
	inactivateTargets();
	setLiftPIDmode(true);
	startTask(maneuvers);

	int sidePos = SensorValue[sidePot];
	int modePos = SensorValue[modePot];

	turnDefaults.reversed = sidePos <= 2585;
	autonVariant = 400<sidePos && sidePos<3600;	//true if in upper half of potentiometer
	clearTimer(T4);	//debug

	//start appropriate autonomous task
	if (skills) {
		startTask(skillz);
	} else	if (1500>sidePos || sidePos>2200) {
		if (modePos < 450) {
			startTask(blockingAuton);
		} else if (modePos < 1970) {
			startTask(dumpyAuton);
		} else if (modePos < 3645) {
			startTask(oneSideAuton);
		} else {
			startTask(fatAngel);
		}
	}

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
