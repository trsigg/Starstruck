#pragma config(Sensor, in2,    hyro,           sensorGyro)
#pragma config(Sensor, in3,    clawPot,        sensorPotentiometer)
#pragma config(Sensor, in4,    wristPot,       sensorPotentiometer)
#pragma config(Sensor, in5,    shoulderPot,    sensorPotentiometer)
#pragma config(Sensor, in6,    modePot,        sensorPotentiometer)
#pragma config(Sensor, in7,    sidePot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Motor,  port1,           lbd,           tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           lfd,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           shoulder1,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           shoulder2,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           rfd,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rbd,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           wristMotor,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           shoulder3,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           clawMotor,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          wheelieMotor,  tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

//#region includes
#include "..\Includes\buttonTracker.c"
#include "..\Includes\motorGroup.c"
#include "..\Includes\parallelDrive.c"
#include "..\Includes\pd_autoMove.c"
//#endregion

//#region buttons
#define openClawBtn Btn6U //claw
#define closeClawBtn Btn6D
#define wheelieOutBtn Btn7U //wheelie bars
#define wheelieInBtn Btn7D
#define toggleLiftModeBtn Btn8L //lift
#define liftUpBtn Btn5U //         -- shoulder/4-bar
#define liftDownBtn Btn5D
#define wristUpBtn Btn8U //        -- wrist
#define wristDownBtn Btn8D
//#endregion

//#region positions
#define shoulderBottom 320 //shoulder
#define shoulderTop 1300
#define shoulderMiddle 460
#define shoulderVert 2040
#define wristMax 2088 //wrist
#define wristMin 1150
#define clawOpenPos 2890 //claw
#define clawClosedPos 3475
#define clawMax 2130
//#endregion

//#region constants
//lift
#define fourBarDeadband 100 //allowable deviation from totalTargetPos
#define shoulderFBUpPower 100 //four bar shoulder powers
#define shoulderFBDownPower 60
#define shoulderStillSpeed 10 //still speeds
#define wristStillSpeed 10
#define clawStillSpeed 15
//#endregion

//#region globals
bool fourBar = false;
bool clawOpen = false;
int totalTargetPos; //the target sum of the shoulder and wrist pot values in 4-bar mode
short autoSign; //for autonomous, positive if robot is left of pillow
short clawCounter, driveCounter, liftCounter; //store the progress of each robot component during autonomous

motorGroup shoulder;
motorGroup wrist;
motorGroup claw;
motorGroup wheelieWinch;
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;

	//configure drive
	initializeDrive(drive);
  setDriveMotors(drive, 4, lfd, lbd, rfd, rbd);
  attachEncoder(drive, leftEnc, LEFT);
  attachEncoder(drive, rightEnc, RIGHT, false, 4);
  attachGyro(drive, hyro);

  //configure shoulder
	initializeGroup(shoulder, 3, shoulder1, shoulder2, shoulder3);
  configureButtonInput(shoulder, liftUpBtn, liftDownBtn, shoulderStillSpeed);
  addSensor(shoulder, shoulderPot);

  //configure wrist
	initializeGroup(wrist, 1, wristMotor);
	configureButtonInput(wrist, wristUpBtn, wristDownBtn);
	addSensor(wrist, wristPot);
	setAbsolutes(wrist, wristMin, wristMax);

	//configure wheelie winch
	initializeGroup(wheelieWinch, 1, wheelieMotor);
	configureButtonInput(wheelieWinch, wheelieInBtn, wheelieOutBtn);

	//configure claw
  initializeGroup(claw, 1, clawMotor);
  addSensor(claw, clawPot);
}

//#region claw
void clawControl() {
	if (vexRT[closeClawBtn] == 1) {
		setPower(claw, 127);
		clawOpen = false;
	} else if (vexRT[openClawBtn] == 1) {
		setPower(claw, -127);
		clawOpen = true;
	} else {
		setPower(claw, clawStillSpeed * (clawOpen ? -1 : 1));
	}
}
//#endregion

//#region lift
int totalLiftPotVal() {
	return potentiometerVal(wrist) + potentiometerVal(shoulder);
}

void toggleLiftMode() {
	fourBar = !fourBar;

	if (fourBar) {
		totalTargetPos = totalLiftPotVal();
		shoulder.upPower = shoulderFBUpPower;
		shoulder.downPower = -shoulderFBDownPower;
	} else {
		shoulder.upPower = 127;
		shoulder.downPower = -127;
	}
}

void liftControl() {
	if (newlyPressed(toggleLiftModeBtn))
		toggleLiftMode();

	short shoulderPos = potentiometerVal(shoulder);

	shoulder.stillSpeed = shoulderStillSpeed * ((shoulderPos<shoulderMiddle || shoulderPos>shoulderVert) ? -1 : 1);
	takeInput(shoulder);
	int wristPower = takeInput(wrist, false);

	int totalPos = totalLiftPotVal();

	if (wristPower != 0) {
		setPower(wrist, wristPower);
		totalTargetPos = totalPos;
	}	else if (fourBar && (abs(totalTargetPos - totalPos) > fourBarDeadband)) { //only moves toward target if in four bar mode, outside of deadband, and doesn't move past bounds
		moveTowardPosition(wrist, totalTargetPos - shoulderPos);
	} else {
		setPower(wrist, wristStillSpeed);
	}
}
//#endregion

//#region autonomous
task maneuvers() {
	while (true) {
		executeManeuver(claw);
		executeManeuver(shoulder);
		executeManeuver(wrist);
	}
}

void setClawStateManeuver(bool open) { //toggles by default
	if (open) {
		createManeuver(claw, clawOpenPos, -clawStillSpeed);
	} else {
		createManeuver(claw, clawClosedPos, clawStillSpeed);
	}

	clawOpen = open;
}

void openClaw(bool stillSpeed=true) {
	goToPosition(claw, clawOpenPos, (stillSpeed ? -clawStillSpeed : 0));
}

void closeClaw(bool stillSpeed=true) {
	goToPosition(claw, clawClosedPos, (stillSpeed ? clawStillSpeed : 0));
}

void hyperExtendClaw(bool stillSpeed=true) {
	goToPosition(claw, clawMax, (stillSpeed ? -clawStillSpeed : 0));
}

void setShoulderStateManeuver(bool top = potentiometerVal(shoulder)<shoulderMiddle) { //toggles by default
  if (top) {
    createManeuver(shoulder, shoulderTop, shoulderStillSpeed);
  } else {
    createManeuver(shoulder, shoulderBottom, shoulderStillSpeed);
  }
}

task skillz() {
	setClawStateManeuver(true);
	setPower(wheelieWinch, 127); //deploy bars
	driveStraight(-20, true); //drive back
	wait1Msec(500);
	setPower(wheelieWinch, 0);
	while (driveData.isDriving || claw.maneuverExecuting);
	wait1Msec(1000);
	closeClaw();
}

task pillowAuton() {
	setClawStateManeuver(true); //open claw
	createManeuver(shoulder, 350, shoulderStillSpeed, 35);
	setPower(wheelieWinch, 127); //start deploying wheelie bars
  driveStraight(5, true); //drive away from wall
  while(driveData.isDriving);
  setPower(wheelieWinch, 0);

  //move toward pillow
  turn(-50, true);
  while(turnData.isTurning || claw.maneuverExecuting || shoulder.maneuverExecuting);
  driveStraight(20);

  closeClaw(); //clamp pillow

  setShoulderStateManeuver(true);
  //goToPosition(wrist, 1200, wristStillSpeed);
  driveStraight(24, true);
  while (driveData.isDriving);
  turn(60, true, 40, 80, -10); //turn to face fence
  while (turnData.isTurning);
  driveStraight(40, true); // drive up to wall
  while (driveData.isDriving || shoulder.maneuverExecuting);

  openClaw(); //release pillow
  wait1Msec(600); //wait for pillow to fall
  closeClaw();
  driveStraight(-10); //back up
  hyperExtendClaw();

  //push jacks over
 	driveStraight(10);
 	closeClaw();

 	createManeuver(claw, clawMax, -clawStillSpeed);

 	//drive to other wall and lift down
 	driveStraight(-15, true);
 	while (driveData.isDriving);
 	turn(75, true, 40, 127, -20);
 	while (turnData.isTurning || claw.maneuverExecuting);
 	createManeuver(shoulder, 985, shoulderStillSpeed);
 	driveStraight(55, true);
 	while (driveData.isDriving || shoulder.maneuverExecuting);
 	turn(-80, false, 40, 120, -40);
 	driveStraight(18);

 	goToPosition(shoulder, 1466); //push jacks over
 	driveStraight(11);
 	closeClaw();
}

task oneSideAuton() {
	createManeuver(claw, clawMax, -clawStillSpeed); //open claw
	createManeuver(shoulder, shoulderTop-400, shoulderStillSpeed); //lift to near top
	setPower(wheelieWinch, 127);
  driveStraight(5, true); //drive away from wall
  while(driveData.isDriving);
  setPower(wheelieWinch, 0);

  turn(30, true);
  while (turnData.isTurning || claw.maneuverExecuting);

  driveStraight(18);
  while (driveData.isDriving);

  turn(-28, true); //turn toward wall
  while (turnData.isTurning || shoulder.maneuverExecuting);

  driveStraight(38);
  goToPosition(shoulder, 1466);
}

task autonomous() {
	shoulder.maneuverExecuting = false;
	claw.maneuverExecuting = false;
	startTask(maneuvers);

  autoSign = (SensorValue[sidePot] < 1800) ? 1 : -1;

  //start appropriate autonomous task
  if (SensorValue[sidePot] > 1575) {
  	startTask(skillz);
  } else if (SensorValue[modePot] > 2670) {
  	startTask(pillowAuton);
  } else if (SensorValue[modePot] > 1275) {
  	startTask(oneSideAuton);
  }
}
//#endregion

task usercontrol() {
	while (true) {
  	driveRuntime(drive);

  	takeInput(wheelieWinch);

		clawControl();

		liftControl();
  }
}
