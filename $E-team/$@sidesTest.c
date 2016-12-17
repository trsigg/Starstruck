#pragma config(Sensor, in2,    hyro,           sensorGyro)
#pragma config(Sensor, in3,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in5,    modePot,        sensorPotentiometer)
#pragma config(Sensor, in6,    sidePot,        sensorPotentiometer)
#pragma config(Sensor, in7,    hyro,           sensorNone)
#pragma config(Sensor, in8,    clawPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEnc,        sensorQuadEncoder)
#pragma config(Motor,  port1,           rd1,           tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rd2,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           lift1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           lift2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           lift3,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           clawMotor,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           lift4,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           lift5,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           ld1,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          ld2,           tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

//#region includes
#include "..\Includes\motorGroup.c"
#include "..\Includes\parallelDrive.c"
#include "..\Includes\pd_autoMove.c"
//#endregion

//#region buttons
#define autoDumpOnBtn Btn8U //claw
#define autoDumpOffBtn Btn8D
#define openClawBtn Btn6U
#define closeClawBtn Btn6D
#define liftUpBtn Btn5U //lift
#define liftDownBtn Btn5D
//#endregion

//#region positions
#define liftBottom 1190 //lift
#define liftMiddle 1420
#define liftTop 1700 //1885
#define liftThrowPos 2260
#define liftMax 2550 //2700
#define clawClosedPos 1450 //claw
#define clawOpenPos 1740 //1800
#define clawMax 2150 //2400
//#endregion

//#region constants
#define liftStillSpeed 10 //still speeds
#define clawStillSpeed 15
//#endregion

//#region globals
bool clawOpen = true;
bool autoDumping = true;
short autoSign; //for autonomous, positive if robot is left of pillow

motorGroup lift;
motorGroup claw;
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;

	//configure drive
	initializeDrive(drive);
	setDriveMotors(drive, 4, ld1, ld2, rd1, rd2);
	attachEncoder(drive, leftEnc, LEFT);
	attachEncoder(drive, rightEnc, RIGHT, false, 3.25);
	attachGyro(drive, hyro);

	//configure lift
	initializeGroup(lift, 5, lift1, lift2, lift3, lift4, lift5);
	configureButtonInput(lift, liftUpBtn, liftDownBtn, liftStillSpeed);
	addSensor(lift, liftPot);

	//configure claw
	initializeGroup(claw, 1, clawMotor);
	addSensor(claw, clawPot);
}

//#region lift
void liftControl() {
	lift.stillSpeed = liftStillSpeed * (potentiometerVal(lift)<liftMiddle ? -1 : 1);

	takeInput(lift);
}
//#endregion

//#region claw
void clawControl() {
	if (potentiometerVal(lift)>liftThrowPos && potentiometerVal(claw)<clawOpenPos && autoDumping) {
		setPower(claw, 127);
		clawOpen = true;
	} else	if (vexRT[openClawBtn] == 1) {
		setPower(claw, 127);
		clawOpen = true;
	} else if (vexRT[closeClawBtn] == 1) {
		setPower(claw, -127);
		clawOpen = false;
	} else {
		setPower(claw, clawStillSpeed * (clawOpen ? 1 : -1));
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
		executeManeuver(claw);
		executeManeuver(lift);
	}
}

void setClawStateManeuver(bool open, int power=127) { //toggles by default
	if (open) {
		createManeuver(claw, clawOpenPos, clawStillSpeed, power);
	} else {
		createManeuver(claw, clawClosedPos, -clawStillSpeed, power);
	}

	clawOpen = open;
}

void openClaw(bool stillSpeed=true) {
	goToPosition(claw, clawOpenPos, (stillSpeed ? clawStillSpeed : 0));

	clawOpen = true;
}

void closeClaw(bool stillSpeed=true) {
	goToPosition(claw, clawClosedPos, (stillSpeed ? -clawStillSpeed : 0));

	clawOpen = false;
}

void hyperExtendClaw(bool stillSpeed=true) {
	goToPosition(claw, clawMax, (stillSpeed ? clawStillSpeed : 0));

	clawOpen = true;
}

void setLiftStateManeuver(bool top) {
	if (top) {
		createManeuver(lift, liftTop, liftStillSpeed);
	} else {
		createManeuver(lift, liftBottom, liftStillSpeed);
	}
}

void grabNdump(int delayDuration) {
	wait1Msec(delayDuration); //wait for objects to be dropped
	closeClaw();
	createManeuver(lift, liftMax, liftStillSpeed);
	driveStraight(-30, true); //drive to fence
	while (potentiometerVal(lift) < liftThrowPos);
	setClawStateManeuver(true);
	while (driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);
}

void driveToWall() {
	goToPosition(lift, liftBottom, -liftStillSpeed);
	driveStraight(24);
}

void initialPillow() {
	setPower(lift, -liftStillSpeed);

	//open claw, drive away from wall, and lift up a little bit
	setClawStateManeuver(true, 50);
	driveStraight(7, true);
	while(driveData.isDriving || lift.maneuverExecuting);

	//drive to central pillow
	turn(autoSign * -57, true);
	while(turnData.isTurning || claw.maneuverExecuting);
	driveStraight(16);

	closeClaw(); //clamp pillow
}

task skillz() {
	setClawStateManeuver(true);
	driveStraight(-10, true);
	while (claw.maneuverExecuting || driveData.isDriving);
	grabNdump(1000);
	driveToWall();

	grabNdump(500);
	driveToWall();

	grabNdump(500);

	//get pillow in center of field
	setLiftStateManeuver(false);
	createManeuver(claw, clawOpenPos+200, clawStillSpeed);
	while (lift.maneuverExecuting || claw.maneuverExecuting);
	turn(-38, false, 40, 127, -10);
	driveStraight(20);
	closeClaw(); //grab pillow

	//dump pillow
	createManeuver(lift, liftMax, -liftStillSpeed);
	turn(45, true);
	while (turnData.isTurning);
	driveStraight(-10, true);
	while (driveData.isDriving || lift.maneuverExecuting);
	openClaw();

	//grab and dump jacks
	driveToWall();
	grabNdump(0);
	goToPosition(lift, liftBottom);
}

task pillowAuton() {
	initialPillow();

	//go to fence and lift up
	setLiftStateManeuver(true);
	driveStraight(10.5, true);
	while (driveData.isDriving);
	turn(autoSign * 57, true, 40, 80, -10); //turn to face fence
	while (turnData.isTurning);
	driveStraight(20, true); // drive up to wall
	while (driveData.isDriving || lift.maneuverExecuting);

	openClaw(); //release pillow
	wait1Msec(600); //wait for pillow to fall
	closeClaw();
	driveStraight(-4); //back up
	hyperExtendClaw();

	//push jacks over
 	driveStraight(6);
 	closeClaw();

 	createManeuver(claw, clawMax, clawStillSpeed);

 	//drive to other wall and lift down
 	driveStraight(-10, true);
 	while (driveData.isDriving);
 	turn(autoSign * 65, true, 40, 127, -20);
 	while (turnData.isTurning || claw.maneuverExecuting);
 	createManeuver(lift, liftMiddle+100, liftStillSpeed);
 	driveStraight(35, true);
 	while (driveData.isDriving || lift.maneuverExecuting);
 	turn(autoSign * -65, false, 40, 120, -40);
 	driveStraight(10);

 	goToPosition(lift, liftTop+75); //push jacks over
 	driveStraight(5);
 	closeClaw();
}

task oneSideAuton() {
	initialPillow();

	goToPosition(lift, liftMiddle, liftStillSpeed);

	turn(autoSign * -100, false, 45, 127, -20);
	driveStraight(-20, true);
	createManeuver(lift, liftMax, liftStillSpeed);
	while (potentiometerVal(lift) < liftThrowPos);
	createManeuver(claw, clawMax-75, clawStillSpeed);
	while (driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);
	wait1Msec(250);

	driveToWall();
	grabNdump(250);
	//createManeuver(claw, clawMax, clawStillSpeed); //open claw
	//createManeuver(lift, liftTop-450, liftStillSpeed); //lift to near top
 // driveStraight(5, true); //drive away from wall
 // while(driveData.isDriving);

 // turn(autoSign * -30, true);
 // while (turnData.isTurning);

 // driveStraight(18, true);
 // while (driveData.isDriving || claw.maneuverExecuting);

 // turn(autoSign * 37, true); //turn toward wall
 // while (turnData.isTurning || lift.maneuverExecuting);

 // //knock off jacks
 // driveStraight(42);
 // goToPosition(lift, 1250, liftStillSpeed);
 // closeClaw();
 // wait1Msec(2500);

 // //go to back jacks
 //	setClawStateManeuver(true);
 //	turn(autoSign * 120, true, 40, 127, -10);
 //	while (turnData.isTurning);
 //	setLiftStateManeuver(false);
 //	driveStraight(46, true);
 //	while (driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);
 //	closeClaw();
 //	wait1Msec(3500);

 //	//dump
 //	createManeuver(lift, liftMax, -liftStillSpeed);
 //	turn(autoSign * 50, true);
 //	while (turnData.isTurning);
 //	driveStraight(-30, true);
 //	while(driveData.isDriving || lift.maneuverExecuting);
 //	openClaw();
 //	goToPosition(lift, liftBottom);
}

task autonomous() {
	lift.maneuverExecuting = false;
	claw.maneuverExecuting = false;
	startTask(maneuvers);

	int sidePos = SensorValue[sidePot];

	autoSign = (sidePos < 1900) ? 1 : -1;

	//start appropriate autonomous task
	if (sidePos>1030 && sidePos<2585) {
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

		liftControl();

		clawControl();
	}
}