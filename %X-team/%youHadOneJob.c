#pragma config(Sensor, in1,    clawPot,        sensorPotentiometer)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    modePot,        sensorPotentiometer)
#pragma config(Sensor, in4,    sidePot,        sensorPotentiometer)
#pragma config(Sensor, in5,    hyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEnc,        sensorQuadEncoder)
#pragma config(Motor,  port1,           lbd,           tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           lfd,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           lift1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           claw1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           claw2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           lift3,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           lift4,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rfd,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          rbd,           tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

#include "../Includes/buttonTracker.c"
#include "../Includes/motorGroup.c"
#include "../Includes/parallelDrive.c"
#include "../Includes/pd_autoMove.c"

//buttons
#define toggleLiftModeBtn Btn8U
#define openClawBtn Btn6D //claw
#define closeClawBtn Btn6U
#define liftUpBtn Btn5U //lift
#define liftDownBtn Btn5D

//positions
#define liftBottom 553//lift
#define liftTop 1350
#define liftMiddle 1167
#define liftVert 2710
#define clawOpenPos 1011 //claw
#define clawClosedPos 1570
#define clawMax 100

//constants
#define liftStillSpeed 10
#define clawStillSpeed 15

//variables
bool clawOpen = false;
short autoSign; //for autonomous, positive if robot is on on the blue side by the pole or the symmetric tile of red side

motorGroup lift;
motorGroup claw;

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeDrive(drive);
  setDriveMotors(drive, 4, lfd, lbd, rfd, rbd);
  attachEncoder(drive, leftEnc, LEFT, true);
  attachEncoder(drive, rightEnc, RIGHT, true, 4);
  attachGyro(drive, hyro);

	initializeGroup(lift, 4, lift1, lift2, lift3, lift4);
  configureButtonInput(lift, liftUpBtn, liftDownBtn, liftStillSpeed);
  addSensor(lift, liftPot);

  initializeGroup(claw, 2, claw1, claw2);
  addSensor(claw, clawPot);
}

//autonomous region
task maneuvers() {
	while (true) {
		executeManeuver(claw);
		executeManeuver(lift);
	}
}

void deployClaw(int waitAtEnd=250) {
	setDrivePower(drive, 127, 127);
	wait1Msec(500);
	setDrivePower(drive, -127, -127);
	wait1Msec(750);
	setDrivePower(drive, 0, 0);
	wait1Msec(waitAtEnd);
}

void setClawStateManeuver(bool open = !clawOpen) { //toggles by default
	if (open) {
		createManeuver(claw, clawOpenPos, -clawStillSpeed);
	} else {
		createManeuver(claw, clawClosedPos, clawStillSpeed);
	}

	clawOpen = open;
}

void openClaw(bool stillSpeed=true) {
	goToPosition(claw, clawOpenPos, (stillSpeed ? clawStillSpeed : 0));
}

void closeClaw(bool stillSpeed=true) {
	goToPosition(claw, clawClosedPos, (stillSpeed ? -clawStillSpeed : 0));
}

void hyperExtendClaw(bool stillSpeed=true) {
	goToPosition(claw, clawMax, (stillSpeed ? clawStillSpeed : 0));
}

void setLiftStateManeuver(bool top = potentiometerVal(lift)<liftMiddle) { //toggles by default
  if (top) {
    createManeuver(lift, liftTop, liftStillSpeed);
  } else {
    createManeuver(lift, liftBottom, liftStillSpeed);
  }
}

task pillowAuton() {
	setClawStateManeuver(true); //open claw
  driveStraight(5, true); //drive away from wall
  while(driveData.isDriving);

  //move toward pillow
  turn(55, true);
  while(turnData.isTurning || claw.maneuverExecuting);
  driveStraight(8);

  closeClaw(); //clamp pillow

  setLiftStateManeuver(true);
  driveStraight(15, true);
  while (driveData.isDriving);
  turn(-40, true, 40, 80, -20); //turn to face fence
  while (turnData.isTurning);
  driveStraight(20, true); // drive up to wall
  while (driveData.isDriving || lift.maneuverExecuting);

  openClaw(); //release pillow
  closeClaw();
  driveStraight(-5); //back up
  hyperExtendClaw();

  //push jacks over
 	driveStraight(10);
 	goToPosition(claw, 850);
 	driveStraight(-7);
 	goToPosition(claw, 700);

}

task oneSideAuton() {
	createManeuver(claw, clawMax, -clawStillSpeed); //open claw
	createManeuver(lift, liftTop-400, liftStillSpeed); //lift to near top
  driveStraight(5, true); //drive away from wall
  while(driveData.isDriving);

  turn(-30, true);
  while (turnData.isTurning || claw.maneuverExecuting);

  driveStraight(10);
  while (driveData.isDriving);

  turn(25, true); //turn toward wall
  while (turnData.isTurning || lift.maneuverExecuting);

  driveStraight(25);
  goToPosition(lift, 1995);
}

task autonomous() {
	lift.maneuverExecuting = false;
	claw.maneuverExecuting = false;
	startTask(maneuvers);

	//deploy stops
	goToPosition(lift, liftMiddle);
	goToPosition(lift, liftBottom);

  deployClaw();

  autoSign = (SensorValue[sidePot] < 1800) ? 1 : -1;

  //start appropriate autonomous task
  if (SensorValue[modePot] > 1870) {
  	startTask(pillowAuton);
  } else {
  	startTask(oneSideAuton);
  }
}
//end autonomous region

//user control region
void liftControl() {
	short potPos = potentiometerVal(lift);
	lift.stillSpeed = liftStillSpeed * ((potPos<liftMiddle || potPos>liftVert) ? -1 : 1);
	takeInput(lift);
}

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

task usercontrol() {
	while (true) {

  	driveRuntime(drive);

  	liftControl();

		clawControl();
  }
}
//end user control region
