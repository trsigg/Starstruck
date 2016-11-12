#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    clawPot,        sensorPotentiometer)
#pragma config(Sensor, in4,    modePot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Motor,  port1,           rbd,           tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rfd,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           lift1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           claw1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           claw2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           lift3,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           lift4,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           lfd,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          lbd,           tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//https://github.com/DHS-Robotics/Starstruck/blob/84a4a8a2bf6a746fdbcc648bd8628b1218530b2a/%24pneumaticSoul.c

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

#include "..\Includes\parallelDrive.c"
#include "..\Includes\motorGroup.c"
#include "..\Includes\pd_autoMove.c"
#include "..\Includes\buttonTracker.c"

//Buttons
#define openClawBtn Btn6U
#define closeClawBtn Btn6D
#define liftUpBtn Btn5U
#define liftDownBtn Btn5D

//Positions
#define liftBottom 346
#define liftMax 2310
#define liftTop 1820
#define liftPush 1495+260
#define clawClosedPos 4096-3250 //3595
#define clawOpenPos 4096-2595
#define clawMax 4096-1200
#define clawPush 4096-1860

//Constants
#define liftStillSpeed 10
#define clawStillSpeed 15

//Varibles
bool clawOpen = false;
//short autoSign;

motorGroup lift;
motorGroup claw;



void pre_auton() {
  bStopTasksBetweenModes = true;

  initializeDrive(drive);
  setDriveMotors(drive, 4, lfd, lbd, rfd, rbd);
  attachEncoder(drive, leftEnc, LEFT);
  attachEncoder(drive, rightEnc, RIGHT, false, 4);
  attachGyro(drive, gyro);

  initializeGroup(lift, 4, lift1, lift2, lift3, lift4);
  configureButtonInput(lift, liftUpBtn, liftDownBtn, 10, 127, -100);
  setAbsolutes(lift, liftBottom);
  addSensor(lift, liftPot);

  initializeGroup(claw, 2, claw1, claw2);
  addSensor(claw, clawPot, true);

}

//BEGIN AUTON TAKEN FROM E
//autonomous region
void deploy(int waitAtEnd=250) {
	//deploy stops

	//OLD DEPLOY
	//goToPosition(lift, liftMax);
	//goToPosition(claw, clawMax, clawStillSpeed);
	//goToPosition(lift, liftBottom + 500);
	//goToPosition(claw, clawOpenPos);

  driveStraight(10);
  driveStraight(-7);

}

void setClawStateManeuver(bool open = !clawOpen) { //toggles by default
	if (open) {
		createManeuver(claw, clawOpenPos, clawStillSpeed);
	} else {
		createManeuver(claw, clawClosedPos, -clawStillSpeed);
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

task pillowAutonRight() { //Put true to run 2 things at once
	setClawStateManeuver(true); //open claw
  //goToPosition(claw, clawOpenPos);
	openClaw();
  driveStraight(7); //drive away from wall
  //while(driveData.isDriving);

  //move toward pillow
  turn(-45.5, false, 40, 120, -40);
  //while(turnData.isTurning || claw.maneuverExecuting);
  driveStraight(18);

  //goToPosition(claw, clawClosedPos, ); //clamp pillow
  closeClaw();

  goToPosition(lift, liftTop);

  turn(30.5, false, 40, 120, -10);
  //while(turnData.isTurning);
  //goToPosition(lift, liftTop);

  //--lift up--//
  driveStraight(13);
  //while (driveData.isDriving);
  //turn(0.4, false, 40, 80, -20); //turn to face fence
  //while (turnData.isTurning);
  driveStraight(10, true); // drive up to wall
  //while (driveData.isDriving || lift.maneuverExecuting); //Changed shoulder to lift; WORK?

  openClaw(); //release pillow
  closeClaw();
  driveStraight(-10); //back up
  //turn(15.5, false, 40, 120, -10);
  goToPosition(lift, liftPush);
  goToPosition(claw, clawPush, clawStillSpeed);

  //push jacks over
 	driveStraight(13);
 	driveStraight(-15);
 	//goToPosition(claw, 900);

 	//Push off Stars
 	turn(-30.5, false, 40, 120, -10);
 	driveStraight(15);
 	turn(30.5, false, 40, 120, -10);
 	driveStraight(15);

 	closeClaw();

 	//createManeuver(claw, clawMax, clawStillSpeed); //hyperextend claw

 	//drive to other wall and lift down
 //	driveStraight(-5); //Put True if you want to perform two tasks at once
 	//while (driveData.isDriving);
 	//turn(63);
 	//while (turnData.isTurning || claw.maneuverExecuting);
 	//createManeuver(lift, liftMax, liftStillSpeed);
 	//driveStraight(43);
 	//while (driveData.isDriving || lift.maneuverExecuting);
 	//turn(-80, false, 40, 120, -40);
 	//driveStraight(7);

 	//goToPosition(lift, liftPush); //push jacks over
}

task pillowAutonLeft() { //Put true to run 2 things at once
	setClawStateManeuver(true); //open claw
  //goToPosition(claw, clawOpenPos);
	openClaw();
  driveStraight(7); //drive away from wall
  //while(driveData.isDriving);

  //move toward pillow
  turn(45.5, false, 40, 120, -40);
  //while(turnData.isTurning || claw.maneuverExecuting);
  driveStraight(18);

  //goToPosition(claw, clawClosedPos, ); //clamp pillow
  closeClaw();

  goToPosition(lift, liftTop);

  turn(-30.5, false, 40, 120, -10);
  //while(turnData.isTurning);
  //goToPosition(lift, liftTop);

  //--lift up--//
  driveStraight(12);
  //while (driveData.isDriving);
  //turn(0.4, false, 40, 80, -20); //turn to face fence
  //while (turnData.isTurning);
  driveStraight(10, true); // drive up to wall
  //while (driveData.isDriving || lift.maneuverExecuting); //Changed shoulder to lift; WORK?

  openClaw(); //release pillow
  closeClaw();
  driveStraight(-10); //back up
  //turn(15.5, false, 40, 120, -10);
  goToPosition(lift, liftPush);
  goToPosition(claw, clawPush, clawStillSpeed);

  //push jacks over
 	driveStraight(13);
 	driveStraight(-13);
 	//goToPosition(claw, 900);

 	//Push off Stars
 	turn(-30.5, false, 40, 120, -10);
 	driveStraight(15);
 	turn(30.5, false, 40, 120, -10);
 	driveStraight(15);

 	//createManeuver(claw, clawMax, clawStillSpeed); //hyperextend claw

 	//drive to other wall and lift down
 //	driveStraight(-5); //Put True if you want to perform two tasks at once
 	//while (driveData.isDriving);
 	//turn(63);
 	//while (turnData.isTurning || claw.maneuverExecuting);
 	//createManeuver(lift, liftMax, liftStillSpeed);
 	//driveStraight(43);
 	//while (driveData.isDriving || lift.maneuverExecuting);
 	//turn(-80, false, 40, 120, -40);
 	//driveStraight(7);

 	//goToPosition(lift, liftPush); //push jacks over
}
 task programmingSkills() {
	/*createManeuver(claw, clawOpen, clawStillSpeed); //open claw   CHANGED TO CLAW OPEN
	createManeuver(lift, liftTop, liftStillSpeed); //lift to near top
  driveStraight(5, true); //drive away from wall
  while(driveData.isDriving);

  turn(30, true);
  while (turnData.isTurning || claw.maneuverExecuting);

  driveStraight(10);
  while (driveData.isDriving);

  turn(-40, true); //turn toward wall
  while (turnData.isTurning || lift.maneuverExecuting);  //CHANGED SHOULDER TO LIFT; WORK?

  driveStraight(25);
  goToPosition(lift, 2100); //CHANGED TO OUR VALUES? */

  //OLD SINGLE SIDE AUTON ^^^^^^^^^^^^^^^

  setClawStateManeuver(true);
	openClaw();  //Open The Claw
  driveStraight(7); //drive away from wall

  //move toward pillow
  turn(-45.5, false, 40, 120, -40);
  driveStraight(18);

  closeClaw(); //Grab Pillow

  goToPosition(lift, liftTop); //Rise Lift

  //Angle to Wall
  turn(30.5, false, 40, 120, -10);
  driveStraight(13);

  driveStraight(10, true); // drive up to wall

  //Relase Claw and Prepare to Push
  openClaw(); //release pillow
  closeClaw();
  driveStraight(-10); //back up
  goToPosition(lift, liftPush);
  goToPosition(claw, clawPush, clawStillSpeed);

  //Push and Backup
 	driveStraight(12);
 	driveStraight(-20); //reverse from wall to turn

 	//Continue to Push off Stars
 	turn(-30.5, false, 40, 120, -10);
 	driveStraight(15);
 	turn(30.5, false, 40, 120, -10);
 	driveStraight(15);

 	driveStraight(-20); //reverse from wall to turn again

 	//Another Round Of Pushing Off Stars
 	turn(-30.5, false, 40, 120, -10);
 	driveStraight(15);
 	turn(30.5, false, 40, 120, -10);
 	driveStraight(15);

 	//Reverse To Try and Grab The Pillow and Drop It ****** NEEDS TUNING
 	driveStraight(-15);
 	turn(95.5, false, 40, 120, -10);
 	goToPosition(lift, liftBottom);
 	driveStraight(13);
 	goToPosition(claw, clawClosedPos);
 	driveStraight(-15);
 	turn(90, false, 40, 120, -10);
 	goToPosition(lift, liftTop);
 	driveStraight(30);
 	goToPosition(claw, clawOpenPos);
 	driveStraight(-15);

}

task autonomous() {
	lift.maneuverExecuting = false;
	claw.maneuverExecuting = false;
	//startTask(maneuvers);

  deploy();

  //autoSign = (SensorValue[sidePot] < 1800) ? 1 : -1;

  //start appropriate autonomous task
  if (SensorValue[modePot] >= 0 && SensorValue[modePot] < 9) {
  	startTask(pillowAutonLeft);
  } else if (SensorValue[modePot] > 10 && (SensorValue[modePot] < 30)) {
  	startTask(programmingSkills);
  } else if (SensorValue[modePot] > 35) {
  	startTask(pillowAutonRight);
  }
}
//end autonomous region

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
//END OF AUTON TAKEN FROM E

/*task autonomous() {
  AutonomousCodePlaceholderForTesting();
}
*/

task usercontrol() {
	while (true) {
    driveRuntime(drive);

    takeInput(lift);

    clawControl();
  }
}
