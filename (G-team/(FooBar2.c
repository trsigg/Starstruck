#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, in2,    clawPotL,       sensorPotentiometer)
#pragma config(Sensor, in3,    clawPotR,       sensorPotentiometer)
#pragma config(Sensor, in4,    modePot,        sensorPotentiometer)
#pragma config(Sensor, in5,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Motor,  port1,           rbd,           tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rfd,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           clawMotorR,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           clawMotorL,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           lift1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           lift3,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           lift4,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           lfd,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          lbd,           tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//https://github.com/DHS-Robotics/Starstruck/blob/84a4a8a2bf6a746fdbcc648bd8628b1218530b2a/%24pneumaticSoul.c

//https://github.com/DHS-Robotics/Starstruck/blob/master/(G-team/(2131G.c
//OUR CODE'S LINK ON GITHUB

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

#include "..\Includes\parallelDrive.c"
#include "..\Includes\motorGroup.c"
#include "..\Includes\pd_autoMove.c"
#include "..\Includes\buttonTracker.c"

//Button Config - Used to call to control the lift/claw see lines 158-174, 571-581
#define openClawBtn Btn6D
#define closeClawBtn Btn6U
#define liftUpBtn Btn5U
#define liftDownBtn Btn5D

#define autoDumpOnBtn Btn8U
#define autoDumpOffBtn Btn8D
#define maydayBtn Btn8R

//Lift Potentiometer Positions - Called to set lift to specific value ie goToPosition - Called from motorGroup.c
#define liftBottom 3119
#define liftMax 865

enum liftState { BOTTOM, MIDDLE, TOP, THROW, MAX };
enum clawState { CLOSED, OPEN, HYPEREXTENDED };

int liftPositions[5] = { 3727, 3000, 1800, 1200, 1387 };	//same order as corresponding enums
int clawPositions[3] = { 400, 1150, 2000 };




//REVERSED POT VALUES - FOR WHEN THE POTENTIOMETER BECAME FLIPPED
//KEEPING IN BECAUSE WE MAY HAVE TO RETUNE POT VALUES AGAIN
//ARE 4095- BECAUSE THE POTENTIOMETER WAS TRYING TO MOVE THE WRONG WAY TO ACHIEVE ITS VALUES
/*
#define clawClosedPosL (4095-3058) //3595
#define clawOpenPosL (4095-3181)
#define clawMaxL (4095-3905)
#define clawPushL (4095-3480)
#define clawClosedPosPillowL (4095-3143) //******
#define clawOpenPillowL (4095-3266) //??????
#define clawStriaghtL (4095-3109)
//

#define clawClosedPosR (4095-257) //3595
#define clawOpenPosR (4095-1170)
#define clawMaxR (4095-3599)
#define clawPushR (4095-2080)
#define clawClosedPosPillowR (4095-591) //******
#define clawOpenPillowR (4095-1620) //??????
#define clawStraightR (4095-545)
*/

//POTENTIOMETER VALUES
//Values read by the Potentiometer used for auton =\/ \/ \/ \/ \/=
//Called in the functions to allow the robot to move to certain values
//LEFT SIDE VALUES - TWO POTS
#define clawClosedPosR 3058 //3595
#define clawOpenPosR 3181
#define clawMaxR 3905
#define clawPushR 3480
#define clawClosedPosPillowR 3143
#define clawOpenPillowR 3266
#define clawStriaghtR 3109
//*/
//Right Side Pot Values
#define clawClosedPosL 3933
#define clawOpenPosL 2968
#define clawMaxL 315
#define clawPushL 1777
#define clawClosedPosPillowL 3077
#define clawOpenPillowL 2508
#define clawStraightL 3058

//Values to Assist with PID
#define driverPID false
#define liftDriftDist 300

//Still Speed Constants
#define liftStillSpeed 1
#define clawStillSpeed 20

//Varibles
bool clawOpen = false;
//short autoSign;

bool autoDumping = true
clawState currentState;

//Initializes Motor Groups - Calls from motorGroup.c
motorGroup lift;
motorGroup claw;

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void pre_auton() { //INITIALIZATIONS
	initializeAutoMovement();
  bStopTasksBetweenModes = true;

  initializeDrive(drive); //Initializes Drive -> calls from prallelDrive.c
  setDriveMotors(drive, 4, lfd, lbd, rfd, rbd); //Sets Drive Motors (as it is called) -> needs to know
  																						  //that it is the drive being defined, the number of
  																							//motors being used, and the names of the motors that
  																							//have been defined in the Motor and Sensor Setup.
  attachEncoder(drive, leftEnc, LEFT); //Calls AttachEncoder from parallelDrive.c to say it's on the drive,
  																		 //the name of the Encoder and which side.
  attachEncoder(drive, rightEnc, RIGHT, false, 4);//Calls the same function as right Encoder.
  attachGyro(drive, gyro); //Calls attachGyro from parallelDrive.c to specify the gyro being on the robot.

  //USES MOTOR GROUPS FUNCTIONS
  initializeGroup(lift, 4, lift1, lift2, lift3, lift4); //USES MOTOR GROUP ARRAY IN INCLUDES
  configureButtonInput(lift, liftUpBtn, liftDownBtn, liftStillSpeed, 127, -100);
  //setAbsolutes(lift, liftBottom, 0);
  addSensor(lift, liftPot);

  initializeGroup(claw, 2, clawMotorL, clawMotorR); //USES MOTOR GROUP ARRAY IN INCLUDES
  configureButtonInput(claw, openClawBtn, closeClawBtn, -clawStillSpeed, 127, -127);
  addSensor(claw, clawPotL, true);
  addSensor(claw, clawPotR, true);

}


//NEW TEST
void inactivateTargets() {
	stopTargeting(lift);
//	stopTargeting(rightClaw);
//	stopTargeting(leftClaw);
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
	if (driverPID) {
		if (vexRT[liftUpBtn] == 1) {
			setPower(lift, 127);
			setTargetPosition(lift, limit(getPosition(lift)+liftDriftDist, liftPositions[BOTTOM], liftPositions[MAX]));
		} else if (vexRT[liftDownBtn] == 1) {
			setPower(lift, -127);
			setTargetPosition(lift, limit(getPosition(lift)-liftDriftDist, liftPositions[BOTTOM], liftPositions[MAX]));
		} else {
			maintainTargetPos(lift);
		}
	} else {
		lift.stillSpeed = liftStillSpeed * (getPosition(lift)<liftPositions[MIDDLE] ? -1 : 1);

		takeInput(lift);
	}
}

void autoRelease() {
	if (liftPot <= 900)
	{
		goToPosition(claw, clawOpenPillowL);
	}
}

task maneuvers() {
	while (true) {
		//executeClawPIDs();

		maintainTargetPos(lift);

		EndTimeSlice();
	}
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////

//BEGIN AUTON TAKEN FROM E - COMPLETELY CHANGED FROM E NOW
//autonomous region
void deploy(int waitAtEnd=250) { //THIS IS THE DEPLOY FUNCTION
	//deploy stops

	//OLD DEPLOY
	//goToPosition(lift, liftMax);
	//goToPosition(claw, clawMax, clawStillSpeed);
	//goToPosition(lift, liftBottom + 500);
	//goToPosition(claw, clawOpenPos);

  //driveStraight(10); //DRIVES FORWARD
  //driveStraight(-7); //TO MAKE GRAVITY LOWER THE CLAW

//setDrivePower(drive, 127, 127);
//wait1Msec(750);
//setDrivePower(drive, -127, -127);
//wait1Msec(1000);
//setDrivePower(drive, 0, 0);

//NEW DEPLOY FOR SOLID CLAW
	//driveStraight(3);
	setDrivePower(drive, 127, 127);
	wait1Msec(200);
	setDrivePower(drive, 0, 0);
	//goToPosition(lift, liftBottom + 200, liftStillSpeed);
goToPosition(claw, clawOpenPosR, -clawStillSpeed);
//goToPosition(claw, clawOpenPosR, -clawStillSpeed);
	//goToPosition(lift, liftBottom  + 30, liftStillSpeed);

}

//LETS THE CLAW BE ABLE TO OPEN AND CLOSE - CALLABLE FUNCTION
void setClawStateManeuverL(bool open = !clawOpen) { //toggles by default
	if (open) {
		createManeuver(claw, clawOpenPosL, -clawStillSpeed);
	} else {
		createManeuver(claw, clawClosedPosL, -clawStillSpeed);
	}

	clawOpen = open;
}

//LETS THE CLAW BE ABLE TO OPEN AND CLOSE - CALLABLE FUNCTION
void setClawStateManeuverR(bool open = !clawOpen) { //toggles by default
	if (open) {
		createManeuver(claw, clawOpenPosR, -clawStillSpeed);
	} else {
		createManeuver(claw, clawClosedPosR, -clawStillSpeed);
	}

	clawOpen = open;
}

void openClawL(bool stillSpeed=true) {
	goToPosition(claw, clawOpenPosL, (stillSpeed ? clawStillSpeed : 0));  //OPEN CLAW FUNCTION
}

void closeClawL(bool stillSpeed=true) {
	goToPosition(claw, clawClosedPosL, (stillSpeed ? -clawStillSpeed : 0)); //CLOSE CLAW FUNCTION
}

///////////////////////////////////////////////////////////////////

void openClawR(bool stillSpeed=true) {
	goToPosition(claw, clawOpenPosR, (stillSpeed ? clawStillSpeed : 0));  //OPEN CLAW FUNCTION
}

void closeClawR(bool stillSpeed=true) {
	goToPosition(claw, clawClosedPosR, (stillSpeed ? -clawStillSpeed : 0)); //CLOSE CLAW FUNCTION
}

//void hyperExtendClaw(bool stillSpeed=true) {
//	goToPosition(claw, clawMax, (stillSpeed ? clawStillSpeed : 0)); //PUSHING VALUE - NOT CALLED
//}

void dump() {
	goToPosition(lift, liftMax);
	goToPosition(claw, clawOpenPosL, -clawStillSpeed);
	goToPosition(claw, clawOpenPosR, -clawStillSpeed);
 }


//AUTONOMOUSES ////////////////////////////////////////////////////////////////////////
//FIRST AUTON -> GRABS THE PILLOW, PUTS IT OVER, AND WILL PUSH OVER SOME JACKS
task pillowAutonRight() { //Put true to run 2 things at once

//VERY CHANGED FROM ORIGINAL, REFERENCE LEFT FOR MORE ORIGINAL ENCODER DRIVING, THERE IS A CURRENT PROBLEM WITH ENCODERS - MAY NEED TO JUST REDOWNLOAD FROM GITHUB

	//setClawStateManeuver(true); //open claw
  //goToPosition(lift, liftBottom, liftStillSpeed);
	//openClaw();
  //driveStraight(1); //drive away from wall
  //while(driveData.isDriving);






//TIME BASED AUTON ON RIGHT SIDE
/*  setDrivePower(drive, 127, 127);
	wait1Msec(1000);
	setDrivePower(drive, 0, 0);
	goToPosition(claw, clawClosedPosPillowL, -clawStillSpeed);
	goToPosition(claw, clawClosedPosPillowR, -clawStillSpeed);
	goToPosition(lift, liftBottom + 200, liftStillSpeed);
	setDrivePower(drive, 127, -127);
	wait1Msec(1500);
	setDrivePower(drive, 0, 0);
	setDrivePower(drive, -127, -127);
	wait1Msec(1000);
	setDrivePower(drive, 0, 0);
	goToPosition(lift, liftMax, liftStillSpeed);
	goToPosition(claw, clawOpenPosL, -clawStillSpeed);
	goToPosition(claw, clawOpenPosR, -clawStillSpeed);
	setDrivePower(drive, -127, -127);
	wait1Msec(1300);
  setDrivePower(drive, 0, 0);


*/


  //move toward pillow
//  turn(50.5, false, 40, 120, -40);
  //while(turnData.isTurning || claw.maneuverExecuting);
//  driveStraight(9);
/*
  goToPosition(claw, clawClosedPosPillow, -clawStillSpeed); //clamp pillow
  //closeClaw();
  goToPosition(lift, liftTop, liftStillSpeed);
  turn(30.5, false, 40, 120, -10);
  //--lift up--//
  driveStraight(13);
  //while (driveData.isDriving);
  //turn(0.4, false, 40, 80, -20); //turn to face fence
  //while (turnData.isTurning);
  driveStraight(5, true); // drive up to wall
  //while (driveData.isDriving || lift.maneuverExecuting); //Changed shoulder to lift; WORK?
  goToPosition(claw, clawOpenPos, -clawStillSpeed); //release pillow
  goToPosition(claw, clawPush, -clawStillSpeed);
  goToPosition(claw, clawClosedPos, -clawStillSpeed);
  driveStraight(-13); //back up
  //turn(15.5, false, 40, 120, -10);
  goToPosition(lift, liftPush, liftStillSpeed);
  //goToPosition(claw, clawPush, -clawStillSpeed);
  //push jacks over
 	driveStraight(13);
 	driveStraight(-14);
 	//goToPosition(claw, 900);
 	//Push off Stars
 	goToPosition(lift, liftPushTop, liftStillSpeed);
 	turn(30.5, false, 40, 120, -10);
 	driveStraight(15);
 	turn(-30.5, false, 40, 120, -10);
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
 	*/





 	driveStraight(7);
 	goToPosition(claw, clawClosedPosPillowL, -clawStillSpeed);
 	goToPosition(lift, liftBottom + 40);
 	setDrivePower(drive, 127, -127);
 	wait1Msec(1500);
 	setDrivePower(drive, 0, 0);
 	goToPosition(lift, liftMax);
 	goToPosition(claw, clawOpenPosR, -clawStillSpeed);



}

////////////////////////////////////////////////////////////////////////////

//FIRST AUTON -> GRABS THE PILLOW, PUTS IT OVER, AND WILL PUSH OVER SOME JACKS BUT FROM OTHER SIDE OF THE FIELD
task pillowAutonLeft() { //Put true to run 2 things at once



setClawStateManeuverL(true); //open claw
setClawStateManeuverR(true); //open claws other side
  //goToPosition(claw, clawOpenPos);
	goToPosition(claw, clawOpenPosL, -clawStillSpeed);
	goToPosition(claw, clawOpenPosR, -clawStillSpeed);
  driveStraight(1); //drive away from wall
  //while(driveData.isDriving);

  //move toward pillow
  turn(50.5, false, 40, 120, -40);
  //while(turnData.isTurning || claw.maneuverExecuting);
  driveStraight(12.5);

//  goToPosition(claw, clawClosedPosPillow, -clawStillSpeed); //clamp pillow
  //closeClaw();

//  goToPosition(lift, liftTop, liftStillSpeed);

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

  goToPosition(claw, clawOpenPosL, -clawStillSpeed); //release pillow
  goToPosition(claw, clawOpenPosR, -clawStillSpeed);
  closeClawL();
  closeClawR();
  driveStraight(-13); //back up
  //turn(15.5, false, 40, 120, -10);
//  goToPosition(lift, liftPush, liftStillSpeed);
//  goToPosition(claw, clawPush, -clawStillSpeed);

  //push jacks over
 	driveStraight(13);
 	driveStraight(-13);
 	//goToPosition(claw, 900);

 	//Push off Stars
// 	goToPosition(lift, liftPushTop);
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

////////////////////////////////////////////////////////////////////////////////////

 task programmingSkills() {

  //ONE SIDE AUTON
	//goToPosition(claw, clawPush, -clawStillSpeed);
	//goToPosition(lift, liftPushTop);
  //driveStraight(30); //drive away from wall

  ////////////////////////////////////////////////////////////////////////////////////

  //NEW SINGLE SIDE AUTON ^^^^^^^^^^^^^^

 /////////////////////////////////////////////////////////////////////////////////////


// goToPosition(lift, liftPushTop, liftStillSpeed);
// goToPosition(claw, clawPush, -clawStillSpeed + 5);
 setDrivePower(drive, 127, 127);
 wait1Msec(1800);
 setDrivePower(drive, 0, 0);


 /*
 //NEW BACK AUTON
 wait1Msec(2000);
 goToPosition(claw, clawOpenPos + 250, -clawStillSpeed);
 driveStraight(15);
 goToPosition(claw, clawClosedNew, -clawStillSpeed);
 goToPosition(lift, liftPushTop, liftStillSpeed);
 driveStraight(-18);
 turn(50, false, 40, 120, -40);
 driveStraight(15);
 goToPosition(claw, clawOpenPos, -clawStillSpeed);
  */

  /*
  //NEW AUTON FOR PROGRAMMING SKILLS
  setClawStateManeuver(true);
	goToPosition(claw, clawOpenPos, -clawStillSpeed); //Open The Claw
  driveStraight(1); //drive away from wall
  //move toward pillow
  turn(-50.5, false, 40, 120, -40);
  driveStraight(12.5);
  goToPosition(claw, clawClosedPosPillow, -clawStillSpeed); //Grab Pillow
  goToPosition(lift, liftTop); //Rise Lift
  //Angle to Wall
  turn(30.5, false, 40, 120, -10);
  driveStraight(13);
  driveStraight(10, true); // drive up to wall
  //Relase Claw and Prepare to Push
  goToPosition(claw, clawOpenPos, -clawStillSpeed); //release pillow
  goToPosition(claw, clawClosedPos, -clawStillSpeed);
  driveStraight(-10); //back up
  goToPosition(lift, liftPush, liftStillSpeed);
  goToPosition(claw, clawPush, -clawStillSpeed);
  //Push and Backup
 	driveStraight(12);
 	driveStraight(-20); //reverse from wall to turn
 	//Continue to Push off Stars
 	goToPosition(lift, liftPushTop, liftStillSpeed);
 	turn(-30.5, false, 40, 120, -10);
 	driveStraight(15);
 	turn(30.5, false, 40, 120, -10);
 	driveStraight(15);
 	driveStraight(-10); //reverse from wall to turn again
 	//Another Round Of Pushing Off Stars
 	turn(-30.5, false, 40, 120, -10);
 	driveStraight(15);
 	turn(30.5, false, 40, 120, -10);
 	driveStraight(15);
 	//Reverse To Try and Grab The Pillow and Drop It ****** NEEDS MAJOR TUNING
 	driveStraight(-5);
 	turn(175.5, false, 40, 120, -10);
 	goToPosition(lift, liftBottom, liftStillSpeed);
 	driveStraight(13);
 	goToPosition(claw, clawClosedPos, -clawStillSpeed);
 	driveStraight(-5);
 	turn(-90, false, 40, 120, -10);
 	goToPosition(lift, liftTop, liftStillSpeed);
 	driveStraight(30);
 	goToPosition(claw, clawOpenPos, -clawStillSpeed);
 	driveStraight(-15);
 	//*/
}

//////////////////////////////////////////////////////////////////////////////////////

task autonomous() {
	lift.maneuverExecuting = false;
	claw.maneuverExecuting = false;
	//startTask(maneuvers);

  deploy();

  //autoSign = (SensorValue[sidePot] < 1800) ? 1 : -1;

  //start appropriate autonomous task
  if (SensorValue[modePot] < 500 && SensorValue[modePot] > 0) {
  	startTask(pillowAutonLeft);
  } else if (SensorValue[modePot] < 1830 && (SensorValue[modePot] > 501)) {
  	startTask(programmingSkills);
  } else if (SensorValue[modePot] < 4000 && (SensorValue[modePot] > 1831)) {
  	startTask(pillowAutonRight);
  }
}
//end autonomous region

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
//old
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
*/





//new
void clawControl() {
	if (vexRT[maydayBtn] == 1) {
		setPower(claw, -80);
		setClawTargets(getPosition(rightClaw));
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







//END OF AUTON TAKEN FROM E

/*task autonomous() {
  AutonomousCodePlaceholderForTesting();
}
*/

task usercontrol() {
	while (true) {






    driveRuntime(drive); //USES CLAW FUNCTION TO TAKE INPUTS

    takeInput(lift); //USES LIFT FUNCTIONS TO CONTROL LIFT

    clawControl(); //USES CLAW CONTROL FUNCTION TO CONTROL CLAW

    liftControl();

   // autoRelease();

    //TEST FOR PUSH BUTTON///////////////////////////////////////////////////
/*
    if (vexRT[Btn8U] == 1) //GOES TO THE POSITION TO PUSH OF STARS OFF OF TOP
    {
    	goToPosition(lift, liftPushTop);
    	//goToPosition(claw, clawPush, clawStillSpeed);
    }
    if (vexRT[Btn8D] == 1) //GOES TO THE POSITION TO PUSH OF STARS OF MIDDLE
    {
    	goToPosition(lift, liftPush);
    	//goToPosition(claw, clawPush, clawStillSpeed);
    }
*/




  }
}
