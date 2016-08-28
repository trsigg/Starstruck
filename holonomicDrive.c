#include "coreIncludes.c"
#include "motorGroup.c"

typedef struct {
	motorGroup leftDrive, rightDrive;
	TVexJoysticks xInput, yInput, turnInput;
	int deadBand;
} holonomicDrive;

void initializeDrive(holonomicDrive *drive, int deadBand=15, TVexJoysticks xInput=Ch4, TVexJoysticks yInput=Ch3) {
	drive->xInput = xInput;
	drive->yInput = yInput;
	drive->deadBand = deadBand;
}

void setLeftMotors(holonomicDrive *drive, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1) {
	initializeGroup(drive->leftDrive, numMotors, motor1, motor2, motor3, motor4, motor5, motor6);
}

void setRightMotors(holonomicDrive *drive, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1) {
	initializeGroup(drive->rightDrive, numMotors, motor1, motor2, motor3, motor4, motor5, motor6);
}

void setDrivePower(holonomicDrive *drive, int leftPower, int rightPower) {
	setPower(drive->leftDrive, leftPower);
	setPower(drive->rightDrive, rightPower);
}

void setDrivePowerByVector(holonomicDrive *drive, float x, float y) { //sets motor powers so that drive exerts a force along <x, y> with magnitude proportional to its length
	setDrivePower(drive, (x+y)/sqrt(2), (y-x)/sqrt(2));
}

void setDrivePowerByAngle(holonomicDrive *drive, float angle, float magnitude=0, angleType inputType=DEGREES) { //sets motor powers to exert force in a specified direction with a specified magnitude
	angle = convertAngle(angle, RADIANS, inputType);

	if (magnitude == 0) //calculate maximum magnitude in specified direction
		magnitude = 127*sqrt(2) / ((abs(tan(angle)) + 1) * cos(angle));

	setDrivePowerByVector(drive, magnitude*cos(angle), magnitude*sin(angle));
}

void driveRuntime(holonomicDrive *drive) {
	int inX = vexRT[ drive->xInput ];
	int inY = vexRT[ drive->yInput ];

	if (inX < drive->deadBand && inY < drive->deadBand) {
		//Input transformed from joystick circle to velocity square. If you want more detail ask Tynan. Sorry.
		int squareX = sgn(inX)*sqrt(2) / (abs(inY/inX) + 1);
		int squareY = squareX * inY/inX;

		float magnitude = (inX*inX + inY*inY) / 127.0;

		setDrivePowerByVector(drive, squareX*magnitude, squareY*magnitude);
	} else {
		int turnPower = vexRT[ drive->turnInput ];

		setDrivePower(drive, turnPower, -turnPower);
	}
}
