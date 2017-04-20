task six() {
  driveStraight(20);
  turn(-30);
  driveStraight(20);
  turn(120);

  driveStraightAggressive(100);
}

void driveStraightAggressive(int distance) {
  PID straighteningPID;
  initializePID(straighteningPID, 0, 1, .01, 0.2);
  float leftDist = 0;
  float rightDist = 0;

  resetDriveEncoders(drive);

  while (driveEncoderVal(drive) < distance) {
    driveData.leftDist += abs(driveEncoderVal(autoDrive, LEFT));
    driveData.rightDist += abs(driveEncoderVal(autoDrive, RIGHT));

    setDrivePower(127, 127+PID_runtime(straighteningPID, rightDist-leftDist));

    wait1Msec(25);
  }
}
