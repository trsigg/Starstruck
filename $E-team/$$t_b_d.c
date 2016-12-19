//g'n'd
createLiftManeuver(MAX); //lift to top
driveStraight(-distance, true); //drive to fence
while (getPosition(lift) < liftThrowPos);
createClawManeuver(OPEN);
while (driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);

//skillz
createLiftManeuver(MAX, 90);
turn(49, true);
while (turnData.isTurning);
driveStraight(-13, true);
while (getPosition(lift) < liftThrowPos);
createClawManeuver(OPEN);
while (driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);

driveStraight(-30, true);
while (driveData.totalDist < 10);
createLiftManeuver(MAX);
while (getPosition(lift) < liftThrowPos);
createClawManeuver(OPEN);
while (driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);

//
createLiftManeuver(MAX);
turn(-60, true);
while (getPosition(lift) < liftThrowPos);
createClawManeuver(OPEN);
while (claw.maneuverExecuting || lift.maneuverExecuting || turnData.isTurning);
liftTo(BOTTOM);

//dumpy
  //turn_back_dump(-90, -17, 15?, 45, 120, -20)
turn(autoSign * -90, false, 45, 120, -20);
driveStraight(-17, true);
createLiftManeuver(MAX);
while (potentiometerVal(lift) < liftThrowPos);
createClawManeuver(MAX);
while (driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);

void turnDriveDump (int angle, int dist, int distCutoff=0, float turnConst1=defTurnFloats[0], float turnConst2=defTurnFloats[1], float turnConst3=defTurnFloats[2]) {
  if (angle != 0) { //turning
    if (dist != 0) { //turning & driving
      if (liftDown) liftTo(MIDDLE); //lift up so claw doesn't drag on ground
      turn(angle, false, turnConst1, turnConst2, turnConst3); //turn
      driveStraight(dist, true); //drive
      while (driveData.totalDist < distCutoff); //wait to throw
    } else { //turning but not driving
      turn (angle, true, turnConst1, turnConst2, turnConst3); //turn
      while (abs(turnProgress()) < distCutoff); //wait to throw
    }
  }

  createLiftManeuver(MAX);
  while (potentiometerVal(lift) < liftThrowPos);
  createClawManeuver(OPEN);
  while (turnData.isTurning || driveData.isDriving || lift.maneuverExecuting || claw.maneuverExecuting);
}
