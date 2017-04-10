#include "PID.c"
#include "quadraticRamp.c"

enum rampType { PID, QUAD };

typedef struct {
  rampType algorithm;
  PID pid;
  quadraticRamp quadRamp;
} rampHandler;

void initializeRampHandler(rampHandler *ramper, rampType type,float target, float in1, float in2, float in3) { //for PID, in1=kP, in2=kI, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
  ramper->algorithm = type;

  if (type == PID)
    initializePID(ramper->pid, target, in1, in2, in3);
  else
    initializeQuadraticRamp(ramper->quadRamp, target, in1, in2, in3);
}

float rampRuntime(rampHandler *ramper, float input) {
  if (ramper->algorithm == PID) {
    return PID_runtime(ramper->pid, input);
  } else {
    return quadraticRampRuntime(ramper->quadRamp, input);
  }
}
