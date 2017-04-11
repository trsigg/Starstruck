#include "PID.c"
#include "quadraticRamp.c"

enum rampType { PD, QUAD };

typedef struct {
  rampType algorithm;
  PID pd;
  quadraticRamp quadRamp;
} rampHandler;

void initializeRampHandler(rampHandler *ramper, rampType type,float target, float in1, float in2, float in3) { //for PID, in1=kP, in2=kI, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
  ramper->algorithm = type;

  if (type == PD)
    initializePID(ramper->pd, target, in1, in2, in3);
  else
    initializeQuadraticRamp(ramper->quadRamp, target, in1, in2, in3);
}

float rampRuntime(rampHandler *ramper, float input) {
  if (ramper->algorithm == PD) {
    return PID_runtime(ramper->pd, input);
  } else {
    return quadraticRampRuntime(ramper->quadRamp, input);
  }
}
