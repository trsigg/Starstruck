#include "PID.c"
#include "quadraticRamp.c"

enum rampType { PD, QUAD };

typedef struct {
  rampType algorithm;
  PID pd;
  quadraticRamp quadRamp;
} rampHandler;

void initializeRampHandler(rampHandler *ramper, float target, float in1, float in2, float in3) { //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
  if (in1 == 0) {
    ramper->algorithm = PD;
    initializePID(ramper->pd, target, in2, 0, in3);
  } else {
    ramper->algorithm = QUAD;
    initializeQuadraticRamp(ramper->quadRamp, target, in1, in2, in3);
  }
}

float rampRuntime(rampHandler *ramper, float input) {
  if (ramper->algorithm == PD) {
    return PID_runtime(ramper->pd, input);
  } else {
    return quadraticRampRuntime(ramper->quadRamp, input);
  }
}
