#include "PID.c"
#include "quadraticRamp.c"

enum rampType { PD, QUAD };

typedef struct {
  rampType algorithm;
  PID pd;
  quadraticRamp quadRamp;
} rampHandler;

void initializeRampHandler(rampHandler *ramper, float target, float in1, float in2, float in3=NULL) { //for PD, in1=kP and in2=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
  if (in3 == NULL) {
    ramper->algorithm = PD;
    initializePID(ramper->pd, target, in1, 0, in2);
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
