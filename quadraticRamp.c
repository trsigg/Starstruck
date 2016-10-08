typedef struct {
  float a, b, c;
} quadraticRamp;

void initializeQuadraticRamp(quadraticRamp *ramp, float target, float initial, float maximum, float final) { //maximum input and initial, maximum, and final output
  ramp->a = (pow(target, 2) * (final+initial-2*maximum) - 2*sqrt(pow(target, 4) * (final-maximum) * (initial-maximum))) / pow(target, 4);
	ramp->b = ((final-initial)/target - a*target) * sgn(target);
  ramp->c = initial;
}

float quadraticRampRuntime(quadraticRamp *ramp, float input) {
  return ramp->a*pow(input, 2) + ramp->b*input + ramp->c;
}
