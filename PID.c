typedef struct {
		float kP, kI, kD; //tuning coefficients
		float target;
		int minSampleTime; //minimum time between sampling input in milliseconds
		float integralMin, integralMax; //minimum and maximum error value which will be added to integral
		float output; //can be used to refer to most recent output
		//internal variables
		long lastUpdated;
		float integral;
		float prevError;
} PID;

void initializePID(PID *pid, float target, float kP, float kI, float kD, int minSampleTime=30, bool inputUpdated=true, float integralMin=NULL, float integralMax=NULL) {
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->target = target;
	pid->minSampleTime = minSampleTime;
	pid->integralMin = integralMin;
	pid->integralMax = integralMax;
	pid->integral = 0;
}

float PID_runtime(PID *pid, float input) {
	long now = nPgmTime;
	long elapsed = now - pid->lastUpdated;
	pid->lastUpdated = now;

	if (elapsed > pid->minSampleTime) {
		float error = (pid->target == 0 ? input : pid->target-input);

		pid->integral += (pid->integralMin==NULL || error>pid->integralMin) && (pid->integralMax==NULL || error>pid->integralMax) ? (error + pid->prevError)*elapsed/2 : 0; //update integral if within bounds of integralMin and integralMax

		pid->output = pid->kP*error + pid->kI*pid->integral + pid->kD*(error - pid->prevError)/elapsed;
		pid->prevError = error;
	}

	return pid->output;
}
