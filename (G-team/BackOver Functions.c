
static float  pinchR_Kp = 0.2;
static float  pinchRRequestedValue;
static float  pinchR_Kd = 0.3;

static float  pinchL_Kp = 0.2;
static float  pinchLRequestedValue;
static float  pinchL_Kd = 0.3;

float pinchRD;
float pinchRP;
float lastpinchRError;
float pinchRDF;

float pinchLD;
float pinchLP;
float lastpinchLError;
float pinchLDF;







//Pinch Values Right
//Clamp = 450
//Straight = 2000
//Locked = 3850
//V = 1150

//Pinch Values Left
//Clamp = 4000
//Straight = 2600
//Locked = 1250
//V = 3600
task pinchRController()
{
	float  pinchRSensorCurrentValue;
	float  pinchRError;
	float  pinchRDrive;

	while( true )
	{
		// Read the sensor value and scale
		pinchRSensorCurrentValue = SensorValue[RClawPot];

		// calculate error
		pinchRError =  pinchRRequestedValue - pinchRSensorCurrentValue;

		// calculate drive
		pinchRP = (pinchR_Kp * pinchRError);

		pinchRD = pinchRError- lastpinchRError;
		pinchRDF = (pinchR_Kd * pinchRD);

		pinchRDrive = pinchRP + pinchRDF;

		// limit drive
		if( pinchRDrive > 127 )
			pinchRDrive = 127;
		if( pinchRDrive < (-127) )
			pinchRDrive = (-127);

		// send to motor

		motor[RClaw] = pinchRDrive;

		lastpinchRError = pinchRError;

		// Don't hog cpu
		wait1Msec( 25 );
	}
}


task pinchLController()
{
	float  pinchLSensorCurrentValue;
	float  pinchLError;
	float  pinchLDrive;

	while( true )
	{
		// Read the sensor value and scale
		pinchLSensorCurrentValue = SensorValue[LClawPot];

		// calculate error
		pinchLError =  pinchLRequestedValue - pinchLSensorCurrentValue;

		// calculate drive
		pinchLP = (pinchL_Kp * pinchLError);

		pinchLD = pinchLError- lastpinchLError;
		pinchLDF = (pinchL_Kd * pinchLD);

		pinchLDrive = pinchLP + pinchLDF;

		// limit drive
		if( pinchLDrive > 127 )
			pinchLDrive = 127;
		if( pinchLDrive < (-127) )
			pinchLDrive = (-127);

		// send to motor

		motor[LClaw] = pinchLDrive;

		lastpinchLError = pinchLError;

		// Don't hog cpu
		wait1Msec( 25 );
	}
}
