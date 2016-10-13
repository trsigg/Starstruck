task main() {
	int coeff;

	while (true) {
		coeff = 127 * (1 - 2*vexRT[Btn6U]);


		motor[port1] = vexRT[Btn5D] * coeff;
		motor[port2] = vexRT[Btn5U] * coeff;
		motor[port3] = vexRT[Btn7U] * coeff;
		motor[port4] = vexRT[Btn7R] * coeff;
		motor[port5] = vexRT[Btn7D] * coeff;
		motor[port6] = vexRT[Btn7L] * coeff;
		motor[port7] = vexRT[Btn8U] * coeff;
		motor[port8] = vexRT[Btn8R] * coeff;
		motor[port9] = vexRT[Btn8D] * coeff;
		motor[port10] = vexRT[Btn8L] * coeff;
	}
}
