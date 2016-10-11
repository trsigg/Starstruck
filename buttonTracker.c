TVexJoysticks buttons[12] = { Btn5U, Btn5D, Btn6U, Btn6D, Btn7U, Btn7D, Btn7L, Btn7R, Btn8U, Btn8D, Btn8L, Btn8R };
bool tracking[12] = { false, false, false, false, false, false, false, false, false, false, false, false };

int findBtnIndex(TVexJoysticks button) {
	for (int i=0; i<12; i++) {
		if (buttons[i] == button) return i;
	}

	return 0;
}

void updateButtons() {
	for (int i=0; i<12; i++) {
		if (tracking[i] && vexRT[buttons[i]]==0) tracking[i]=false;
	}
}

void startTracking(TVexJoysticks button) {
	tracking[ findBtnIndex(button) ] = true;
}

bool newlyPressed(TVexJoysticks button, bool startTrackingIfPressed=true) {
	updateButtons();

	int index = findBtnIndex(button);

	if (!tracking[index] && vexRT[buttons[index]]==1) {
		if (startTrackingIfPressed) startTracking(button);
		return true;
	} else {
		return false;
	}
}
