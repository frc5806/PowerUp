package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonHandler {
	Button[] buttons;
	public ButtonHandler(Joystick joystick) {
		buttons = new Button[]{new Button(joystick, 1), new Button(joystick, 3), new Button(joystick, 4), new Button(joystick, 2), new Button(joystick, 5), new Button(joystick, 6)};
	}
	
	public Button getButtonWithTitle(char buttonLabel) {
		if(buttonLabel == 'A') return buttons[0];
		if(buttonLabel == 'X') return buttons[1];
		if(buttonLabel == 'Y') return buttons[2];
		if(buttonLabel == 'B') return buttons[3];
		if(buttonLabel == 'L') return buttons[4];
		if(buttonLabel == 'R') return buttons[5];
		
		return null;
	}
	
	public boolean readButton(char buttonLabel) {
		return getButtonWithTitle(buttonLabel).readButton();
	}
	
	public boolean isDown(char buttonLabel) {
		return getButtonWithTitle(buttonLabel).isDown();
	}
}
