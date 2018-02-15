package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.Joystick;

class Button {
	public boolean alreadyRead = false;
	Joystick joystick;
	private int buttonIndex;
	
	public Button(Joystick joystick, int buttonIndex) {
		this.joystick = joystick;
		this.buttonIndex = buttonIndex;
	}
	
	public boolean readButton() {
		boolean buttonValue = joystick.getRawButton(buttonIndex);
		boolean returnValue = buttonValue == true && alreadyRead == false;
		
		// Set alreadyRead
		if(buttonValue == true) alreadyRead = true;
		else alreadyRead = false;
		
		return returnValue;
	}
	
	public boolean isDown() {
		 return joystick.getRawButton(buttonIndex);
	}
}