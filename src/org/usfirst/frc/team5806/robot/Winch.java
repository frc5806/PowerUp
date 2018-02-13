package org.usfirst.frc.team5806.robot;
import edu.wpi.first.wpilibj.VictorSP;

public class Winch {
	
	public static final double UP_SPEED = 1.0;
	
	private VictorSP motor;
	private WinchState currentState = WinchState.STOP;

	public Winch(VictorSP motor) {
		this.motor = motor;
		currentState = WinchState.STOP;
	}

	public void pull() {
		currentState = WinchState.UP;
	}

	public void stop() {
		currentState = WinchState.STOP;
	}

	public void update() {
		if (currentState == WinchState.UP) {
			motor.set(UP_SPEED);
		} else {
			motor.set(0);
		}
	}
}

enum WinchState {
	UP, STOP
}
