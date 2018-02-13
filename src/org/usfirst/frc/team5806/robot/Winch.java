package org.usfirst.frc.team5806.robot;
import edu.wpi.first.wpilibj.VictorSP;

public class Winch {
	private VictorSP motor;
	private WinchState currentState;
	private static final double UP_SPEED = 1.0;

	public Winch(int port) {
		motor = new VictorSP(port);
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
