package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.VictorSP;

public class Grabber {

	public static final double DEFAULT_SPEED = 1.0;

    private double grabSpeed;
    private VictorSP grab;
    private GrabberState currentState = GrabberState.STOP;

    public Grabber(VictorSP grab) {
        this(grab, DEFAULT_SPEED);
	}

    public Grabber(VictorSP grab, double speed) {
        grabSpeed = speed;
        this.grab = grab;
    }

    public void up() {
		currentState = GrabberState.UP;
	}

    public void down() {
		currentState = GrabberState.DOWN;
	}

    public void stop() {
		currentState = GrabberState.STOP;
	}

	public void update() {
		switch (currentState) {
		case UP:
			grab.set(grabSpeed);
            break;

		case DOWN:
			grab.set(-grabSpeed);
		    break;

		case STOP:
			grab.set(0);
            break;

		}
	}
}

enum GrabberState {
	UP, DOWN, STOP
}
