package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;


public class IO {

	private double motorSpeed;
	private IOMotorState currentState = IOMotorState.OFF;

	private DoubleSolenoid left; //Left flap solenoid
	private DoubleSolenoid right; //Right flap solenoid

	private DoubleSolenoid leftPopper;
	private DoubleSolenoid rightPopper;

	private VictorSP leftWall;
	private VictorSP leftFlap;
	private VictorSP rightWall;
	private VictorSP rightFlap;

	public IO(int leftSolenoid1, int leftSolenoid2, int rightSolenoid1, int rightSolenoid2, int leftPopperSolenoid1, int leftPopperSolenoid2, int rightPopperSolenoid1, int rightPopperSolenoid2, int leftWallVictor, int leftFlapVictor, int rightWallVictor,  int rightFlapVictor, double initialMotorSpeed) {
		motorSpeed = initialMotorSpeed;

		left = new DoubleSolenoid(leftSolenoid1, leftSolenoid2);
		right = new DoubleSolenoid(rightSolenoid1, rightSolenoid2);

		leftPopper = new DoubleSolenoid(leftPopperSolenoid1, leftPopperSolenoid1);
		rightPopper = new DoubleSolenoid(rightPopperSolenoid1, rightPopperSolenoid2);

		leftWall = new VictorSP(leftWallVictor);
		leftFlap = new VictorSP(leftFlapVictor);
		rightWall = new VictorSP(rightWallVictor);
		rightFlap = new VictorSP(rightFlapVictor);
	}

	public void shoot() {
		leftPopper.set(DoubleSolenoid.Value.kForward);
		rightPopper.set(DoubleSolenoid.Value.kForward);
		rollersOn(true);
	}

    public void finishShooting() {
    	leftPopper.set(DoubleSolenoid.Value.kOff);
    	rightPopper.set(DoubleSolenoid.Value.kOff);
		rollersOff();
    }

	public void rollersOn(boolean reverse) {
		if (reverse) {
            currentState = IOMotorState.REVERSE;
        } else {
            currentState = IOMotorState.FORWARD;
        }
	}

	public void rollersOff() {
		currentState = IOMotorState.OFF;
	}

	public void rollersSetSpeed(int speed) {
		motorSpeed = speed;
	}

	public void openFlaps() {
		left.set(DoubleSolenoid.Value.kForward);
		right.set(DoubleSolenoid.Value.kForward);
	}

	public void closeFlaps() {
		left.set(DoubleSolenoid.Value.kReverse);
		right.set(DoubleSolenoid.Value.kReverse);
	}

	public void update() {
        double speed = currentState == IOMotorState.FORWARD ? motorSpeed : currentState == IOMotorState.REVERSE ? -motorSpeed : 0.0;

        leftWall.set(-speed);
        leftFlap.set(-speed);
        rightWall.set(speed);
        rightFlap.set(speed);
	}
}

enum IOMotorState {
	FORWARD, REVERSE, OFF
}
