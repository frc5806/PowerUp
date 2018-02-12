package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;


public class IO {

	private double motorSpeed;

	private DoubleSolenoid left; //Left flap solenoid
	private DoubleSolenoid right; //Right flap solenoid

	private DoubleSolenoid leftPopper;
	private DoubleSolenoid rightPopper;

	private Victor leftWall;
	private Victor leftFlap;
	private Victor rightWall;
	private Victor rightFlap;

	public IO(int leftSolenoid1, int leftSolenoid2, int rightSolenoid1, int rightSolenoid2, int leftPopperSolenoid1, int leftPopperSolenoid2, int rightPopperSolenoid1, int rightPopperSolenoid2, int leftWallVictor, int leftFlapVictor, int rightWallVictor,  int rightFlapVictor, double initialMotorSpeed) {
		motorSpeed = initialMotorSpeed;

		left = new DoubleSolenoid(leftSolenoid1, leftSolenoid2);
		right = new DoubleSolenoid(rightSolenoid1, rightSolenoid2);

		leftPopper = new DoubleSolenoid(leftPopperSolenoid1, leftPopperSolenoid1);
		rightPopper = new DoubleSolenoid(rightPopperSolenoid1, rightPopperSolenoid2);

		leftWall = new Victor(leftWallVictor);
		leftFlap = new Victor(leftFlapVictor);
		rightWall = new Victor(rightWallVictor);
		rightFlap = new Victor(rightFlapVictor);
	}

	public void shoot() {
		leftPopper.set(DoubleSolenoid.Value.kForward);
		rightPopper.set(DoubleSolenoid.Value.kForward);
		rollersOn(true);
	}

	public void rollersOn(boolean reverse) {
		double speed = reverse ? -motorSpeed : motorSpeed;
		leftWall.set(-speed);
		leftFlap.set(-speed);
		rightWall.set(speed);
		rightFlap.set(speed);
	}

	public void rollersOff() {
		leftWall.set(0.0);
		leftFlap.set(0.0);
		rightWall.set(0.0);
		rightFlap.set(0.0);
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

	/* Nothing to do in an update function unless Victors' speed needs to be constantly set
	 *
	 * public void update() {
	 *
	}*/

}
