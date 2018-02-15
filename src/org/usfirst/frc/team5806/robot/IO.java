package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;


public class IO {

	private double motorSpeed;
	private IOMotorState currentState = IOMotorState.OFF;

	private DoubleSolenoid leftFlapPiston; //Left flap solenoid
	private DoubleSolenoid rightFlapPiston; //Right flap solenoid

	private DoubleSolenoid leftPopper;
	private DoubleSolenoid rightPopper;

	private VictorSP leftWallMotor;
	private VictorSP leftFlapMotor;
	private VictorSP rightWallMotor;
	private VictorSP rightFlapMotor;

	public IO(DoubleSolenoid leftFlapPiston, DoubleSolenoid rightFlapPiston, 
		DoubleSolenoid leftPopper, DoubleSolenoid rightPopper, 
		VictorSP leftWallMotor, VictorSP rightWallMotor,
		VictorSP leftFlapMotor, VictorSP rightFlapMotor, 
		double initialMotorSpeed) {
		motorSpeed = initialMotorSpeed;

		this.leftFlapPiston = leftFlapPiston;
		this.rightFlapPiston = rightFlapPiston;

		this.leftPopper = leftPopper;
		this.rightPopper = rightPopper;

		this.leftWallMotor = leftWallMotor;
		this.rightWallMotor = rightWallMotor;

		this.leftFlapMotor = leftFlapMotor;
		this.rightFlapMotor = rightFlapMotor;
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
		leftFlapPiston.set(DoubleSolenoid.Value.kForward);
		rightFlapPiston.set(DoubleSolenoid.Value.kForward);
	}

	public void closeFlaps() {
		leftFlapPiston.set(DoubleSolenoid.Value.kReverse);
		rightFlapPiston.set(DoubleSolenoid.Value.kReverse);
	}

	public void update() {
        double speed = currentState == IOMotorState.FORWARD ? motorSpeed : currentState == IOMotorState.REVERSE ? -motorSpeed : 0.0;

        leftWallMotor.set(-speed);
        leftFlapMotor.set(-speed);
        rightWallMotor.set(speed);
        rightFlapMotor.set(speed);
	}
}

enum IOMotorState {
	FORWARD, REVERSE, OFF
}
