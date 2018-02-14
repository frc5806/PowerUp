package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class Arm {
	private VictorSP armLeft;
	private VictorSP armRight;
	private Potentiometer potLeft;
	private Potentiometer potRight;
	private VictorSP wheels;
	private ArmState state;
	private double stateNum;
	private final static double POT_RIGHT_ABOVE_FLOOR = 0.0;
	private final static double POT_INITIAL = 15.0;
	private final static double POT_SWITCH = 30.0;
	private final static double POT_SCALE = 60.0;
	private final static double MAX_ARM_SPEED = 0.8;

	public Arm(VictorSP armLeft, VictorSP armRight, AnalogPotentiometer potLeft, AnalogPotentiometer potRight, VictorSP wheels) {
		this.armLeft = armLeft;
		this.armRight = armRight;
		this.potLeft = potLeft;
		this.potRight = potRight;
		this.wheels = wheels;
		state = ArmState.INITIAL;
		stateNum = POT_INITIAL;
	}

	public void intake() {
		state = ArmState.INTAKE;
	}

	public void rightAboveFloor() {
		state = ArmState.RIGHT_ABOVE_FLOOR;
	}

	public void switchPosition() {
		state = ArmState.SWITCH;
	}

	public void initial() {
		state = ArmState.INITIAL;
	}

	public void scale() {
		state = ArmState.SCALE;
	}

	public void update() {
		if (state.equals(ArmState.INTAKE)) {
			wheels.set(1.0);
		} else {
			switch(state) {
				case INITIAL:
					stateNum = POT_INITIAL;
					break;
				case RIGHT_ABOVE_FLOOR:
					stateNum = POT_RIGHT_ABOVE_FLOOR;
					break;
				case SWITCH:
					stateNum = POT_SWITCH;
					break;
				case SCALE:
					stateNum = POT_SCALE;
					break;
				default:
					break;
			}
			if (potLeft.get() < stateNum) armLeft.set(MAX_ARM_SPEED);
			else if (potLeft.get() > stateNum) armLeft.set(-MAX_ARM_SPEED);
			else armLeft.set(0.0);

			if (potRight.get() < stateNum) armRight.set(MAX_ARM_SPEED);
			else if (potRight.get() > stateNum) armRight.set(-MAX_ARM_SPEED);
			else armRight.set(0.0);
		}
	}
}

enum ArmState {
	INITIAL, RIGHT_ABOVE_FLOOR, INTAKE, SWITCH, SCALE;
}
