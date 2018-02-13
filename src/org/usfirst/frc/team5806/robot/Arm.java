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
	
	public Arm(int victorPortLeft, int victorPortRight, int potPortLeft, int potPortRight, int wheelsPort) {
		armLeft = new VictorSP(victorPortLeft);
		armRight = new VictorSP(victorPortRight);
		potLeft = new AnalogPotentiometer(potPortLeft, 360, 30);
		potRight = new AnalogPotentiometer(potPortRight, 360, 30);
		wheels = new VictorSP(wheelsPort);
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
		if (state.equals(ArmState.INTAKE)) wheels.set(1.0);
		else
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
			if (potLeft.get() < stateNum) armLeft.set(0.8);
			else if (potLeft.get() > stateNum) armLeft.set(-0.8);
			else armLeft.set(0.0);
			
			if (potRight.get() < stateNum) armRight.set(0.8);
			else if (potRight.get() > stateNum) armRight.set(-0.8);
			else armRight.set(0.0);
	}
}

enum ArmState {
	INITIAL, RIGHT_ABOVE_FLOOR, INTAKE, SWITCH, SCALE;
}