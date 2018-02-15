package org.usfirst.frc.team5806.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shifter {
	private DoubleSolenoid solShift;
	private boolean highGear;

	public Shifter(DoubleSolenoid sholShift, boolean startInHighGear) {
		this.solShift = solShift;
		highGear = startInHighGear ? true : false;
		solShift.set(startInHighGear ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
	}

	public void high() {
		highGear = true;
		solShift.set(DoubleSolenoid.Value.kForward);
	}

	public void low() {
		highGear = false;
		solShift.set(DoubleSolenoid.Value.kReverse);
	}

    public boolean getIfHighGear() {
        return highGear;
    }
}
