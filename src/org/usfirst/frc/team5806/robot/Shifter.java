package org.usfirst.frc.team5806.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shifter {
	private DoubleSolenoid solshift;
	private boolean highGear;

	public Shifter(int port1, int port2, boolean startInHighGear) {
		solshift = new DoubleSolenoid(port1, port2);
		highGear = startInHighGear ? true : false;
		solshift.set(startInHighGear ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
	}

	public void high() {
		highGear = true;
		solshift.set(DoubleSolenoid.Value.kForward);
	}

	public void low() {
		highGear = false;
		solshift.set(DoubleSolenoid.Value.kReverse);
	}

    public boolean getIfHighGear() {
        return highGear;
    }
}
