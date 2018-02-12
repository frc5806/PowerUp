package org.usfirst.frc.team5806.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shifter 
{
	private DoubleSolenoid solshift;
	public boolean highGear;
	
	public Shifter(int port1, int port2)
	{
		solshift = new DoubleSolenoid(port1, port2);
		highGear = false;
		solshift.set(DoubleSolenoid.Value.kReverse);
	}
	public void high()
	{
		highGear = true;
		solshift.set(DoubleSolenoid.Value.kForward);
	}
	public void low()
	{
		highGear = false;
		solshift.set(DoubleSolenoid.Value.kReverse);
	}
}
