package org.usfirst.frc.team5806.robot;
import edu.wpi.first.wpilibj.VictorSP;

public class Winch
{
	private VictorSP motor;
	private WinchState currentstate;
	private static final double UP_SPEED = 1.0;
	
	public Winch(int port)
	{
		motor = new VictorSP(port);
		currentstate = WinchState.STOP;
	}
	
	public void pull()
	{
		currentstate = WinchState.UP;
	}
	
	public void stop()
	{
		currentstate = WinchState.STOP;
	}
	
	public void update()
	{
		if (currentstate == WinchState.UP)
		{
			motor.set(UP_SPEED);
		}
		else
		{
			motor.set(0);
		}
	}
}

enum WinchState 
{
	UP, STOP
}