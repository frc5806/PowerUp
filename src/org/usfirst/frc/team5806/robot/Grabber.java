package org.usfirst.frc.team5806.robot;

import edu.wpi.first.wpilibj.VictorSP;

public class Grabber 
{
	private final double SPEED = 1;
	private VictorSP grab;
	private GrabPositions position;
	public Grabber(int port)
	{
		grab = new VictorSP(port);
		position = GrabPositions.STOP;
	}
	public void up()
	{
		position = GrabPositions.UP;
	}
	public void down()
	{
		position = GrabPositions.DOWN;
	}
	public void stop()
	{
		position = GrabPositions.STOP;
	}
	public void update() {
		switch (position){
		case UP:
			grab.set(SPEED);
		
		case DOWN:
			grab.set(-SPEED);
		
		case STOP:
			grab.set(0);
	
		}
	}
}
enum GrabPositions{
	UP,DOWN,STOP
}