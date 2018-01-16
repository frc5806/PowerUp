
package org.usfirst.frc.team5806.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	RobotDrive robotdrive;
	Joystick rightstick;
	Joystick leftstick;
	Encoder leftencoder;
	Encoder rightencoder;
	static final double PERIOD_OF_OSCILLATION = 0.05;
	static final double TICKS_PER_INCH = 7.676;
	
	
	AHRS ahrs;
	
	double integral = 0.0;
	
	public void robotInit() {
		robotdrive = new RobotDrive(1, 3);
		leftstick = new Joystick(0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
		
		rightstick = leftstick;
		leftencoder = new Encoder(2,3);
		rightencoder = new Encoder(0,1);
		rightencoder.setReverseDirection(true);
		
		ahrs = new AHRS(SPI.Port.kMXP);
		
	}


	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}
	
	@Override
	public void autonomousInit() {
		rightencoder.reset();
		leftencoder.reset();
		
		//goForward(1000.0, 0.7);
		//goForward(600.0, 0.6);
		//Timer.delay(0.3);
		turnTest(90.0, 0.7, 20.0);
//		ahrs.reset();
//		ahrs.resetDisplacement();
		//goForward(400.0, 0.6);
		Timer.delay(0.5);
		turnTest(-90.0, 0.7, 20.0);
		//turn(180.0, 0.7, 20.0);
		//turn(90.0, 0.7, 20.0);
		//goForward(400.0, 0.6);
		//goForward(400.0, 0.6);
	}
	
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("gyro", ahrs.getAngle());
	}
	
	static final double VOLT_DAMP = 0.0001;
	
	public void goForward(double distance, double maxSpeed) {
		leftencoder.reset();
		rightencoder.reset();
		
		double leftVolt = maxSpeed;
		double rightVolt = maxSpeed;
		
		double RAMP_UP_TICKS = 200;
		double MIN_SPEED_UP = 0.5;
		double MIN_SPEED_DOWN = 0.5;
		double volt = 0.0;
		double errorAccumulation = 0.0;

		while (leftencoder.get()/2 + rightencoder.get()/2 < distance) {
			robotdrive.tankDrive(leftVolt, rightVolt);
			double ticksTraveled = (leftencoder.get()/2 + rightencoder.get()/2);
			
			double encoderLeft = leftencoder.get();
			double encoderRight = rightencoder.get();
			SmartDashboard.putNumber("leftEncoders", encoderLeft);
			SmartDashboard.putNumber("rightEncoders", encoderRight);
			
			Timer.delay(0.01);
			
			double error = encoderLeft-encoderRight;
			errorAccumulation += error*VOLT_DAMP;
			if (ticksTraveled > 0.0 && ticksTraveled < RAMP_UP_TICKS) {
				volt = MIN_SPEED_UP + (maxSpeed-MIN_SPEED_UP)*(ticksTraveled/RAMP_UP_TICKS);
			} else if (ticksTraveled >= distance - RAMP_UP_TICKS) {
				volt = MIN_SPEED_DOWN + (maxSpeed-MIN_SPEED_DOWN)*(distance-ticksTraveled)/(distance-RAMP_UP_TICKS);
			} else {
				volt = maxSpeed;
			}
			leftVolt = volt - errorAccumulation;
			rightVolt = volt + errorAccumulation;
		} 
		robotdrive.tankDrive(0, 0);
	}
	
	public void turnTest(double degrees, double speed, double rampUpDegrees) {
		ahrs.reset();
		ahrs.resetDisplacement();
		turn(degrees, speed, rampUpDegrees);
	}
	
	public void turn(double degrees, double speed, double rampUpDegrees) {
		System.out.println("turn");
		ahrs.reset();
		ahrs.resetDisplacement();
		
		
		double sign = degrees/Math.abs(degrees);
		double volt = 0.0;
		double MIN_SPEED_UP = 0.71;
		double MIN_SPEED_DOWN = 0.6;
		int timesRun = 0;
		System.out.println(ahrs.getAngle() + " " + degrees);
		System.out.println(Math.abs(ahrs.getAngle()) < Math.abs(degrees));
		double angle;
		ArrayList<Double> angleArray = new ArrayList<Double>();
		do {
			angle = 0;
			for(int a = 0; a < 10; a++) {
				angle += ahrs.getAngle();
			}
			angle /= 10.0;
			
			System.out.println("angle: " + angle);
			angleArray.add(ahrs.getAngle());
			timesRun++;
			SmartDashboard.putNumber("gyro", angle);
			if (Math.abs(angle) > 0.0 && Math.abs(angle) < Math.abs(rampUpDegrees)) {
				volt = (MIN_SPEED_UP + (speed-MIN_SPEED_UP)*(Math.abs(angle)/rampUpDegrees))*sign;
			} else if (Math.abs(angle) >= Math.abs(degrees) - rampUpDegrees) {
				volt = (MIN_SPEED_DOWN + (speed-MIN_SPEED_DOWN)*(Math.abs(degrees-angle))/(Math.abs(degrees)-rampUpDegrees))*sign;
			} else {
				volt = speed*sign;
			}
			SmartDashboard.putNumber("voltage", volt);
			System.out.println("voltage: " + volt);
			//if (!((Math.abs(ahrs.getAngle()) < Math.abs(degrees)))) System.out.println("angle: " + ahrs.getAngle());
			robotdrive.tankDrive(-volt, volt);
			System.out.println("angle after: " + angle);
			//if (!((Math.abs(ahrs.getAngle()) < Math.abs(degrees)))) System.out.println("angle: " + ahrs.getAngle());
		} while(Math.abs(angle) < Math.abs(2*degrees));
		int above70 = 0;
		System.out.println(angleArray.size());
		for (int i = 0; i < angleArray.size(); i++) {
			if (angleArray.get(i) >= 70.0) {
				System.out.println("get(i): " + angleArray.get(i) + " i: " + i);
				above70++;
			}
			if (above70 == 5) break;
		}
		System.out.println("times run: " + timesRun);
		System.out.println("done");
		volt = 0.0;
		
		robotdrive.tankDrive(0, 0);
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
