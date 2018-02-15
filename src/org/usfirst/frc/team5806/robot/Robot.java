
package org.usfirst.frc.team5806.robot;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Date;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
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
	DriveTrain drivetrain;
	Joystick joy;
//	Encoder leftencoder;
//	Encoder rightencoder;
//	static final double PERIOD_OF_OSCILLATION = 0.05;
//	static final double TICKS_PER_INCH = 7.676;

//	double integral = 0.0;
	
	public void robotInit() {
		//reader = new FileReader("/home/lvuser/TestFile");

		drivetrain = new DriveTrain(1, 3);
		joy = new Joystick(0);
		
		//leftencoder = new Encoder(0,1);
		//rightencoder = new Encoder(2,3);
		//leftencoder.setReverseDirection(true);
				
		System.out.println("INIT");
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
	
	}
	
	@Override
	public void autonomousInit() {
		drivetrain.goAuto();
	}
	
	@Override
	public void autonomousPeriodic() {
		robotdrive.drive(0, 0);
	}
	
	
	/*public void turnTest(double degrees, double speed, double rampUpDegrees) {
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
	}*/

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		//leftencoder.reset();
		//rightencoder.reset();
	}

//	/**
//	 * This function is called periodically during operator control
//	 */
//	double delay = 0.05;
//	final double MIN_JOYSTICK_READ_STRAIGHT = 0.025; // Minimum registered axis movement from joystick. For forward/backward moving
//	final double MIN_JOYSTICK_READ_TURN = 0.025; // Same as above but for turning
////	double correctionFactor = .4/400;
//	double teleopMaxVoltage = 1.0;
//	final double MAX_ENCODERS_PER_SECOND = 1000.0;
//	final double ERROR_SENSITIVITY = 0.02;
//	final double TANK_DRIVE_MAX = 0.75;
////	double leftVolt = 0;
////	double rightVolt = 0;
//
////	double leftSpeed = 0.0;
////	double rightSpeed = 0.0; 
//	double leftTotalError = 0.0;
//	double rightTotalError = 0.0;
//	double leftAvgSpeed = 0.0;
//	double rightAvgSpeed = 0.0;
	
	@Override
	public void teleopPeriodic() {
		double forward = -joy.getRawAxis(1)*0.9;
		double turn = -joy.getRawAxis(2);
		forward = Math.abs(forward) > 0.1 ? forward : 0.0;
		
		double left = forward-turn*0.3;
		double right = forward+turn*0.3;
		drivetrain.setDistanceSpeeds(left, right);
		
		SmartDashboard.putNumber("LeftEncoder: ", drivetrain.lEncoder.get());
		SmartDashboard.putNumber("RightEncoder: ", drivetrain.rEncoder.get());
		drivetrain.update();
		drivetrain.updateDashboard();
		/*SmartDashboard.putNumber("DesiredLeft: ", desiredLeft);
		SmartDashboard.putNumber("DesiredRight: ", desiredRight);
		SmartDashboard.putNumber("LeftAvgSpeed: ", leftAvgSpeed);
		SmartDashboard.putNumber("rightAvgSpeed: ", rightAvgSpeed);
		SmartDashboard.putNumber("StraightSpeed: ", straightSpeed);
		SmartDashboard.putNumber("TurnSpeed: ", turnSpeed);
		SmartDashboard.putNumber("LeftError: ", leftError);
		SmartDashboard.putNumber("RightError: ", rightError);
		SmartDashboard.putNumber("LeftTotalError: ", leftTotalError);
		SmartDashboard.putNumber("RightTotalError: ", rightTotalError);*/
	}
	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
