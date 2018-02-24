
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
	
	public void robotInit() {

		drivetrain = new DriveTrain(1, 3);
		joy = new Joystick(0);
		
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
	
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		//leftencoder.reset();
		//rightencoder.reset();
	}

	
	@Override
	public void teleopPeriodic() {
		double forward = -joy.getRawAxis(1)*0.9;
		double turn = -joy.getRawAxis(4);
		forward = Math.abs(forward) > 0.1 ? forward : 0.0;
		
		double left = forward-turn*0.5;
		double right = forward+turn*0.5;
		drivetrain.setSpeeds(left, right);
		
		drivetrain.updateDashboard();
	}
	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
