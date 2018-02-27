
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

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
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
	Compressor c;
	Joystick joy;
	Victor lift;
	double stateNum = -1;
	AnalogPotentiometer leftPot;
	AnalogPotentiometer rightPot;
	Solenoid left, right;
	DoubleSolenoid air, air2, shifter;
	ButtonHandler handler;
	Victor front, back, dartR, dartL, winch1, winch2;
	
	public void robotInit() {
		drivetrain = new DriveTrain(4, 9);
		joy = new Joystick(0);
		handler = new ButtonHandler(joy);
		c = new Compressor();
		c.start();

		leftPot = new AnalogPotentiometer(5);
		rightPot = new AnalogPotentiometer(4);
		
		left = new Solenoid(4);
		right = new Solenoid(5);
		air = new DoubleSolenoid(1, 6);
		air2 = new DoubleSolenoid(3, 2);
		shifter = new DoubleSolenoid(0, 7);
		
		front = new Victor(0);
		back = new Victor(7);
		dartR = new Victor(5);
		dartL = new Victor(8);
		winch1 = new Victor(2);
		winch2 = new Victor(6);


		CameraServer.getInstance().startAutomaticCapture();
					
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
		drivetrain.setSpeeds(0, 0);
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
		
		double leftD = forward-turn*0.9;
		double rightD = forward+turn*0.9;
		//drivetrain.setDistanceSpeeds(leftD, rightD);
		//drivetrain.setDistanceSpeeds(-joy.getRawAxis(1), -joy.getRawAxis(3));
		//drivetrain.setSpeeds(leftD, (rightD < 0 ? rightD : rightD*1.05));
		//drivetrain.setSpeeds(-joy.getRawAxis(1), -joy.getRawAxis(3));
		
		if(joy.getRawAxis(3) > 0.7) {
			left.set(true);
			right.set(true);
			/*Timer.delay(0.04);
			left.set(false);
			right.set(false);
			Timer.delay(1);*/
			System.out.println("fire");
		} else {
			left.set(false);
			right.set(false);
		}
		if(joy.getRawButton(4)) {
			left.set(true);
			right.set(true);
			Timer.delay(0.04);
			left.set(false);
			right.set(false);
			Timer.delay(0.5);
		}
		
		if(joy.getRawAxis(2) > 0.7) {
			air.set(DoubleSolenoid.Value.kForward);
			air2.set(DoubleSolenoid.Value.kForward);
		} else {
			air.set(DoubleSolenoid.Value.kReverse);
			air2.set(DoubleSolenoid.Value.kReverse);
		}
		
		if(joy.getRawButton(3)) {
			//shifter.set(DoubleSolenoid.Value.kReverse);
		} else {
			shifter.set(DoubleSolenoid.Value.kForward);
		}
		
		if(joy.getRawButton(1)) {
			stateNum = 0.02;
		}
		if(joy.getRawButton(2)) {
			stateNum = 0.38;
		}
		if(joy.getRawButton(3)) {
			stateNum = 0.56;
		}
		System.out.println(stateNum);
	
		if(joy.getRawButton(6)) {
			front.set(1);
			back.set(1);
		} else if(joy.getRawButton(5)) {
			front.set(-0.3);
			back.set(-1);
		} else {
			front.set(0);
			back.set(0);
		}
		
		if(joy.getPOV() == 0 || joy.getPOV() == 315 || joy.getPOV() == 45) {
			winch1.set(0.6);
			winch2.set(0.6);
		} else if(joy.getPOV() == 180) {
			winch1.set(-0.6);
			winch2.set(-0.6);
		} else {
			winch1.set(0);
			winch2.set(0);
		}
		
		double rightP = rightPot.get();
		double leftP = leftPot.get();
		double dampConst = 1.0/0.99;
		double error = leftP-(rightP*dampConst-0.01);
		double move = -joy.getRawAxis(1);
		
		double moveR = move+error*8;
		double moveL = move-error*8;
		
		if((rightP > 0.03 || moveR > 0) && (rightP < 0.58*dampConst || moveR < 0)) dartR.set(moveR);
		else dartR.set(0);
		
		if((leftP > 0.03 || moveL > 0) && (leftP < 0.58 || moveL < 0)) dartL.set(moveL);
		else dartL.set(0);
		
		/*double rightP = rightPot.get();
		double leftP = leftPot.get();
		if(stateNum > 0 && Math.abs(rightP-stateNum) > 0.02) {
			double dampConst = 1.0/0.99;
			double error = leftP-(rightP*dampConst-0.01);
			double move = Math.abs(rightP-stateNum) > 0.1 ? 0.7*Math.signum(stateNum-rightP) : 0.4*Math.signum(stateNum-rightP);
			
			double moveR = move+error*8;
			double moveL = move-error*8;
			
			//dartR.set(move);
			if((rightP > 0.03 || moveR > 0) && (rightP < 0.58*dampConst || moveR < 0)) dartR.set(moveR);
			else dartR.set(0);
			
			if((leftP > 0.03 || moveL > 0) && (leftP < 0.58 || moveL < 0)) dartL.set(moveL);
			else dartL.set(0);
		} else {
			dartR.set(0);
			dartL.set(0);
		}*/
		//SmartDashboard.putNumber("LeftEncoder: ", drivetrain.lEncoder.get());
		//SmartDashboard.putNumber("RightEncoder: ", drivetrain.rEncoder.get());
		
		// right 20.9349
		// left 20.7800
		
		// 0.4 - 0.65
		SmartDashboard.putNumber("rightPot", rightPot.get());
		
		// 0.05 - 0.57
		SmartDashboard.putNumber("leftPot", leftPot.get());
		//drivetrain.update();
		drivetrain.updateDashboard();
		Timer.delay(0.05);
	}
	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
