
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

	DriveTrain drivetrain;
	Compressor c;
	Joystick joyDrive, joyOp;

	double dartStateNum = -1;
	boolean isLowGear = true;

	Victor lift;
	AnalogPotentiometer leftPot;
	AnalogPotentiometer rightPot;
	Solenoid left, right;
	DoubleSolenoid air, air2, shifter;
	Victor front, back, dartR, dartL, winch1, winch2;
	
	double BOTTOM = 0.02;
	double VAULT = 0.07;
	double SWITCH = 0.38;
	double HIGH_SWITCH = 0.45;
	double MAINTENANCE = 0.56;
	double INITIAL = 0.4;
	double SCALE = 0.56;
	
	public void robotInit() {
		drivetrain = new DriveTrain(4, 9);
		joyDrive = new Joystick(0);
		joyOp = new Joystick(1);
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
		// Get AUTO DATA
		int retries = 100;
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		while (gameData.length() < 2 && retries > 0) {
			retries--;
			Timer.delay(0.01);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		boolean isSwitchLeft, isScaleLeft;
		if (gameData.length() > 0) {
			isSwitchLeft = gameData.charAt(0) == 'L';
			isScaleLeft = gameData.charAt(1) == 'L';
		} else {
			return;
		}
		
		dartStateNum = SWITCH;
		new Thread(new Runnable() {
		    public void run() {
		        while(updateDart()){};
		    }
		}).start();
		
		// RUN AUTO
		if(isSwitchLeft) {
			
		} else {
			
		}
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
		/*DRIVER*/
		double forward = -joyDrive.getRawAxis(1)*0.9;
		double turn = -joyDrive.getRawAxis(4);
		forward = Math.abs(forward) > 0.1 ? forward : 0.0;
		
		double leftD = forward-turn*0.9;
		double rightD = forward+turn*0.9;
		//drivetrain.setDistanceSpeeds(leftD, rightD);
		//drivetrain.setDistanceSpeeds(-joyDrive.getRawAxis(1), -joyDrive.getRawAxis(3));
		//drivetrain.setSpeeds(leftD, (rightD < 0 ? rightD : rightD*1.05));
		//drivetrain.setSpeeds(-joyDrive.getRawAxis(1), -joyDrive.getRawAxis(3));

		if(joyDrive.getRawAxis(2) > 0.7) {
			isLowGear = true;
		} 
		if(joyDrive.getRawAxis(3) > 0.7) {
			isLowGear = false;
		}
		if(isLowGear) {
			shifter.set(DoubleSolenoid.Value.kForward);
		} else {
			shifter.set(DoubleSolenoid.Value.kReverse);
		}
		
		/*OPERATOR*/
		// Hard pop 
		if(joyOp.getRawAxis(3) > 0.7) {
			left.set(true);
			right.set(true);
			System.out.println("fire");
		} else {
			left.set(false);
			right.set(false);
		}
		// Soft pop
		if(joyOp.getRawAxis(2) > 0.7) {
			left.set(true);
			right.set(true);
			Timer.delay(0.04);
			left.set(false);
			right.set(false);
			Timer.delay(0.5);
		}
		
		// Swing out intake
		if(joyOp.getRawButton(4)) {
			air.set(DoubleSolenoid.Value.kForward);
			air2.set(DoubleSolenoid.Value.kForward);
		} else {
			air.set(DoubleSolenoid.Value.kReverse);
			air2.set(DoubleSolenoid.Value.kReverse);
		}
			
		// Darts
		if(joyOp.getRawButton(1)) {
			dartStateNum = 0.02;
		}
		if(joyOp.getRawButton(2)) {
			dartStateNum = 0.38;
		}
		if(joyOp.getRawButton(3)) {
			dartStateNum = 0.56;
		}
		System.out.println(dartStateNum);
	
		// IO wheels 
		if(joyOp.getRawButton(6)) {
			front.set(1);
			back.set(1);
		} else if(joyOp.getRawButton(5)) {
			front.set(-0.3);
			back.set(-1);
		} else {
			front.set(0);
			back.set(0);
		}
		
		// Climb
		if(joyOp.getPOV() == 0 || joyOp.getPOV() == 315 || joyOp.getPOV() == 45) {
			winch1.set(0.6);
			winch2.set(0.6);
		} else if(joyOp.getPOV() == 180 || joyOp.getPOV() == 225 || joyOp.getPOV() == 135) {
			winch1.set(-0.6);
			winch2.set(-0.6);
		} else {
			winch1.set(0);
			winch2.set(0);
		}

		updateDart();
		
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
	
	boolean updateDart() {
		double rightP = rightPot.get();
		double leftP = leftPot.get();
		if(dartStateNum > 0 && Math.abs(rightP-dartStateNum) > 0.02) {
			double dampConst = 1.0/0.99;
			double error = leftP-(rightP*dampConst-0.01);
			double move = Math.abs(rightP-dartStateNum) > 0.1 ? 0.7*Math.signum(dartStateNum-rightP) : 0.4*Math.signum(dartStateNum-rightP);
			
			double moveR = move+error*8;
			double moveL = move-error*8;
			
			//dartR.set(move);
			if((rightP > 0.03 || moveR > 0) && (rightP < 0.58*dampConst || moveR < 0)) dartR.set(moveR);
			else dartR.set(0);
			
			if((leftP > 0.03 || moveL > 0) && (leftP < 0.58 || moveL < 0)) dartL.set(moveL);
			else dartL.set(0);
			return true;
		} else {
			dartR.set(0);
			dartL.set(0);
			return false;
		}
	}
	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
