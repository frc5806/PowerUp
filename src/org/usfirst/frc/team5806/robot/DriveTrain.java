package org.usfirst.frc.team5806.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
	enum AutoPosition {
		LEFT, RIGHT, CENTER	
	}

	enum AutoState {
		FORWARD, LEFT_SPLINE, RIGHT_SPLINE
	}

	static final double MAX_SPEED = 0.85;
	static final double MIN_SPEED = 0.1;
	static final double FORWARD_DAMPENING_THRESHOLD = 4*Math.PI/2.0;
	static final double TURN_DAMPENING_THRESHOLD = 50.0;
	static final double LEFT_ENCODER_TO_DIST = 4*Math.PI / 100.0;
	static final double RIGHT_ENCODER_TO_DIST = 4*Math.PI / 100.0;
	static final double FORWARD_CORRECTION_FACTOR = 0.05;
	static final double TURN_CORRECTION_FACTOR = 0.1;
	static final int TOP_TICKS_PER_SECOND = 1300;
	static final double RATE_TO_SPEED_LEFT = 500;
	static final double RATE_TO_SPEED_RIGHT = 500;

	// Auto
	static final double VOLT_DAMP = 0.00055;
	double MIN_VOLT = 0.2;
	double K1 = 1.0/(14*1.2);
	double K3 = 0.0;
	
	ArrayList<Double> normalLeft = new ArrayList<Double>();
	ArrayList<Double> normalRight = new ArrayList<Double>();
	
	FileReader reader;

	AutoState aState;

	int splineCounter;
	
	double lEnc, rEnc, lDistSpeed, lRateAvg, lRateAvg2, rRateAvg, rRateAvg2, rDistSpeed, lastLTicks, lastRTicks, lastUpdate, lastLSpeed, lBaseSpeed, rBaseSpeed, lastRSpeed;

	Encoder lEncoder;
	Encoder rEncoder;
	VictorSP[] motors = new VictorSP[2];


	AHRS ahrs;

	public DriveTrain(int left, int right) {
		//lEncoder = new Encoder(0, 1);
		//rEncoder = new Encoder(2, 3);
		motors[0] = new VictorSP(left);
		motors[0].setInverted(true);
		motors[1] = new VictorSP(right);

		for (VictorSP motor:motors) motor.set(0);

		ahrs = new AHRS(SPI.Port.kMXP);  	
		ahrs.reset();

		/*lEncoder.reset();
		lEncoder.setDistancePerPulse(1);
		rEncoder.reset();
		rEncoder.setDistancePerPulse(1);
		lEncoder.setReverseDirection(true);
		*/
		lastUpdate = Timer.getFPGATimestamp();
		lastLTicks = lEncoder.get();
		lastRTicks = rEncoder.get();

		lBaseSpeed = 0.0;
		rBaseSpeed = 0.0;
		lastLSpeed = 0.0;
		lastRSpeed = 0.0;
		lRateAvg = 0.0;
		rRateAvg = 0.0;

		reader = new FileReader("/home/lvuser/TestFile");
	}

	private void setLeft(double speed) {
		motors[0].set(speed);
	}

	private void setRight(double speed) {
		motors[1].set(speed);
	}

	public void goForwardNEW(double distance, double maxSpeed) {
		lEncoder.reset();
		rEncoder.reset();

		double leftVolt = maxSpeed;
		double rightVolt = maxSpeed;

		double Ku = 0.25 * (0.0007/VOLT_DAMP); // In seconds
		double Tu = 0.25 * (0.0007/VOLT_DAMP); // In seconds
		double RAMP_UP_TICKS = 200;
		double MIN_SPEED_UP = 0.5;
		double MIN_SPEED_DOWN = 0.5;
		double volt = 0.0;
		double errorAccumulation = 0.0;
		double errorDamp = 1.25;
		double integral = 0.0;
		double I = (VOLT_DAMP/0.6) * 1.2 * (Ku/Tu);
		double error = 0.0;
		double prevError = 0.0;
		double prevTime = 0.0;
		double curTime = 1000.0*System.currentTimeMillis();
		double derivative = 0.0;
		double D = (VOLT_DAMP/0.6) * (3.0*Ku*Tu/40.0);

		while (lEncoder.get()/2 + rEncoder.get()/2 < distance) {
			motors[0].set(leftVolt); motors[1].set(rightVolt);
			double ticksTraveled = (lEncoder.get()/2 + rEncoder.get()/2);

			double encoderLeft = lEncoder.get();
			double encoderRight = rEncoder.get();
			SmartDashboard.putNumber("leftEncoders", encoderLeft);
			SmartDashboard.putNumber("rightEncoders", encoderRight);

			Timer.delay(0.01);

			prevError = error;
			error = encoderLeft-encoderRight;
			//if (integral*error < 0.0) integral = 0.0;
			integral += error*0.2;
			errorAccumulation += error*VOLT_DAMP*Ku;
			if (ticksTraveled > 0.0 && ticksTraveled < RAMP_UP_TICKS) {
				volt = MIN_SPEED_UP + (maxSpeed-MIN_SPEED_UP)*(ticksTraveled/RAMP_UP_TICKS);
			} else if (ticksTraveled >= distance - RAMP_UP_TICKS) {
				volt = MIN_SPEED_DOWN + (maxSpeed-MIN_SPEED_DOWN)*(distance-ticksTraveled)/(distance-RAMP_UP_TICKS);
			} else {
				volt = maxSpeed;
			}

			prevTime = curTime;
			curTime = 1000.0*System.currentTimeMillis();
			derivative = (error - prevError)/(curTime - prevTime);

			leftVolt = volt - errorAccumulation + I*integral + D*derivative;
			rightVolt = volt + errorAccumulation + I*integral + D*derivative;
		} 
		motors[0].set(0); motors[1].set(0);
		return;
	}

	// Don't try negative speed for now
	public void driveFowardOLD(double maxSpeed, double minSpeed, double accelLength, double deaccelLength, double distance, double direction) {
		lEncoder.reset();
		rEncoder.reset();

		double speed = minSpeed;
		double distanceTraveled;

		setLeft(speed);
		setRight(speed);
		do {
			distanceTraveled = ((Math.abs(lEncoder.get()*LEFT_ENCODER_TO_DIST)+Math.abs(rEncoder.get()*RIGHT_ENCODER_TO_DIST)) / 2.0);

			double speedCorrection = FORWARD_CORRECTION_FACTOR * (Math.abs(lEncoder.get()*LEFT_ENCODER_TO_DIST)-Math.abs(rEncoder.get()*RIGHT_ENCODER_TO_DIST));
			speedCorrection = Math.min(Math.max(speedCorrection, -speed), speed);
			setLeft(direction*(speed-speedCorrection));
			setRight(direction*(speed+speedCorrection));

			SmartDashboard.putNumber("speedCorrection", speedCorrection);
			SmartDashboard.putNumber("spppppeeed", speed);
			SmartDashboard.putNumber("leftMotor", speed-speedCorrection);
			SmartDashboard.putNumber("rightMotor", speed+speedCorrection);
			SmartDashboard.putNumber("distanceTraveled", distanceTraveled);
			double error = Math.abs(distance-distanceTraveled) / Math.abs(distance);
			if(1-error < accelLength) {
				speed = minSpeed + ((maxSpeed - minSpeed) * ((1-error) / accelLength));
			}
			if(error < deaccelLength) {
				speed = minSpeed + ((maxSpeed - minSpeed) * (error / deaccelLength));
			}
			updateDashboard();
		} while(distanceTraveled < distance);
		setLeft(0);
		setRight(0);
	}

	/**
	 * Set speed to negative to turn the opposite direction.
	 */
	// Don't try negative speed for now
	public void turn(double maxSpeed, double minSpeed, double accelLength, double deaccelLength, double degrees, double direction) {
		lEncoder.reset();
		rEncoder.reset();

		double startingAngle = ahrs.getAngle();
		double speed = minSpeed;
		double degreesTurned;
		do { 
			degreesTurned = Math.abs(ahrs.getAngle() - startingAngle);

			double speedCorrection = TURN_CORRECTION_FACTOR * (Math.abs(lEncoder.get()*LEFT_ENCODER_TO_DIST)-Math.abs(rEncoder.get()*RIGHT_ENCODER_TO_DIST));
			speedCorrection = Math.min(Math.max(speedCorrection, -speed), speed);
			//speedCorrection = 0;
			setLeft(direction*Math.max((speed-speedCorrection), 0));
			setRight(direction*Math.min(-(speed+speedCorrection), 0));

			double error = Math.abs(degrees-degreesTurned) / Math.abs(degrees);
			if(1-error < accelLength) {
				speed = minSpeed + ((maxSpeed - minSpeed) * (1-error) / accelLength);
			}
			if(error < deaccelLength) {
				speed = minSpeed + ((maxSpeed - minSpeed) * error / deaccelLength);
			}

			SmartDashboard.putNumber("speedCorrection", speedCorrection);
			SmartDashboard.putNumber("spppppeeed", speed);
			SmartDashboard.putNumber("leftMotor", speed-speedCorrection);
			SmartDashboard.putNumber("rightMotor", speed+speedCorrection);
			SmartDashboard.putNumber("degreesTurned", degreesTurned);
			updateDashboard();
		} while(degreesTurned < degrees);
		setLeft(0);
		setRight(0);
	}
	
	/*
	 * Teleop:
	 */

	public void setSpeeds(double lSpeed, double rSpeed) {
		/*if(Math.abs(lSpeed) < MIN_SPEED) lSpeed = 0;
        if(Math.abs(rSpeed) < MIN_SPEED) rSpeed = 0;
        lSpeed = Math.signum(lSpeed)*Math.min(MAX_SPEED, Math.abs(lSpeed));
        rSpeed = Math.signum(rSpeed)*Math.min(MAX_SPEED, Math.abs(rSpeed));*/

		setLeft(lSpeed);
		setRight(rSpeed);
	}

	public void setDistanceSpeeds(double lSpeed, double rSpeed) {
		this.lDistSpeed = lSpeed;
		this.rDistSpeed = 0.98*rSpeed;
		lBaseSpeed = lDistSpeed*TOP_TICKS_PER_SECOND*RATE_TO_SPEED_LEFT;
		rBaseSpeed = rDistSpeed*TOP_TICKS_PER_SECOND*RATE_TO_SPEED_RIGHT;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("angle", ahrs.getAngle());
		SmartDashboard.putNumber("leftEncoer", lEncoder.get());
		SmartDashboard.putNumber("rightEncoder", rEncoder.get());
		SmartDashboard.putNumber("leftEncoderDist", lEncoder.get()*LEFT_ENCODER_TO_DIST);
		SmartDashboard.putNumber("rightEncoderDist", rEncoder.get()*RIGHT_ENCODER_TO_DIST);
		SmartDashboard.putBoolean("leftEncoderStopped", lEncoder.getStopped());
		SmartDashboard.putBoolean("rightEncoderStopped", rEncoder.getStopped());
		SmartDashboard.putNumber("lSpeed", motors[0].get());
		SmartDashboard.putNumber("rSpeed", motors[1].get());
		SmartDashboard.putNumber("lSpeedAvg", lRateAvg2);
		SmartDashboard.putNumber("rSpeedAvg", rRateAvg2);
		SmartDashboard.putNumber("desiredLSpeed", lastLSpeed+lBaseSpeed);
		SmartDashboard.putNumber("desiredRSpeed", (double)lastRSpeed+rBaseSpeed);
		SmartDashboard.putNumber("desireddddLSpeed", lDistSpeed*TOP_TICKS_PER_SECOND);
		SmartDashboard.putNumber("desireddddRSpeed", rDistSpeed*TOP_TICKS_PER_SECOND);
		SmartDashboard.putNumber("lDistSpeed", lDistSpeed);
		SmartDashboard.putNumber("rDistSpeed", rDistSpeed);
		SmartDashboard.putNumber("lBaseSpeed", lBaseSpeed);
		SmartDashboard.putNumber("rBaseSpeed", rBaseSpeed);
		SmartDashboard.putNumber("lastLSpeed", lastLSpeed);
		SmartDashboard.putNumber("lastRSpeed", lastRSpeed);
		SmartDashboard.putNumber("lastLTicks", lastLTicks);
		SmartDashboard.putNumber("lRate", lEncoder.getRate());
		SmartDashboard.putNumber("rRate", rEncoder.getRate());
		SmartDashboard.putNumber("lastRTicks", lastRTicks);

		lRateAvg2 = lRateAvg2*0.95 + lEncoder.getRate()*0.05;
		rRateAvg2 = rRateAvg2*0.95 + rEncoder.getRate()*0.05;

	}

	public void stop() {
		for (VictorSP motor : motors) motor.stopMotor();
	}

	public void update() {
		lRateAvg = lRateAvg*0.5 + lEncoder.getRate()*0.5;
		rRateAvg = rRateAvg*0.5 + rEncoder.getRate()*0.5;
		double errorL = lDistSpeed*TOP_TICKS_PER_SECOND - lRateAvg;
		lastLSpeed += 0.05 * errorL / (double)(TOP_TICKS_PER_SECOND);
		setLeft(lastLSpeed+lDistSpeed);
		lastLTicks = lEncoder.get();

		double errorR = rDistSpeed*TOP_TICKS_PER_SECOND - rRateAvg;
		lastRSpeed += 0.05 * errorR / (double)(TOP_TICKS_PER_SECOND);
		setRight(lastRSpeed+rDistSpeed);
		lastRTicks = rEncoder.get();
		lastUpdate = Timer.getFPGATimestamp();
	}

	/*
	 * Autonomous
	 */
	
	public void setupAuto(boolean switchOnLeft, AutoPosition position) {
		System.out.println("AUTO_INIT");

		if (position == AutoPosition.RIGHT || position == AutoPosition.LEFT) {
			aState = AutoState.FORWARD;	
		} else {
			setupSpline();
			aState = switchOnLeft ? AutoState.LEFT_SPLINE : AutoState.RIGHT_SPLINE;	
		}
	}

	public boolean updateAuto() {
		if(aState == AutoState.FORWARD) {
			//TODO: update based on gyro
			setSpeeds(0.7, 0.7);
			Timer.delay(0.05);
		} else {
			nextSpline();
		}
	}

	public void setupSpline() {
		splineCounter = 0;
		if(side == 'L' || side == 'l') {
			normalLeft = reader.left;
			normalRight = reader.right;
		} else {
			normalRight = reader.left;
			normalLeft = reader.right;
		}
	}
	
	public void nextSpline() {
			leftSpeed = normalLeft.get(splineCounter);
			rightSpeed = normalRight.get(splineCounter);
			leftAccel = normalLeft.get(splineCounter+1);
			rightAccel = normalRight.get(splineCounter+1);

			double leftVoltage = K1*leftSpeed+K3*leftAccel+MIN_VOLT;
			double rightVoltage = K1*rightSpeed+K3*rightAccel+MIN_VOLT;
			motors[0].set(leftVoltage); 
			motors[1].set(rightVoltage);

			splineCounter += 8;
			Timer.delay(0.01);
	}
}
