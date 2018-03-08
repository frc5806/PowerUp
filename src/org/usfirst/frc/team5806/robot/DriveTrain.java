package org.usfirst.frc.team5806.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
import edu.wpi.first.wpilibj.Counter;
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
	//static final double LEFT_ENCODER_TO_DIST = 4*Math.PI / 100.0;
	//static final double RIGHT_ENCODER_TO_DIST = 4*Math.PI / 100.0;
	static final double FORWARD_CORRECTION_FACTOR = 0.05;
	static final double TURN_CORRECTION_FACTOR = 0.1;
	static final double LEFT_TICKS_PER_SECOND = 900;
	static final double RIGHT_TICKS_PER_SECOND = (6150.0/5570.0)*LEFT_TICKS_PER_SECOND;
	static final double RATE_TO_SPEED_LEFT = 500;
	static final double RATE_TO_SPEED_RIGHT = 500;
	static final double RIGHT_STARTER = 1.05;

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
	
	double lEnc, rEnc, errorL, errorR, lDistSpeed, lRateAvg, lRateAvg2, rRateAvg, rRateAvg2, rDistSpeed, lastLTicks, lastRTicks, lastUpdate, lastLSpeed, lastRSpeed;

	Encoder lEncoder;
	Encoder rEncoder;
	AnalogTrigger rTrigger1, rTrigger2, lTrigger;
	AnalogInput a1;
	Counter rMag1, rMag2;
	
	VictorSP[] motors = new VictorSP[2];


	AHRS ahrs;

	public DriveTrain(int left, int right) {
		//lEncoder = new Encoder(9, 8);
		//rEncoder = new Encoder(7, 6);
		motors[0] = new VictorSP(left);
		motors[0].setInverted(true);
		motors[1] = new VictorSP(right);
		motors[1].setInverted(false);
		
		//a1 = new AnalogInput(2);
		AnalogTrigger rTrigger1 = new AnalogTrigger(2);
		AnalogTrigger rTrigger2 = new AnalogTrigger(3);
		rTrigger1.setLimitsVoltage(1, 3.0);
		rTrigger2.setLimitsVoltage(1, 3.0);
		rMag1 = new Counter(rTrigger1);
		rMag2 = new Counter(rTrigger2);

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

		lastLSpeed = 0.0;
		lastRSpeed = 0.0;
		lRateAvg = 0.0;
		rRateAvg = 0.0;

		reader = new FileReader("/home/lvuser/TestFile");
	}

	
	/*TELEOP*/

	private void setLeft(double speed) {
		motors[0].set(speed);
	}

	private void setRight(double speed) {
		motors[1].set(speed);
	}

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
		this.rDistSpeed = rSpeed;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("leftEncoer", lEncoder.get());
		SmartDashboard.putNumber("rightEncoder", rEncoder.get());
		SmartDashboard.putNumber("lChange", lastLSpeed);
		SmartDashboard.putNumber("rChange", lastRSpeed);
		SmartDashboard.putNumber("lSpeed", motors[0].get());
		SmartDashboard.putNumber("rSpeed", motors[1].get());
		SmartDashboard.putNumber("lSpeedAvg", lRateAvg);
		SmartDashboard.putNumber("rSpeedAvg", rRateAvg);
		SmartDashboard.putNumber("errorinavgpercent", (rRateAvg*(5570.0/6150.0)-lRateAvg)/(lRateAvg));
		SmartDashboard.putNumber("errorinavg", (rRateAvg*(5570.0/6150.0)-lRateAvg));
		
		SmartDashboard.putNumber("rMag1", rMag1.get());
		SmartDashboard.putNumber("rMag2", rMag2.get());
	}
	
	public void stop() {
		for (VictorSP motor : motors) motor.stopMotor();
	}

	public void update() {
		lRateAvg = lRateAvg*0.6 + ((lEncoder.get() - lastLTicks)/(Timer.getFPGATimestamp()-lastUpdate))*0.4;
		rRateAvg = rRateAvg*0.6 + ((rEncoder.get() - lastRTicks)/(Timer.getFPGATimestamp()-lastUpdate))*0.4;

		double errorL = lDistSpeed*LEFT_TICKS_PER_SECOND - lRateAvg;
		lastLSpeed += 0.05 * errorL / (double)(LEFT_TICKS_PER_SECOND);
		setLeft(lastLSpeed+lDistSpeed);

		double errorR = rDistSpeed*RIGHT_TICKS_PER_SECOND - rRateAvg;
		lastRSpeed += 0.05 * errorR / (double)(RIGHT_TICKS_PER_SECOND);
		setRight(lastRSpeed+rDistSpeed);
	
		lastLTicks = lEncoder.get();
		lastRTicks = rEncoder.get();
		lastUpdate = Timer.getFPGATimestamp();
		
	}
	

	/*AUTO*/
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

	/* MOVE ROUTINES */
	// Don't try negative speed for now
	public void driveFowardEncoders(double maxSpeed, double minSpeed, double accelLength, double deaccelLength, double distance, double direction) {
		lEncoder.reset();
		rEncoder.reset();

		double speed = minSpeed;
		double distanceTraveled;

		setLeft(speed);
		setRight(speed);
		do {
			distanceTraveled = ((Math.abs(lEncoder.get()/LEFT_TICKS_PER_SECOND)+Math.abs(rEncoder.get()/RIGHT_TICKS_PER_SECOND)) / 2.0);

			double speedCorrection = FORWARD_CORRECTION_FACTOR * (Math.abs(lEncoder.get()/LEFT_TICKS_PER_SECOND)-Math.abs(rEncoder.get()/RIGHT_TICKS_PER_SECOND));
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
			System.out.println(distanceTraveled);
		} while(distanceTraveled < distance);
		setLeft(0);
		setRight(0);
	}
	public void driveFowardGyro(double speed, double time, double direction) {
		double startingAngle = ahrs.getAngle();
		setLeft(speed);
		setRight(speed);
		do {
			degreesTurned = Math.abs(ahrs.getAngle() - startingAngle);

			double speedCorrection = 0.05*degreesTurned;
			setLeft(direction*(speed-speedCorrection));
			setRight(direction*(speed+speedCorrection));

			Timer.delay(0.01);
		} while(distanceTraveled < distance);
		Timer.delay(0.02);
		setLeft(0);
		setRight(0);
	}

	/**
	 * Set speed to negative to turn the opposite direction.
	 */
	// Don't try negative speed for now
	public void turnEncoders(double maxSpeed, double minSpeed, double accelLength, double deaccelLength, double degrees, double direction) {
		lEncoder.reset();
		rEncoder.reset();

		double startingAngle = ahrs.getAngle();
		double speed = minSpeed;
		double degreesTurned;
		do { 
			degreesTurned = Math.abs(ahrs.getAngle() - startingAngle);

			double speedCorrection = TURN_CORRECTION_FACTOR * (Math.abs(lEncoder.get()*LEFT_TICKS_PER_SECOND)-Math.abs(rEncoder.get()*RIGHT_TICKS_PER_SECOND));
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
	public void turn(double maxSpeed, double minSpeed, double accelLength, double deaccelLength, double degrees, double direction) {

		double startingAngle = ahrs.getAngle();
		double speed = minSpeed;
		double degreesTurned;
		do { 
			degreesTurned = Math.abs(ahrs.getAngle() - startingAngle);

			setLeft(direction*speed);
			setRight(direction*-speed);

			double error = Math.abs(degrees-degreesTurned) / Math.abs(degrees);
			if(1-error < accelLength) {
				speed = minSpeed + ((maxSpeed - minSpeed) * (1-error) / accelLength);
			} else if(error < deaccelLength) {
				speed = minSpeed + ((maxSpeed - minSpeed) * error / deaccelLength);
			} else {
				speed = maxSpeed;
			}

			Timer.delay(0.01);
		} while(degreesTurned < degrees);
		Timer.delay(0.02);
		setLeft(0);
		setRight(0);
	}
}
