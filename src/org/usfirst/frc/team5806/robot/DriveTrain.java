package org.usfirst.frc.team5806.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
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
    
    double lEnc, rEnc, lDistSpeed, lRateAvg, lRateAvg2, rRateAvg, rRateAvg2, rDistSpeed, lastLTicks, lastRTicks, lastUpdate, lastLSpeed, lBaseSpeed, rBaseSpeed, lastRSpeed;
    
    Encoder lEncoder;
    Encoder rEncoder;
    VictorSP[] motors = new VictorSP[2];
   

    AHRS ahrs;

    public DriveTrain(int left, int right) {
        lEncoder = new Encoder(0, 1);
        rEncoder = new Encoder(2, 3);
        motors[0] = new VictorSP(left);
        motors[0].setInverted(true);
        motors[1] = new VictorSP(right);

        for (VictorSP motor:motors) motor.set(0);

        ahrs = new AHRS(SPI.Port.kMXP);  	
        ahrs.reset();
        
        lEncoder.reset();
        lEncoder.setDistancePerPulse(1);
        rEncoder.reset();
        rEncoder.setDistancePerPulse(1);
        lEncoder.setReverseDirection(true);
        
        lastUpdate = Timer.getFPGATimestamp();
        lastLTicks = lEncoder.get();
        lastRTicks = rEncoder.get();
        
        lBaseSpeed = 0.0;
        rBaseSpeed = 0.0;
        lastLSpeed = 0.0;
        lastRSpeed = 0.0;
        lRateAvg = 0.0;
        rRateAvg = 0.0;
    }
    
    private void setLeft(double speed) {
    		motors[0].set(speed);
    }
    
    private void setRight(double speed) {
		motors[1].set(speed);
    }
    
    // Don't try negative speed for now
    public void driveFoward(double maxSpeed, double minSpeed, double accelLength, double deaccelLength, double distance, double direction) {
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

	public void updateSubsystem() {
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
}
