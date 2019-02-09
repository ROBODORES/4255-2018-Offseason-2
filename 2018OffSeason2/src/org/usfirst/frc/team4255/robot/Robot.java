package org.usfirst.frc.team4255.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {
	Timer time = new Timer();
	Timer liftTime = new Timer();
	
	Joystick jLeft = new Joystick (0);
    Joystick jRight = new Joystick (1);
    Joystick jSide = new Joystick (2);
    Joystick chooser = new Joystick (3);
	
    AHRS navX = new AHRS(SPI.Port.kMXP);
    AnalogInput sonar = new AnalogInput(0); //5v/512
    
    DigitalInput limitTop = new DigitalInput(4);
    DigitalInput limitBottom = new DigitalInput(5);
    
    TalonSRX leftDrive = new TalonSRX(1);
    TalonSRX leftFollow = new TalonSRX(2);
    TalonSRX rightDrive = new TalonSRX(4);
    TalonSRX rightFollow = new TalonSRX(5);
    
    TalonSRX lift = new TalonSRX(3);
    TalonSRX clamp = new TalonSRX(0);
    
    Drive drive = new Drive(leftDrive, FeedbackDevice.None, rightDrive, FeedbackDevice.CTRE_MagEncoder_Relative);
    NavDrive navDrive = new NavDrive(navX, drive);
    Route middleL = new Route(etc.middleL, drive, navDrive);
    Route middleR = new Route(etc.middleR, drive, navDrive);
    Route leftL = new Route(etc.leftL, drive, navDrive);
    Route leftR = new Route(etc.leftR, drive, navDrive);
    Route rightR = new Route(etc.rightR, drive, navDrive);
    Route rightL = new Route(etc.rightL, drive, navDrive);
    /*Route leftLS = new Route(etc.leftLS, drive, navDrive);
    Route rightRS = new Route(etc.rightRS, drive, navDrive);*/
    
	Encoder liftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder clampEncoder = new Encoder(2,3, false, Encoder.EncodingType.k4X);
	
	Lifterator lifterator = new Lifterator(lift, liftEncoder, clamp, clampEncoder, limitTop, limitBottom);
	
	CameraServer camserv;
	UsbCamera cam0;
	UsbCamera cam1;

	String sides;
	String position;
	int step = 0;
	boolean done = false;
	boolean stop = false;
	
	@Override
	public void robotInit() {
		//release.set(DoubleSolenoid.Value.kOff);
		
		liftEncoder.setDistancePerPulse(12.0/350.0);
		clampEncoder.setDistancePerPulse(12.0/100.0);
		
	    rightDrive.setInverted(true);
		rightFollow.setInverted(true);
		leftFollow.follow(leftDrive);
		rightFollow.follow(rightDrive);
		
		rightDrive.configPeakCurrentDuration(10, 0);
		rightDrive.configPeakCurrentLimit(55, 0);
		rightDrive.configContinuousCurrentLimit(45, 0);
		rightDrive.enableCurrentLimit(true);
		
		leftDrive.configPeakCurrentDuration(10, 0);
		leftDrive.configPeakCurrentLimit(55, 0);
		leftDrive.configContinuousCurrentLimit(45, 0);
		leftDrive.enableCurrentLimit(true);
		
		rightFollow.configPeakCurrentDuration(10, 0);
		rightFollow.configPeakCurrentLimit(55, 0);
		rightFollow.configContinuousCurrentLimit(45, 0);
		rightFollow.enableCurrentLimit(true);
		
		leftFollow.configPeakCurrentDuration(10, 0);
		leftFollow.configPeakCurrentLimit(55, 0);
		leftFollow.configContinuousCurrentLimit(45, 0);
		leftFollow.enableCurrentLimit(true);
		
		camserv = CameraServer.getInstance();
	    
	    cam0 = camserv.startAutomaticCapture(0);
	    //cam0.setResolution(640, 480);
	    //cam0.setFPS(5);
	    /*cam1 = camserv.startAutomaticCapture(1);
	    cam1.setResolution(1920, 1080);*/
	    //cam0.setFPS(30); //this probably won't work if you activate it
		time.start();
		liftTime.start();
	}

	@Override
	public void autonomousInit() {
		sides = DriverStation.getInstance().getGameSpecificMessage();
		if (chooser.getRawButton(1)) position = "Left";
		else if (chooser.getRawButton(2)) position = "Right";
		else position = "Middle";
		navX.reset();
		navDrive.reset();
		drive.reset();
		done = false;
		step = 0;
		middleL.setTo(0);
		middleR.setTo(0);
		leftL.setTo(0);
		leftR.setTo(0);
		rightR.setTo(0);
		rightL.setTo(0);
		//leftLS.setTo(0);
		time.reset();
		lifterator.reset();
		if (!lifterator.isSet()) {
			lifterator.reset();
			while (!lifterator.encoderInit());
		}
	}

	@Override
	public void autonomousPeriodic() {
		if (done) {
		switch (position) {
			case "Left":
				switch (sides.charAt(1)) {
				case 'L':
					switch (sides.charAt(0)) {
					case 'L': //Left side Left switch Left scale
						switch (step) {
						case 0:
							//boolean up = liftUp();
							boolean run = leftL.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
							break;
						}
						break;
					case 'R': //Left side Right switch Left scale
						switch (step) { //switch code
						case 0:
							//boolean up = liftUp();
							boolean run = leftR.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
							break;
						}
						
						/*switch (step) { //scale code
						case 0:
							boolean up = liftToScale();
							boolean run = leftLS.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (scaleDeploy()) step++;
							break;
						}*/
						break;
					}
					break;
				case 'R':
					switch (sides.charAt(0)) {
					case 'L': //Left side Left switch Right scale
						switch (step) {
						case 0:
							//boolean up = liftUp();
							boolean run = leftL.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
							break;
						}
						break;
					case 'R': //Left side Right switch Right scale
						switch (step) {
						case 0:
							//boolean up = liftUp();
							boolean run = leftR.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
							break;
						}
						break;
					}
					break;
				}
				break;
				
			case "Middle":
				switch (sides.charAt(0)) {
				case 'L':
					switch (step) {
					case 0:
						//boolean up = liftUp();
						boolean run = middleL.run();
						if (run) {
							step++;
						}
						break;
					case 1:
						deploy();
						break;
					}
					break;
				case 'R':
					switch (step) {
					case 0:
						//boolean up = liftUp();
						boolean run = middleR.run();
						if (run) {
							step++;
						}
						break;
					case 1:
						deploy();
						break;
					}
					break;
				}
				break;
			case "Right":
				switch (sides.charAt(1)) {
				case 'L':
					switch (sides.charAt(0)) {
					case 'L': //Right side Left switch Left scale
						switch (step) {
						case 0:
							//boolean up = liftUp();
							boolean run = rightL.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
							break;
						}
						break;
					case 'R': //Right side Right switch Left scale
						switch (step) {
						case 0:
							//boolean up = liftUp();
							boolean run = rightR.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
							break;
						}
						break;
					}
					break;
				case 'R':
					switch (sides.charAt(0)) {
					case 'L': //Right side Left switch Right scale
						/*switch (step) { //scale code
						case 0:
							boolean up = liftToScale();
							boolean run = rightRS.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (scaleDeploy()) step++;
							break;
						}*/
						
						switch (step) { //switch code
						case 0:
							//boolean up = liftUp();
							boolean run = rightL.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
						}
						break;
					case 'R': //Right side Right switch Right scale
						switch (step) {
						case 0:
							//boolean up = liftUp();
							boolean run = rightR.run();
							if (run) {
								step++;
							}
							break;
						case 1:
							deploy();
						}
						break;
					}
					break;
				}
				break;
			}
		} else { //failsafe initializer
			if (!lifterator.isSet()) {
				while (!lifterator.encoderInit());
			}
			done = true;
		}
	}
	
	@Override
	public void teleopInit() {
		time.reset();
		drive.zeroLeftDist();
		done = false;
		if (!lifterator.isSet()) {
			lifterator.reset();
			while (!lifterator.encoderInit());
		}
	}
	
	@Override
	public void teleopPeriodic() {
		System.out.println(sonar.getVoltage()*102.4);
		/*if (limitTop.get() && (-jSide.getY() < 0)) {
			lift.set(ControlMode.PercentOutput, 0.0);
			clamp.set(ControlMode.PercentOutput, 0.0);
		} else {
			lift.set(ControlMode.PercentOutput, jSide.getY()*0.5);
			clamp.set(ControlMode.PercentOutput, jSide.getY()*0.5+jSide.getX()*0.5);
		}*/
		
		lifterator.print();
		//if(!jSide.getRawButton(3)|| !jSide.getRawButton(5)) 
		lifterator.setLift(jSide.getY(), !jSide.getRawButton(1));
		
		if (jSide.getRawButton(5)) {
			lifterator.modGap(5);
		} else if (jSide.getRawButton(6)) {
			lifterator.modGap(-5);
		}
		
		/*if(jSide.getRawButton(2)) liftEncoder.reset(); 
		if(jSide.getRawButton(3)){
			clamp.set(ControlMode., arg1);
		}
		else{	
		}*/
		
		//System.out.println("TOP: " + limitTop.get() + " Bottom: " + limitBottom.get());
		//System.out.println(clampEncoder.getDistance());
		drive.setDrive(-jLeft.getY()*0.8, -jRight.getY()*0.8, false);
		//drive.singleJoystickDrive(-jRight.getY()*0.8, -jRight.getTwist()*0.8, false);
		
	}

	public void nextStep(){
		navDrive.reset();
		drive.reset();
		step++;
	}
	
	public boolean liftUp() {
		return lifterator.isSet();
	}
	
	public void deploy() {
		double deployDist = 20.0; //inches
		if (sonar.getVoltage()*102.4 <= deployDist) {
			if (liftTime.get() >= 1.0) {
				lifterator.setLift(0.0, false);
				drive.setDrive(0.0, 0.0, false);
			} else {
				drive.setDrive(0.3, 0.3, false);
			}
		} else {
			drive.setDrive(0.3, 0.3, false);
			liftTime.reset();
		}
	}
	
	public boolean setClamp(double toVal) {
		return true;
	}
	
	public void resetLift() {
		liftTime.reset();
		stop = false;
	}
	
	public boolean liftToScale() {
		return true;
	}
	
	public boolean scaleDeploy() {
		return true;
	}
}
