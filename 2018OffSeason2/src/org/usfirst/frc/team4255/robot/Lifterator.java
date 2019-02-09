package org.usfirst.frc.team4255.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Lifterator {
	public static TalonSRX lift;
	public static TalonSRX clamp;
	public static Encoder liftEncoder;
	public static Encoder clampEncoder;
	public static DigitalInput topLimit;
	public static DigitalInput bottomLimit;
	public static int open, closed;
	private int caseStep;
	private boolean properlyset;
	
	public Lifterator(TalonSRX lift, Encoder liftEncoder, TalonSRX clamp, Encoder clampEncoder, 
			DigitalInput topLimit, DigitalInput bottomLimit) {
		this.lift = lift;
		this.clamp = clamp;
		this.liftEncoder = liftEncoder;
		this.clampEncoder = clampEncoder;
		this.topLimit = topLimit;
		this.bottomLimit = bottomLimit;
		caseStep = 0;
		clamp.setInverted(true);
		properlyset = false;
		open = -1900; 
		closed = 400;
		//liftEncoder.setDistancePerPulse(???);
		//liftEncoder.setDistancePerPulse(???);
	}
	
	public void reset() {
		caseStep = 0;
	}
	
	public boolean isSet() {
		return properlyset;
	}
	
	public void printLift() {
		int liftHeight = liftEncoder.getRaw();
		int clampHeight = clampEncoder.getRaw();
		int open = -1143, closed = 600;
		int gap = liftHeight-clampHeight;
		print();
		System.out.println("difference: "+gap);
	}
	
	public void setLift(double liftSpeed, boolean grab) {
		if (properlyset) {
			int liftHeight = liftEncoder.getRaw();
			int clampHeight = clampEncoder.getRaw();
			int gap = liftHeight-clampHeight;
			double calcLiftSpeed = 0.0, calcClampSpeed = 0.0;
			double topSpeed = 1.0, bottomSpeed = -1.0;
			if (topLimit.get()) {
				topSpeed = 0.0;
			} else if (liftHeight <= -5100) {
				topSpeed = 0.4;
			} else {
				topSpeed = 1.0;
			}
			
			if (liftHeight >= 7600) {
				bottomSpeed = 0.0;
			} else if (liftHeight >= 7000) {
				bottomSpeed = -0.3;
			} else {
				bottomSpeed = -1.0;
			}
			if (grab) {
				double diffspeed = (double)Math.abs(gap-closed)/2000.0 + 0.1;
				diffspeed = etc.constrain(diffspeed, -1.0, 1.0);
				if (gap <= closed) {
					calcLiftSpeed = liftSpeed-0.8*diffspeed;
					calcClampSpeed = liftSpeed+0.8*diffspeed;
				} else {
					calcLiftSpeed = liftSpeed+0.8*diffspeed;
					calcClampSpeed = liftSpeed-0.8*diffspeed;
				}
				calcLiftSpeed = etc.constrain(calcLiftSpeed, bottomSpeed, topSpeed);
				calcClampSpeed = etc.constrain(calcClampSpeed, bottomSpeed, topSpeed);
			} else {
				double diffspeed = (double)Math.abs(gap-open)/2000.0 + 0.1;
				diffspeed = etc.constrain(diffspeed, -1.0, 1.0);
				if (gap > open) {
					calcLiftSpeed = liftSpeed+0.8*diffspeed;
					calcClampSpeed = liftSpeed-0.8*diffspeed;
				} else {
					calcLiftSpeed = liftSpeed-0.8*diffspeed;
					calcClampSpeed = liftSpeed+0.8*diffspeed;
				}
				calcLiftSpeed = etc.constrain(calcLiftSpeed, bottomSpeed, topSpeed);
				calcClampSpeed = etc.constrain(calcClampSpeed, bottomSpeed, topSpeed);
			}
			lift.set(ControlMode.PercentOutput, calcLiftSpeed);
			clamp.set(ControlMode.PercentOutput, calcClampSpeed);
		} else {
			clamp.set(ControlMode.PercentOutput, 0.0);
			lift.set(ControlMode.PercentOutput, 0.0);
		}
	}
	
	public void modGap(int add) {
		open += add;
		closed += add;
	}
	
	public void print() {
		System.out.println("Lift: "+liftEncoder.getRaw()+" Clamp: "+clampEncoder.getRaw());
		System.out.println("bottom: "+bottomLimit.get()+" top: "+topLimit.get());
	}
	
	public boolean encoderInit() {
		switch (caseStep) {
		case 0:
			if (bottomLimit.get()) {
				clamp.set(ControlMode.PercentOutput, 1.0);
				lift.set(ControlMode.PercentOutput, 1.0);
			} else {
				resetClampEncoder();
				caseStep++;
			}
			break;
		case 1:
			clamp.set(ControlMode.PercentOutput, -0.2);
			lift.set(ControlMode.PercentOutput, -0.2);
			if (bottomLimit.get()) {
				caseStep++;
			}
			break;
		case 2:
			clamp.set(ControlMode.PercentOutput, 0.0);
			lift.set(ControlMode.PercentOutput, 1.0);
			if (!bottomLimit.get()) {
				lift.set(ControlMode.PercentOutput, 0.0);
				resetLiftEncoder();
				caseStep++;
			}
			break;
		case 3:
			clamp.set(ControlMode.PercentOutput, 0.0);
			lift.set(ControlMode.PercentOutput, 0.0);
			break;
		}
		if (caseStep >= 3) {
			properlyset = true;
			return true;
		}
		return false;
	}
	
	public void resetLiftEncoder() {liftEncoder.reset();}
	
	public void resetClampEncoder() {clampEncoder.reset();}
}
