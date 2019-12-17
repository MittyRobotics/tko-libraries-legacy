package com.github.mittyrobotics.path.following.controllers;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.path.following.util.DifferentialDriveKinematics;

public class RamseteController {
	public static double DEFAULT_AGGRESSIVE_GAIN = 2.0;
	public static double DEFAULT_DAMPING_GAIN = 0.7;
	private static RamseteController instance = new RamseteController();
	
	private double aggressiveGain; //(x > 0), makes turning more aggressive
	private double dampingGain; //(0 < x < 1) provides more damping
	
	private RamseteController() {
		setGains();
	}
	
	public static RamseteController getInstance() {
		return instance;
	}
	
	public void setGains() {
		setGains(DEFAULT_AGGRESSIVE_GAIN, DEFAULT_DAMPING_GAIN);
	}
	
	public void setGains(double aggressiveGain, double dampingGain) {
		this.aggressiveGain = aggressiveGain;
		this.dampingGain = dampingGain;
	}
	
	public DrivetrainVelocities calculate(Transform robotTransform, Transform desiredTransform, double robotVelocity, double turningRadius) {
		//Get the transform error in meters.
		Transform error = desiredTransform.relativeTo(robotTransform).inToM();
		//Calculate the angular velocity in radians per second given the turning radius and the robot velocity.
		double angularVelocity = robotVelocity / turningRadius;
		//Calculate linear velocity in meters per second given robot velocity in inches per second
		double linearVelocity = robotVelocity * Conversions.IN_TO_M;
		
		double eX = error.getPosition().getX();
		double eY = error.getPosition().getY();
		double eTheta = error.getRotation().getHeading();
		
		//Calculate the Ramsete k value
		double k = 2.0 * dampingGain * Math.sqrt(Math.pow(angularVelocity, 2) + aggressiveGain * Math.pow(linearVelocity, 2));
		
		//Calculate the adjusted linear velocity from the Ramsete algorithm
		double adjustedLinearVelocity = linearVelocity * error.getRotation().cos() + k * eX;
		//Convert linear velocity back into inches per second
		adjustedLinearVelocity = adjustedLinearVelocity * Conversions.M_TO_IN;
		
		//Calculate the adjusted angular velocity from the Ramsete algorithm (stays in radians per second)
		double adjustedAngularVelocity = angularVelocity + k * eTheta + aggressiveGain * linearVelocity * error.getRotation().sinc() * eY;
		
		//Use differential drive kinematics given linear velocity in inches per second and angular velocity in radians per second
		return DifferentialDriveKinematics.getInstance().calculateFromAngularVelocity(adjustedLinearVelocity, adjustedAngularVelocity);
	}
	
	public double getAggressiveGain() {
		return aggressiveGain;
	}
	
	public double getDampingGain() {
		return dampingGain;
	}
}
