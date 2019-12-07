package com.amhsrobotics.datatypes.libs.auton.motionprofile;

import com.amhsrobotics.datatypes.libs.datatypes.VelocityConstraints;

public class LimitVelocityMotion {
	
	private static LimitVelocityMotion instance = new LimitVelocityMotion();
	
	public static LimitVelocityMotion getInstance(){
		return instance;
	}
	
	public double limitVelocity(double currentVelocity, double desiredVelocity, double deltaTime, VelocityConstraints constraints){
		double velocityError = desiredVelocity-currentVelocity;
	
		if(velocityError > 0){
			if(Math.abs(velocityError) < constraints.getMaxAcceleration()*deltaTime){
				return desiredVelocity;
			}
			else{
				return currentVelocity + constraints.getMaxAcceleration()*deltaTime;
			}
		}
		else{
			if(Math.abs(velocityError) < constraints.getMaxDeceleration()*deltaTime){
				return desiredVelocity;
			}
			else{
				return currentVelocity - constraints.getMaxDeceleration()*deltaTime;
			}
		}
	}
}
