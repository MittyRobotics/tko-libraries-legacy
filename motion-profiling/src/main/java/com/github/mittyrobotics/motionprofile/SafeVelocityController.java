package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;

public class SafeVelocityController {
	
	private final VelocityConstraints velocityConstraints;
	
	public SafeVelocityController(VelocityConstraints velocityConstraints) {
		this.velocityConstraints = velocityConstraints;
	}
	
	public double getVelocity(double currentVelocity, double desiredVelocity, double deltaTime) {
		double finalVelocity = 0;
		if (currentVelocity >= desiredVelocity) {
			finalVelocity = currentVelocity - velocityConstraints.getMaxDeceleration() * deltaTime;
		} else {
			finalVelocity = currentVelocity + velocityConstraints.getMaxAcceleration() * deltaTime;
		}
		
		return Math.min(finalVelocity, desiredVelocity);
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
}
