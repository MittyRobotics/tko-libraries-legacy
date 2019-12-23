package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;

public class PathVelocityController {
	private final VelocityConstraints velocityConstraints;
	private final double startVelocity;
	private final double endVelocity;
	SafeVelocityController safeVelocityController;
	
	
	public PathVelocityController(VelocityConstraints velocityConstraints, double startVelocity, double endVelocity) {
		this.velocityConstraints = velocityConstraints;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
		safeVelocityController = new SafeVelocityController(velocityConstraints);
	}
	
	public double getVelocity(double currentVelocity, double distanceToEnd, double deltaTime) {
		double maxDistanceVelocity = Math.sqrt(2 * velocityConstraints.getMaxDeceleration() * distanceToEnd);
		double desiredVelocity = Math.min(velocityConstraints.getMaxVelocity(), maxDistanceVelocity);
		
		return safeVelocityController.getVelocity(currentVelocity, desiredVelocity, deltaTime);
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
	
	public double getStartVelocity() {
		return startVelocity;
	}
	
	public double getEndVelocity() {
		return endVelocity;
	}
}
