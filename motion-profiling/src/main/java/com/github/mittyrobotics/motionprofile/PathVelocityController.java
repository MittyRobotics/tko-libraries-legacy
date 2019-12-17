package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;

public class PathVelocityController {
	private final VelocityConstraints velocityConstraints;
	private final double startVelocity;
	private final double endVelocity;
	private final double minSlowdownVelocity;
	private final double curvatureSlowdownGain;
	SafeVelocityController safeVelocityController;
	
	public PathVelocityController(VelocityConstraints velocityConstraints, double startVelocity, double endVelocity, double minSlowdownVelocity, double curvatureSlowdownGain){
		this.velocityConstraints = velocityConstraints;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
		this.minSlowdownVelocity = minSlowdownVelocity;
		this.curvatureSlowdownGain = curvatureSlowdownGain;
		safeVelocityController = new SafeVelocityController(velocityConstraints);
	}
	
	public double getVelocity(double currentVelocity, double distanceToEnd, double curvature, double deltaTime){
		double maxDistanceVelocity = Math.sqrt(2*velocityConstraints.getMaxDeceleration()*distanceToEnd);
		double maxCurvatureVelocity = Math.max(curvatureSlowdownGain/curvature,minSlowdownVelocity);
		double desiredVelocity = Math.min(velocityConstraints.getMaxVelocity(),Math.min(maxCurvatureVelocity,maxDistanceVelocity));
		
		return safeVelocityController.getVelocity(currentVelocity,desiredVelocity,deltaTime);
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
	
	public double getCurvatureSlowdownGain() {
		return curvatureSlowdownGain;
	}
	
	public double getMinSlowdownVelocity() {
		return minSlowdownVelocity;
	}
}
