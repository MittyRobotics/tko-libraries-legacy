package com.github.mittyrobotics.datatypes.motion;

public class VelocityConstraints {
	private double maxAcceleration;
	private double maxDeceleration;
	private double maxVelocity;

	public VelocityConstraints(double maxAcceleration, double maxDeceleration, double maxVelocity) {
		this.maxAcceleration = Math.abs(maxAcceleration);
		this.maxDeceleration = Math.abs(maxDeceleration);
		this.maxVelocity = Math.abs(maxVelocity);
	}
	
	public double getMaxAcceleration() {
		return maxAcceleration;
	}
	
	public void setMaxAcceleration(double maxAcceleration) {
		this.maxAcceleration = maxAcceleration;
	}
	
	public double getMaxDeceleration() {
		return maxDeceleration;
	}
	
	public void setMaxDeceleration(double maxDeceleration) {
		this.maxDeceleration = maxDeceleration;
	}
	
	public double getMaxVelocity() {
		return maxVelocity;
	}
	
	public void setMaxVelocity(double maxVelocity) {
		this.maxVelocity = maxVelocity;
	}
}
