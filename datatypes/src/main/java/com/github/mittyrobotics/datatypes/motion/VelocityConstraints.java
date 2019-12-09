package com.github.mittyrobotics.datatypes.motion;

public class VelocityConstraints {
	private double maxAcceleration;
	private double maxDeceleration;
	private double maxVelocity;
	private double minVelocity;
	private double startVelocity;
	private double endVelocity;
	
	public VelocityConstraints(double maxAcceleration, double maxDeceleration, double maxVelocity) {
		this(maxAcceleration,maxDeceleration,maxVelocity,0,0,0);
	}
	
	public VelocityConstraints(double maxAcceleration, double maxDeceleration, double maxVelocity, double minVelocity, double startVelocity, double endVelocity) {
		this.maxAcceleration = Math.abs(maxAcceleration);
		this.maxDeceleration = Math.abs(maxDeceleration);
		this.maxVelocity = Math.abs(maxVelocity);
		this.minVelocity = minVelocity;
		this.startVelocity = Math.max(Math.min(startVelocity, maxVelocity),-maxVelocity);
		this.endVelocity = Math.max(Math.min(endVelocity, maxVelocity),-maxVelocity);
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
	
	public double getStartVelocity() {
		return startVelocity;
	}
	
	public void setStartVelocity(double startVelocity) {
		this.startVelocity = startVelocity;
	}
	
	public double getEndVelocity() {
		return endVelocity;
	}
	
	public void setEndVelocity(double endVelocity) {
		this.endVelocity = endVelocity;
	}
	
	public double getMinVelocity() {
		return minVelocity;
	}
	
	public void setMinVelocity(double minVelocity) {
		this.minVelocity = minVelocity;
	}
}
