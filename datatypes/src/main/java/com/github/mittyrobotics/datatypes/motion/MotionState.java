package com.github.mittyrobotics.datatypes.motion;


public class MotionState {
	private double position;
	private double velocity;
	private double acceleration;
	private double t;
	
	public MotionState(double velocity) {
		this.velocity = velocity;
	}
	
	public MotionState(double position, double velocity) {
		this.position = position;
		this.velocity = velocity;
	}
	
	public MotionState(double position, double velocity, double acceleration, double t) {
		this.position = position;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.t = t;
	}
	
	public double getPosition() {
		return position;
	}
	
	public void setPosition(double position) {
		this.position = position;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}
	
	public double getAcceleration() {
		return acceleration;
	}
	
	public void setAcceleration(double acceleration) {
		this.acceleration = acceleration;
	}
	
	public double getT() {
		return t;
	}
	
	public void setT(double t) {
		this.t = t;
	}
	
	@Override
	public String toString() {
		return String.format("MotionState(pos: %s, vel: %s, accel: %s, time: %s )", position, velocity, acceleration, t);
	}
}
