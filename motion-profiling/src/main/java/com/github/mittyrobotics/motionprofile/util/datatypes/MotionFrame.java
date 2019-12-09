package com.github.mittyrobotics.motionprofile.util.datatypes;


public class MotionFrame {
	private double position;
	private double velocity;
	private double acceleration;
	private double t;
	
	public MotionFrame(double position, double velocity) {
		this.position = position;
		this.velocity = velocity;
	}
	
	public MotionFrame(double position, double velocity, double acceleration, double t) {
		this.position = position;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.t = t;
	}

	public double getPosition() {
		return position;
	}

	public double getVelocity() {
		return velocity;
	}

	public double getAcceleration() {
		return acceleration;
	}

	public double getT() {
		return t;
	}
}
