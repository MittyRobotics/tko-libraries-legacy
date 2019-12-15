package com.github.mittyrobotics.simulation.motors;

/**
 * Motor class.
 * <p>
 * Data for specific motors can be found from: https://motors.vex.com/
 */
public class Motor {
	
	private final double stallTorque; //Newton meters
	private final double stallCurrent; //Amps
	private final double freeSpeed; //RPM
	private final double freeCurrent; //Amps
	
	public Motor(double stallTorque, double stallCurrent, double freeSpeed, double freeCurrent) {
		this.stallTorque = stallTorque;
		this.stallCurrent = stallCurrent;
		this.freeSpeed = freeSpeed;
		this.freeCurrent = freeCurrent;
	}
	
	public double getStallTorque() {
		return stallTorque;
	}
	
	public double getStallCurrent() {
		return stallCurrent;
	}
	
	public double getFreeSpeed() {
		return freeSpeed;
	}
	
	public double getFreeCurrent() {
		return freeCurrent;
	}
}
