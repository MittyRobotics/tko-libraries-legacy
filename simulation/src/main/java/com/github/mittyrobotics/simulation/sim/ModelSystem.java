package com.github.mittyrobotics.simulation.sim;

import com.github.mittyrobotics.simulation.motors.Motor;

public class ModelSystem {
	private final Motor motor;
	private double mass;
	private double gearRatio;
	private double wheelRadius;
	private double periodTime;
	
	private double resistance;
	private double Kv;
	private double Kt;
	
	private double velocity;
	private double position;
	private double voltage;
	
	public ModelSystem(Motor motor) {
		this.motor = motor;
	}
	
	public void initSystemModel(double mass, double gearRatio, double wheelRadius, double periodTime) {
		this.mass = mass;
		this.gearRatio = gearRatio;
		this.wheelRadius = wheelRadius;
		this.periodTime = periodTime;
		double numMotors = 1;
		
		double stallTorque = motor.getStallTorque();
		double stallCurrent = motor.getStallCurrent();
		double freeSpeed = motor.getFreeSpeed();
		double freeCurrent = motor.getFreeCurrent();
		this.resistance = 12 / stallCurrent;
		this.Kv = ((freeSpeed / 60.0 * 2.0 * Math.PI) / (12.0 - resistance * freeCurrent));
		this.Kt = (numMotors * stallTorque) / stallCurrent;
	}
	
	public void updateModel(double voltage) {
		this.voltage = voltage;
		double acceleration = getAcceleration(voltage);
		position += velocity * periodTime;
		velocity += acceleration * periodTime;
	}
	
	/**
	 * Calculates acceleration from a given voltage following the input motor model.
	 *
	 * @param voltage
	 * @return
	 */
	private double getAcceleration(double voltage) {
		return -Kt * gearRatio * gearRatio / (Kv * resistance * wheelRadius * wheelRadius * mass) * velocity + gearRatio * Kt / (resistance * wheelRadius * mass) * voltage;
	}
	
	public Motor getMotor() {
		return motor;
	}
	
	public double getMass() {
		return mass;
	}
	
	public double getGearRatio() {
		return gearRatio;
	}
	
	public double getWheelRadius() {
		return wheelRadius;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}
	
	public double getPosition() {
		return position;
	}
	
	public void setPosition(double position) {
		this.position = position;
	}
	
	public double getVoltage() {
		return voltage;
	}
	
	public void setVoltage(double voltage) {
		this.voltage = voltage;
	}
}
