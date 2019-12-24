/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.simulation.sim;

import com.github.mittyrobotics.simulation.motors.Motor;

public class ModelSystem {
	private final Motor motor;
	private double mass;
	private double gearRatio;
	private double wheelRadius;
	
	private double resistance;
	private double Kv;
	private double Kt;
	
	private double velocity;
	private double position;
	private double voltage;
	
	/**
	 * Constructs a system model given the {@link Motor} used in the system.
	 *
	 * @param motor the {@link Motor} used in the system.
	 */
	public ModelSystem(Motor motor) {
		this.motor = motor;
	}
	
	/**
	 * Initializes the system model.
	 *
	 * @param mass        the mass of the system (kg)
	 * @param gearRatio   the gear ratio of the system
	 * @param wheelRadius the radius of the wheel or pulley in the system (meters)
	 */
	public void initSystemModel(double mass, double gearRatio, double wheelRadius) {
		this.mass = mass;
		this.gearRatio = gearRatio;
		this.wheelRadius = wheelRadius;
		double numMotors = 1;
		
		double stallTorque = motor.getStallTorque();
		double stallCurrent = motor.getStallCurrent();
		double freeSpeed = motor.getFreeSpeed();
		double freeCurrent = motor.getFreeCurrent();
		this.resistance = 12 / stallCurrent;
		this.Kv = ((freeSpeed / 60.0 * 2.0 * Math.PI) / (12.0 - resistance * freeCurrent));
		this.Kt = (numMotors * stallTorque) / stallCurrent;
	}
	
	/**
	 * Updates the {@link ModelSystem} with a given voltage and delta time.
	 *
	 * @param voltage   the voltage applied to the motors of the {@link ModelSystem}.
	 * @param deltaTime the change in time since the last update call.
	 */
	public void updateModel(double voltage, double deltaTime) {
		this.voltage = voltage;
		double acceleration = getAcceleration(voltage);
		position += velocity * deltaTime;
		velocity += acceleration * deltaTime;
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
