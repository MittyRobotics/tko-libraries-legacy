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

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.simulation.enums.ControlLoopType;

public class ControlLoop {
	
	double integral = 0;
	private double lastError = 0;
	private double lastMeasured = 0;
	private double totalError = 0;
	private ControlLoopType type;
	private double iterationTime;
	private double maxVoltage;
	private double Kv;
	private double Ka;
	private double Kp;
	private double Ki;
	private double Kd;
	private double Kf;
	
	public ControlLoop(ControlLoopType type, double maxVoltage, double iterationTime) {
		this.type = type;
		this.maxVoltage = maxVoltage;
		this.iterationTime = iterationTime;
	}
	
	public void setupVelocityController(double Kv, double Ka, double Kp) {
		this.Kv = Kv;
		this.Ka = Ka;
		this.Kp = Kp;
	}
	
	public void setupPIDFController(double Kp, double Ki, double Kd, double Kf) {
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.Kf = Kf;
	}
	
	public double update(double target, double measured) {
		target = target * Conversions.M_TO_IN;
		measured = measured * Conversions.M_TO_IN;
		switch (type) {
			case PIDF:
				return PIDFControl(target, measured);
			case VELOCITY:
				return velocityControl(target, measured);
		}
		return 0;
	}
	
	private double velocityControl(double target, double measured) {
		double voltage = 0;
		
		double FF = Kv * target + Ka * ((measured - lastMeasured) / iterationTime);
		
		double error = target - measured;
		
		double FB = Kp * error;
		
		voltage = FF + FB;
		
		voltage = Math.max(-maxVoltage, Math.min(maxVoltage, voltage));
		
		lastMeasured = measured;
		lastError = error;
		
		return voltage;
	}
	
	private double PIDFControl(double target, double measured) {
		double voltage = 0;
		
		double error = target - measured;
		
		integral = integral + error * iterationTime;
		double derivative = (error - lastError) / iterationTime;
		
		voltage = Kp * error + Ki * integral + Kd * derivative + Kf * target;
		
		voltage = Math.max(-maxVoltage, Math.min(maxVoltage, voltage));
		
		lastMeasured = measured;
		lastError = error;
		
		return voltage;
	}
}
