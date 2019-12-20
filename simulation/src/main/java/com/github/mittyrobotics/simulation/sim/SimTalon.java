package com.github.mittyrobotics.simulation.sim;

import com.github.mittyrobotics.datatypes.units.Conversions;

public class SimTalon implements Runnable {
	private ModelSystem model;
	private double voltage = 0;
	private SimTalon master;
	private boolean follower = false;
	
	private double Kp;
	private double Kd;
	private double Ki;
	private double Kf;
	
	private double maxPercent = 1;
	private double integral;
	private double lastError;
	private double lastMeasured;
	
	public SimTalon(ModelSystem modelSystem) {
		this.model = modelSystem;
	}
	
	public void set(double percent) {
		this.voltage = Math.min(Math.max(percent * 12, -12), 12);
	}
	
	private double PIDFControl(double target, double measured) {
		double voltage = 0;
		
		double error = target - measured;
		
		integral = integral + error * .01;
		double derivative = (error - lastError) / .01;
		
		voltage = Kp * error + Ki * integral + Kd * derivative + Kf * target;
		
		voltage = Math.max(-maxPercent, Math.min(maxPercent, voltage));
		
		lastMeasured = measured;
		lastError = error;
		
		return voltage;
	}
	
	public void setFollower(SimTalon master) {
		this.master = master;
		follower = true;
	}
	
	public void setPIDF(double Kp, double Kd, double Ki, double Kf) {
		this.Kp = Kp;
		this.Kd = Kd;
		this.Ki = Ki;
		this.Kf = Kf;
	}
	
	public void setMaxPercent(double maxPercent) {
		this.maxPercent = maxPercent;
	}
	
	/**
	 * Run method, updates the motor's position with the voltage
	 * <p>
	 * This is called every 10 ms
	 */
	@Override
	public void run() {
		while (true) {
			if (follower) {
				model.updateModel(master.getVoltage());
			} else {
				model.updateModel(voltage);
			}
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	public double getPosition() {
		return (double) model.getPosition() * Conversions.M_TO_IN ;
	}
	
	public void setPosition(double position) {
		set(PIDFControl(position, getPosition()));
	}
	
	public double getVelocity() {
		return (double) model.getVelocity() * Conversions.M_TO_IN ;
	}
	
	public void setVelocity(double velocity) {
		set(PIDFControl(velocity, getVelocity()));
	}
	
	public ModelSystem getModel() {
		return model;
	}
	
	public void setModel(ModelSystem model) {
		this.model = model;
	}
	
	public double getVoltage() {
		return voltage;
	}
	
	public double getKp() {
		return Kp;
	}
	
	public void setKp(double kp) {
		Kp = kp;
	}
	
	public double getKd() {
		return Kd;
	}
	
	public void setKd(double kd) {
		Kd = kd;
	}
	
	public double getKi() {
		return Ki;
	}
	
	public void setKi(double ki) {
		Ki = ki;
	}
	
	public double getKf() {
		return Kf;
	}
	
	public void setKf(double kf) {
		Kf = kf;
	}
}
