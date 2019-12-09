package com.github.mittyrobotics.motionprofile.util.datatypes;


import com.github.mittyrobotics.motionprofile.util.Function;

public class MotionSegment {
	private double t;
	private double distance;
	private Function f;
	
	public MotionSegment(double t, double distance, Function f) {
		this.t = t;
		this.distance = distance;
		this.f = f;
	}
	
	public MotionSegment(double t) {
		this.t = t;
	}
	
	public double getTime() {
		return t;
	}
	
	public void setTime(double t) {
		this.t = t;
	}
	
	public double getDistance() {
		return distance;
	}
	
	public Function getF() {
		return f;
	}
	
	public void setF(Function f) {
		this.f = f;
	}
}