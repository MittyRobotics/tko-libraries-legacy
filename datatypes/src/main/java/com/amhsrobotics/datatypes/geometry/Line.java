package com.amhsrobotics.datatypes.geometry;

public class Line {
	private double slope;
	private double yIntercept;
	
	public Line(double slope, double yIntercept){
		this.slope = slope;
		this.yIntercept = yIntercept;
	}
	
	public double getSlope() {
		return slope;
	}
	
	public double getYIntercept() {
		return yIntercept;
	}
}
