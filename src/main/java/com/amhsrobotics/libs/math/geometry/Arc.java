package com.amhsrobotics.libs.math.geometry;

public class Arc {
	private Position center;
	private double radius;
	
	public Arc(Position center, double radius){
		
		this.center = center;
		this.radius = radius;
	}
	
	public Position getCenter() {
		return center;
	}
	
	public void setCenter(Position center) {
		this.center = center;
	}
	
	public double getRadius() {
		return radius;
	}
	
	public void setRadius(double radius) {
		this.radius = radius;
	}
}
