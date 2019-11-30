package com.amhsrobotics.libs.util.geometry;

public class Arc {
	private Position center;
	private double radius;
	
	private double minAngle;
	private double maxAngle;
	
	public Arc(Position center, double radius){
		
		this.center = center;
		this.radius = radius;
	}
	
	public Arc(Position p1, Position p2, Position p3){
		 Arc arc = p1.findIntersectingArc(p2, p3);
		 this.center = arc.center;
		 this.radius = arc.radius;
		 double minAngle = center.angleTo(p1);
		 double maxAngle = center.angleTo(p3);
		 this.minAngle = Math.min(minAngle,maxAngle);
		 this.maxAngle = Math.max(minAngle,maxAngle);
	}
	
	
	/**
	 * Determines whether the point lies on this circle.
	 *
	 * @param point
	 * @return whether the point is on this circle
	 */
	public boolean isOnCircle(Position point){
		return (point.distance(center)-radius) < 2e-16;
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
	
	public double getMinAngle() {
		return minAngle;
	}
	
	public void setMinAngle(double minAngle) {
		this.minAngle = minAngle;
	}
	
	public double getMaxAngle() {
		return maxAngle;
	}
	
	public void setMaxAngle(double maxAngle) {
		this.maxAngle = maxAngle;
	}
}
