package com.amhsrobotics.datatypes.libs.util.geometry;

public class Arc {
	private Position center;
	private double radius;
	
	private double minAngle;
	private double maxAngle;
	
	public Arc(Position center, double radius){
		this(center,radius,0,360);
	}
	
	public Arc(Position center, double radius, double minAngle, double maxAngle){
		this.center = center;
		this.radius = radius;
		this.minAngle = minAngle;
		this.maxAngle = maxAngle;
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
	 * Finds the two, one, or zero intersection points between this circle and the other circle
	 * @param other
	 * @return
	 */
	public Position[] intersectionPointsWithCircle(Arc other){

		double x0 = getCenter().getX();
		double y0 = getCenter().getY();
		double x1 = other.getCenter().getX();
		double y1 = other.getCenter().getY();
		double r0 = getRadius();
		double r1 = other.getRadius();
		
		double d = Math.hypot(x1 - x0, y1 - y0);
		
		if (d <= r0 + r1 && d >= Math.abs(r1 - r0)) {
			double ex = (x1 - x0) / d;
			double ey = (y1 - y0) / d;
			
			double x = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
			double y = Math.sqrt(r0 * r0 - x * x);
			
			Position p1 = new Position(x0 + x * ex - y * ey,y0 + x * ey + y * ex);
			Position p2 = new Position(x0 + x * ex + y * ey,y0 + x * ey - y * ex);
			
			return new Position[]{p1,p2};
		} else {
			// No Intersection
			return new Position[]{};
		}
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
