package com.github.mittyrobotics.libs.util.geometry;

public class Line {
	
	private Transform transform;
	
	private Position p1;
	private Position p2;
	
	public Line(Transform transform){
		this.p1 = transform.getPosition();
		this.p2 = new Position(transform.getRotation().cos(), transform.getRotation().sin()).add(transform.getPosition());
		this.transform = transform;
	}
	
	public Line(Position p1, Position p2){
		this.p1 = p1;
		this.p2 = p2;
		this.transform = new Transform(new Position(p1.getX(),p1.getY()), new Rotation(Math.toDegrees(Math.atan2(p2.getY()-p1.getY(), p2.getX()-p1.getX()))));
	}
	
	
	/**
	 * Finds the point that intersects the line defined by this {@link Line} and the other {@link Line}.
	 *
	 * @return the {@link Position} of the intersecting point.
	 */
	public Position findLineIntersectionPoint(Line other){
		Line l1 = this;
		Line l2 = other;
		
		double m1 = l1.getSlope();
		double b1 = l1.getYIntercept();
		double m2 = l2.getSlope();
		double b2 = l2.getYIntercept();
		
		//Parallel lines, no intersection
		if(m1 == m2){
			return new Position();
		}
		
		System.out.println(m1 + " " + m2);
		
		double x = (b2-b1)/(m1-m2);
		double y = m1*x+b1;
		
		return new Position(x,y);
	}
	
	public boolean isColinear(Position point){
		return isColinear(point,0.01);
	}
	
	
	public Position[] intersectionPointsWithCircle(Arc other){
		double baX = p2.getX() - p1.getX();
		double baY = p2.getY() - p1.getY();
		double caX = other.getCenter().getX() - p1.getX();
		double caY = other.getCenter().getY() - p1.getY();
		
		double a = baX * baX + baY * baY;
		double bBy2 = baX * caX + baY * caY;
		double c = caX * caX + caY * caY - other.getRadius() * other.getRadius();
		
		double pBy2 = bBy2 / a;
		double q = c / a;
		
		double disc = pBy2 * pBy2 - q;
		
		if (disc < 0) {
			return new Position[]{};
		}

		double tmpSqrt = Math.sqrt(disc);
		double abScalingFactor1 = -pBy2 + tmpSqrt;
		double abScalingFactor2 = -pBy2 - tmpSqrt;
		
		Position pos1 = new Position(p1.getX() - baX * abScalingFactor1, p1.getY()
				- baY * abScalingFactor1);
		if (disc == 0) {
			return new Position[]{pos1};
		}
		Position pos2 = new Position(p1.getX() - baX * abScalingFactor2, p1.getY()
				- baY * abScalingFactor2);
		return new Position[]{pos1,pos2};
	}
	
	/**
	 * Determines whether the point falls on this line.
	 *
	 * @param point
	 * @return whether the point falls on this line.
	 */
	public boolean isColinear(Position point, double tolerance){
		double x1 = p1.getX();
		double y1 = p1.getY();
		double x2 = point.getX();
		double y2 = point.getY();
		double x3 = p2.getX();
		double y3 = p2.getY();
		
		double colinear = (y2-y1)*(x3-x2) - (y3-y2)*(x2-x1);
		return Math.abs(colinear) < tolerance;
	}
	
	public Position getP1(){
		return p1;
	}
	
	public Position getP2(){
		return p2;
	}
	
	public double getSlope(){
		double slope = (getP2().getY()-getP1().getY())/(getP2().getX()- getP1().getX());
		if (Double.isInfinite(slope)) {
			return 999999999;
		}
		return slope;
	}
	
	public double getYIntercept(){
		return getP1().getY() - getSlope() * getP1().getX();
	}
	
	public Transform getTransform() {
		return transform;
	}
	
	public void setTransform(Transform transform) {
		this.transform = transform;
	}
	
	public void setP1(Position p1) {
		this.p1 = p1;
	}
	
	public void setP2(Position p2) {
		this.p2 = p2;
	}
}
