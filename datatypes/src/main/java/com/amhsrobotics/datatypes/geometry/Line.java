package com.amhsrobotics.datatypes.geometry;

import com.amhsrobotics.datatypes.positioning.Position;

/**
 * Represents a 2d line on a standard cartesian coordinate plane going infinitely in both directions.
 */
public class Line {
	private Position firstPoint;
	private Position secondPoint;
	private double slope;
	private double yIntercept;
	
	/**
	 * Constructs a {@link Line} given a slope and a y intercept
	 *
	 * @param slope the slope
	 * @param yIntercept the y intercept y value
	 */
	public Line(double slope, double yIntercept){
		this.slope = slope;
		this.yIntercept = yIntercept;
		this.firstPoint = new Position(0,yIntercept);
		this.secondPoint = new Position(1,yIntercept+slope);
	}
	
	/**
	 * Constructs a {@link Line} given two {@link Position} points
	 *
	 * @param firstPoint the first {@link Position}
	 * @param secondPoint the second {@link Position}
	 */
	public Line(Position firstPoint, Position secondPoint){
		this.firstPoint = firstPoint;
		this.secondPoint = secondPoint;
		this.slope = (firstPoint.getY()-secondPoint.getY())/(firstPoint.getX()-secondPoint.getX());
		this.yIntercept = firstPoint.getY()-(slope*firstPoint.getX());
	}
	
	/**
	 * Finds the intersection {@link Position} between this {@link Line} and <code>other</code>.
	 *
	 * If the two lines are parallel, it will return null.
	 *
	 * @return the intersection {@link Position} between the two {@link Line}s.
	 */
	public Position getIntersection(Line other){
		double m1 = getSlope();
		double m2 = other.getSlope();
		double b1 = getYIntercept();
		double b2 = other.getYIntercept();
		
		//If the lines are parallel, return null
		if(m1 == m2){
			return null;
		}
		
		double x = (b2-b1)/(m1-m2);
		double y = m1*x+b1;
		
		return new Position(x,y);
	}
	
	/**
	 * Determines whether or not <code>point</code> is collinear with this {@link Line}.
	 *
	 * @param point the {@link Position} to determine whether or not it is collinear.
	 * @return whether or not the <code>point</code> is collinear with this {@link Line}.
	 */
	public boolean isCollinear(Position point){
		return isCollinear(point,0.001);
	}
	
	/**
	 * Determines whether or not <code>point</code> is collinear with this {@link Line} given a <code>tolerance</code>.
	 *
	 * @param point the {@link Position} to determine whether or not it is collinear.
	 * @param tolerance the tolerance for how much the point can be off the {@link Line} to be classified as collinear.
	 * @return whether or not the <code>point</code> is collinear with this {@link Line}.
	 */
	public boolean isCollinear(Position point, double tolerance){
		double x1 = firstPoint.getX();
		double y1 = firstPoint.getY();
		double x2 = point.getX();
		double y2 = point.getY();
		double x3 = secondPoint.getX();
		double y3 = secondPoint.getY();
		
		double collinear = (y2-y1)*(x3-x2) - (y3-y2)*(x2-x1);
		
		return Math.abs(collinear) < tolerance;
	}
	
	public Position getFirstPoint(){
		return firstPoint;
	}
	
	public Position getSecondPoint(){
		return secondPoint;
	}
	
	public double getSlope() {
		return slope;
	}
	
	public double getYIntercept() {
		return yIntercept;
	}
}
