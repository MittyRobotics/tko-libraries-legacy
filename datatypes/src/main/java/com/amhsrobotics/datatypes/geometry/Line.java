package com.amhsrobotics.datatypes.geometry;

import com.amhsrobotics.datatypes.positioning.Position;

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
