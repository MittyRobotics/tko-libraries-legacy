package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;

import java.util.Optional;

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
	 * @param slope      the slope
	 * @param yIntercept the y intercept y value
	 */
	public Line(double slope, double yIntercept) {
		this.slope = slope;
		this.yIntercept = yIntercept;
		this.firstPoint = new Position(0, yIntercept);
		this.secondPoint = new Position(1, yIntercept + slope);
	}
	
	/**
	 * Constructs a {@link Line} given two {@link Position} points
	 *
	 * @param firstPoint  the first {@link Position}
	 * @param secondPoint the second {@link Position}
	 */
	public Line(Position firstPoint, Position secondPoint) {
		this.firstPoint = firstPoint;
		this.secondPoint = secondPoint;
		this.slope = (firstPoint.getY() - secondPoint.getY()) / (firstPoint.getX() - secondPoint.getX());
		if (Double.isInfinite(this.slope)) {
			this.slope = 2e16;
		}
		this.yIntercept = firstPoint.getY() - (slope * firstPoint.getX());
	}
	
	/**
	 * Finds the intersection {@link Position} between this {@link Line} and <code>other</code>.
	 * <p>
	 * If the two lines are parallel, it will return null.
	 *
	 * @return the intersection {@link Position} between the two {@link Line}s.
	 */
	public Optional<Position> getIntersection(Line other) {
		double m1 = getSlope();
		double m2 = other.getSlope();
		double b1 = getYIntercept();
		double b2 = other.getYIntercept();
		
		//If the lines are parallel, return null
		if (m1 == m2) {
			return Optional.empty();
		}
		
		double x = (b2 - b1) / (m1 - m2);
		double y = m1 * x + b1;
		
		return Optional.of(new Position(x, y));
	}
	
	/**
	 * Finds the closest point on this {@link Line} to the <code>referencePosition</code>.
	 * <p>
	 * This is done by finding the line parallel to this line that intersects with the <code>referenceTransform</code>.
	 * https://www.desmos.com/calculator/trqlffx7ha
	 *
	 * @param referencePosition the {@link Position} to find the closest point to.
	 * @return the closest {@link Position} to the <code>referencePosition</code>.
	 */
	public Position getClosestPoint(Position referencePosition) {
		double m = getSlope();
		
		//Get parallel slope
		double m1 = -1 / m;
		
		//Create parallel line from position
		Line parallelLine = getParallelLine(referencePosition);
		
		//Return the intersection between the two lines
		return getIntersection(parallelLine).get();
	}
	
	/**
	 * Returns the angle of this line in the form of a {@link Rotation}.
	 *
	 * @return the {@link Rotation} of this line.
	 */
	public Rotation getLineAngle(){
		return new Rotation(Math.toDegrees(Math.atan2(getSlope(),1)));
	}
	
	/**
	 * Returns the parallel {@link Line} to this {@link Line} that passes through the <code>referencePosition</code>.
	 *
	 * @param referencePosition the {@link Position} that the parallel {@link Line} passes through.
	 * @return the parallel {@link Line} to this {@link Line}.
	 */
	public Line getParallelLine(Position referencePosition) {
		//Get parallel slope
		double m1 = -1 / getSlope();
		if (Double.isInfinite(m1)) {
			return new Line(referencePosition, referencePosition.add(new Position(0, 1)));
		} else {
			return new Line(referencePosition, new Position(1, m1).add(referencePosition));
		}
	}
	
	/**
	 * Finds which side of a line the point is on
	 *
	 * @param point the {@link Position} to find which side of the {@link Line} it is on
	 * @return a -1 for right side, +1 for left side
	 */
	public double findSide(Position point) {
		double x = point.getX();
		double y = point.getY();
		double x1 = firstPoint.getX();
		double y1 = firstPoint.getY();
		double x2 = secondPoint.getX();
		double y2 = secondPoint.getY();
		return -Math.signum((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1));
	}
	
	/**
	 * Determines whether or not <code>point</code> is collinear with this {@link Line}.
	 *
	 * @param point the {@link Position} to determine whether or not it is collinear.
	 * @return whether or not the <code>point</code> is collinear with this {@link Line}.
	 */
	public boolean isCollinear(Position point) {
		return isCollinear(point, 0.001);
	}
	
	/**
	 * Determines whether or not <code>point</code> is collinear with this {@link Line} given a <code>tolerance</code>.
	 *
	 * @param point     the {@link Position} to determine whether or not it is collinear.
	 * @param tolerance the tolerance for how much the point can be off the {@link Line} to be classified as collinear.
	 * @return whether or not the <code>point</code> is collinear with this {@link Line}.
	 */
	public boolean isCollinear(Position point, double tolerance) {
		double x1 = firstPoint.getX();
		double y1 = firstPoint.getY();
		double x2 = point.getX();
		double y2 = point.getY();
		double x3 = secondPoint.getX();
		double y3 = secondPoint.getY();
		
		double collinear = (y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1);
		
		return Math.abs(collinear) < tolerance;
	}
	
	public Position getFirstPoint() {
		return firstPoint;
	}
	
	public Position getSecondPoint() {
		return secondPoint;
	}
	
	public double getSlope() {
		return slope;
	}
	
	public double getYIntercept() {
		return yIntercept;
	}
}
