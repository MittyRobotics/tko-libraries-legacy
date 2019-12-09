package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.positioning.Position;

/**
 * Represents a 2d arc segment on a standard cartesian coordinate plane with two end points.
 */
public class ArcSegment extends Circle {
	
	private Position startPoint;
	private Position endPoint;
	private Position intermediatePoint;
	
	/**
	 * Constructs an arc given two {@link Position} end points and an intermediate {@link Position}.
	 *
	 * @param startPoint        the starting {@link Position} of the arc
	 * @param endPoint          the ending {@link Position} of the arc
	 * @param intermediatePoint a {@link Position} in between the start and end point of the arc used to determine
	 *                          which way the arc is filled in and defines the {@link Circle}.
	 */
	public ArcSegment(Position startPoint, Position endPoint, Position intermediatePoint) {
		super(startPoint, intermediatePoint, endPoint);
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.intermediatePoint = intermediatePoint;
	}
	
	/**
	 * Determines whether or not a point is on this {@link ArcSegment} segment.
	 * <p>
	 * This checks for both if the point is on the circle and if the point is within the defining end points of the arc.
	 *
	 * @param point the {@link Position} to determine if it is on the arc or not
	 * @return whether or not <code>point</code> is on this {@link ArcSegment} segment.
	 */
	public boolean isOnSegment(Position point) {
		double intermediateToStartDist = intermediatePoint.distance(startPoint);
		double intermediateToEndDist = intermediatePoint.distance(endPoint);
		double intermediateToPoint = intermediatePoint.distance(point);
		
		boolean isWithinPoints = intermediateToPoint <= intermediateToStartDist || intermediateToPoint <= intermediateToEndDist;
		
		System.out.println(isWithinPoints + " " + isOnCircle(point) + " " + intermediateToPoint + " " + intermediateToStartDist + " " + intermediateToEndDist);
		
		return isOnCircle(point) && isWithinPoints;
	}
	
	public Position getStartPoint() {
		return startPoint;
	}
	
	public Position getEndPoint() {
		return endPoint;
	}
	
	public Position getIntermediatePoint() {
		return intermediatePoint;
	}
	
}
