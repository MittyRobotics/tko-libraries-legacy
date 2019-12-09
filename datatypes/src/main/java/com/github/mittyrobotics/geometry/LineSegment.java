package com.github.mittyrobotics.geometry;

import com.github.mittyrobotics.positioning.Position;

/**
 * Represents a 2d line segment on a standard cartesian coordinate plane with two end points.
 */
public class LineSegment extends Line{
	private Position startPoint;
	private Position endPoint;
	
	public LineSegment(Position startPoint, Position endPoint){
		super(startPoint,endPoint);
		this.startPoint = startPoint;
		this.endPoint = endPoint;
	}
	
	/**
	 * Returns whether or not the <code>point</code> is on this line segment.
	 *
	 * This checks for both if the <code>point</code> is collinear with the line and if the <code>point</code> is
	 * within the start and end points of the segment.
	 *
	 * @param point the {@link Position} to determine if it is on the segment or not.
	 * @return whether or not the <code>point</code> is on this line segment.
	 */
	public boolean isOnSegment(Position point){
		boolean withinStartAndEnd = Math.abs(point.distance(getFirstPoint()) + point.distance(getSecondPoint()) -
				getFirstPoint().distance(getSecondPoint())) < 0.001;
		return withinStartAndEnd && isCollinear(point);
	}
}
