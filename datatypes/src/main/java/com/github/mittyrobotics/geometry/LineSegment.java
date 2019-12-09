package com.github.mittyrobotics.geometry;

import com.github.mittyrobotics.positioning.Position;

/**
 * Represents a 2d line segment on a standard cartesian coordinate plane with two end points.
 */
public class LineSegment {
	private Line line;
	private Position startPoint;
	private Position endPoint;
	
	public LineSegment(Line line, Position startPoint, Position endPoint){
		this.line = line;
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
		boolean withinStartAndEnd = Math.abs(point.distance(getStartPoint()) + point.distance(getEndPoint()) -
				getStartPoint().distance(getEndPoint())) < 0.001;
		return withinStartAndEnd && line.isCollinear(point);
	}
	
	public Line getLine() {
		return line;
	}
	
	public void setLine(Line line) {
		this.line = line;
	}
	
	public Position getStartPoint() {
		return startPoint;
	}
	
	public void setStartPoint(Position startPoint) {
		this.startPoint = startPoint;
	}
	
	public Position getEndPoint() {
		return endPoint;
	}
	
	public void setEndPoint(Position endPoint) {
		this.endPoint = endPoint;
	}
}
