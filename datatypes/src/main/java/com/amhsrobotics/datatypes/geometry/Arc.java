package com.amhsrobotics.datatypes.geometry;

import com.amhsrobotics.datatypes.positioning.Position;
import com.amhsrobotics.datatypes.positioning.Rotation;
import com.amhsrobotics.datatypes.positioning.Transform;

import java.nio.file.attribute.AclEntry;

/**
 * Represents a 2d arc segment on a standard cartesian coordinate plane with two end points.
 */
public class Arc {
	
	private Circle circle;
	private Position startPoint;
	private Position endPoint;
	private Position intermediatePoint;
	
	/**
	 * Constructs an arc given a {@link Circle} and two {@link Position} end points.
	 *
	 * @param circle the {@link Circle} of the arc
	 * @param startPoint the starting {@link Position} of the arc
	 * @param endPoint the ending {@link Position} of the arc
	 * @param intermediatePoint a {@link Position} in between the start and end point of the arc used to determine
	 *                             which way the arc is filled in.
	 */
	public Arc(Circle circle, Position startPoint, Position endPoint, Position intermediatePoint){
		this.circle = circle;
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.intermediatePoint = intermediatePoint;
	}
	
	/**
	 * Determines whether or not a point is on this {@link Arc} segment.
	 *
	 * This checks for both if the point is on the circle and if the point is within the defining end points of the arc.
	 *
	 * @param point the {@link Position} to determine if it is on the arc or not
	 * @return whether or not <code>point</code> is on this {@link Arc} segment.
	 */
	public boolean isOnArc(Position point){
		double intermediateToStartDist = intermediatePoint.distance(startPoint);
		double intermediateToEndDist = intermediatePoint.distance(endPoint);
		double intermediateToPoint = intermediatePoint.distance(point);
		
		boolean isWithinPoints = intermediateToPoint <= intermediateToStartDist || intermediateToPoint <= intermediateToEndDist;
		
		System.out.println(isWithinPoints + " " + circle.isOnCircle(point) + " " + intermediateToPoint + " " + intermediateToStartDist + " " + intermediateToEndDist);
		
		return circle.isOnCircle(point) && isWithinPoints;
	}
	
	public Circle getCircle() {
		return circle;
	}
	
	public void setCircle(Circle circle) {
		this.circle = circle;
	}
	
	public Position getStartPoint() {
		return startPoint;
	}
	
	/**
	 * Sets the start point of the segment.
	 *
	 * @param startPoint the starting {@link Position}
	 */
	public void setStartPoint(Position startPoint) {
		this.startPoint = startPoint;
	}
	
	public Position getEndPoint() {
		return endPoint;
	}
	
	/**
	 * Sets the ending point of the segment.
	 *
	 * @param endPoint the ending {@link Position}
	 */
	public void setEndPoint(Position endPoint) {
		this.endPoint = endPoint;
	}
	
	
	public Position getIntermediatePoint() {
		return intermediatePoint;
	}
	
	/**
	 * Sets the intermediate point of the segment. This is a point within the start and end point so the arc knows
	 * which way it is facing.
	 *
	 * @param intermediatePoint the intermediate {@link Position}
	 */
	public void setIntermediatePoint(Position intermediatePoint) {
		this.intermediatePoint = intermediatePoint;
	}
}
