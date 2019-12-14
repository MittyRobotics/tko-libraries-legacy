package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

/**
 * Represents a 2d line segment on a standard cartesian coordinate plane with two end points.
 */
public class LineSegment extends Line {
	private Position startPoint;
	private Position endPoint;
	
	/**
	 * Constructs a line segment given a start and end {@link Position}.
	 *
	 * @param startPoint the start {@link Position}
	 * @param endPoint   the end {@link Position}
	 */
	public LineSegment(Position startPoint, Position endPoint) {
		super(startPoint, endPoint);
		this.startPoint = startPoint;
		this.endPoint = endPoint;
	}

	/**
	 * Gets the length of the line segment.
	 *
	 * @return the length of the line segment.
	 */
	public double getArcLength(){
		return startPoint.distance(endPoint);
	}
	
	/**
	 * Finds the closest point on this {@link LineSegment} to the <code>referencePosition</code> that is
	 * <code>distanceShift</code> away from the <code>referencePosition</code>.
	 * <p>
	 * This is done by finding the circle with a center of <code>referencePosition</code> that has a radius of
	 * <code>distanceShift</code> and finding the points that it intersects with this {@link Line}. It then gets the
	 * point that falls within the {@link LineSegment}.
	 *
	 * If two intersection points fall within the {@link LineSegment},
	 * it will pick the point closest to <code>distanceShift</code> away from the <code>referencePosition</code>.
	 *
	 * If no intersection points fall within the {@link LineSegment} or exist, it will return null.
	 *
	 * @param referencePosition the {@link Position} to find the closest point to.
	 * @param distanceShift     the distance away from the <code>referencePosition</code> to find the closest point to.
	 * @param roundMode
	 * @return the closest {@link Position} to the <code>referencePosition</code> that is <code>distanceShift</code> away.
	 */
	public Position getClosestPointOnSegment(Position referencePosition, double distanceShift, RoundMode roundMode) {
		//Get points that intersect the circle
		Position[] positions = new Circle(referencePosition,distanceShift).circleLineIntersection(this);
		
		//If no points intersect the the line and the circle, return the closest point to the reference point.
		if(positions.length == 0){
			Position pos = getClosestPoint(referencePosition);
			if(isOnSegment(pos)){
				return pos;
			}
			else{
				return null;
			}
		}
		
		double pointsOnSegment = 0;
		int latestPointIndex = 0;
		//Loop through all points and determine how many points intersect with the line segment
		for(int i = 0; i  < positions.length; i++){
			if(isOnSegment(positions[i])){
				pointsOnSegment ++;
				latestPointIndex = i;
			}
		}
		
		//If no points fall on the segment, return null.
		if(pointsOnSegment == 0){
			return null;
		}
		//If only one point falls on the segment, return that point.
		else if(pointsOnSegment == 1){
			return positions[latestPointIndex];
		}
		
		System.out.println(positions[0] + " " + positions[1]);
		
		//If two points fall on the segment, return the point closest to the distance shift
		double currentClosest = Double.NaN;
		Position currentClosestPosition = null;
		for (int i = 0; i < positions.length; i++) {
			if(roundMode == RoundMode.ROUND_DOWN){
				Position relative = new Transform(positions[i]).relativeTo(new Transform(referencePosition,referencePosition.angleTo(endPoint))).getPosition();
				if(relative.getX() < 0){
					currentClosest = positions[i].distance(referencePosition);
					currentClosestPosition = positions[i];
				}
			}
			else if(roundMode == RoundMode.ROUND_UP){
				Position relative = new Transform(positions[i]).relativeTo(new Transform(referencePosition,referencePosition.angleTo(endPoint))).getPosition();
				if(relative.getX() > 0){
					currentClosest = positions[i].distance(referencePosition);
					currentClosestPosition = positions[i];
				}
			}
			else {
				if (Double.isNaN(currentClosest) || Math.abs(positions[i].distance(referencePosition)-distanceShift) < currentClosest) {
					currentClosest = positions[i].distance(referencePosition);
					currentClosestPosition = positions[i];
				}
			}
		}
		
		return currentClosestPosition;
	}
	
	/**
	 * Returns whether or not the <code>point</code> is on this line segment.
	 * <p>
	 * This checks for both if the <code>point</code> is collinear with the line and if the <code>point</code> is
	 * within the start and end points of the segment.
	 *
	 * @param point the {@link Position} to determine if it is on the segment or not.
	 * @return whether or not the <code>point</code> is on this line segment.
	 */
	public boolean isOnSegment(Position point) {
		boolean withinStartAndEnd = Math.abs(point.distance(getFirstPoint()) + point.distance(getSecondPoint()) -
				getFirstPoint().distance(getSecondPoint())) < 0.001;
		return withinStartAndEnd && isCollinear(point);
	}
}
