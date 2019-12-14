package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

import java.util.Optional;

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
	public double getSegmentLength(){
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
	 * @param roundMode         the {@link RoundMode}
	 * @return the closest {@link Position} to the <code>referencePosition</code> that is <code>distanceShift</code> away.
	 */
	public Optional<Position> getClosestPointOnSegment(Position referencePosition, double distanceShift, RoundMode roundMode) {
		//Get the actual closest point on the line segment
		Position actualClosestPoint = getClosestPoint(referencePosition);
		
		//If the actual closest point is not on the segment, set it to the closest end point
		if (!isOnSegment(actualClosestPoint)) {
			actualClosestPoint = getClosestEndPoint(referencePosition);
		}
		
		//If the distance shift is zero, meaning we want to find the actual closest point to the reference position,
		//return the actual closest point on the segment.
		if (distanceShift == 0) {
			return Optional.ofNullable(actualClosestPoint);
		}
		
		//Get points that intersect the circle
		Position[] positions = new Circle(referencePosition,distanceShift).circleLineIntersection(this);
		
		//If no points intersect the two circles, return the actual closest point on the segment
		if (positions.length == 0) {
			return Optional.ofNullable(actualClosestPoint);
		}
		
		double pointsOnSegment = 0;
		int latestPointIndex = 0;
		//Loop through all points and determine how many points intersect with the line segment
		for (int i = 0; i < positions.length; i++) {
			if (isOnSegment(positions[i])) {
				pointsOnSegment++;
				latestPointIndex = i;
			}
		}
		
		//If no intersection points fall on the segment, return either the start or end point
		if (pointsOnSegment == 0) {
			return Optional.ofNullable(getClosestEndPoint(referencePosition));
		}
		
		//If only one point falls on the segment, return that point.
		else if (pointsOnSegment == 1) {
			return Optional.ofNullable(positions[latestPointIndex]);
		}
		
		//If two points fall on the segment, return the point depending on the round mode
		double currentClosest = Double.NaN;
		Position currentClosestPosition = null;
		for (int i = 0; i < positions.length; i++) {
			//If we round down, return the point behind the reference position
			if (roundMode == RoundMode.ROUND_DOWN) {
				//Get the intersection point relative to the actual closest point on the line segment
				Position relative = new Transform(positions[i]).relativeTo(new Transform(referencePosition, actualClosestPoint.angleTo(endPoint))).getPosition();
				//Check if relative point is in front of the origin, meaning it is behind
				if (relative.getX() <= 0) {
					currentClosest = positions[i].distance(referencePosition);
					currentClosestPosition = positions[i];
				}
			} else if (roundMode == RoundMode.ROUND_UP) {
				//Get the intersection point relative to the actual closest point on the line segment
				Position relative = new Transform(positions[i]).relativeTo(new Transform(referencePosition, actualClosestPoint.angleTo(endPoint))).getPosition();
				//Check if relative point is in front of the origin, meaning it is ahead of the reference point
				if (relative.getX() >= 0) {
					currentClosest = positions[i].distance(referencePosition);
					currentClosestPosition = positions[i];
				}
			} else {
				if (Double.isNaN(currentClosest) || Math.abs(positions[i].distance(referencePosition) - distanceShift) < currentClosest) {
					currentClosest = positions[i].distance(referencePosition);
					currentClosestPosition = positions[i];
				}
			}
		}
		
		return Optional.ofNullable(currentClosestPosition);
	}
	
	/**
	 * Gets the closest end point of the {@link LineSegment} to the <code>referencePosition</code>.
	 *
	 * @param referencePosition The point to get the closest end point to.
	 * @return the closest end point {@link Position} to the <code>referencePosition</code>.
	 */
	public Position getClosestEndPoint(Position referencePosition) {
		double distanceToStartPoint = referencePosition.distance(startPoint);
		double distanceToEndPoint = referencePosition.distance(endPoint);
		if (distanceToStartPoint < distanceToEndPoint) {
			return startPoint;
		} else {
			return endPoint;
		}
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
