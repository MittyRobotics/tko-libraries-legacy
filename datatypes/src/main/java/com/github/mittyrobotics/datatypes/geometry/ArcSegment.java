package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

import java.awt.*;
import java.util.Optional;

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
	 * Gets the length of the arc segment.
	 *
	 * @return the length of the arc segment.
	 */
	public double getSegmentLength() {
		return getDistanceToPoint(endPoint);
	}
	
	/**
	 * Gets the distance along the segment from the start point to an end point.
	 *
	 * @param position the ending point
	 * @return the distance
	 */
	public double getDistanceToPoint(Position position){
		return 2 * getRadius() * Math.asin(getStartPoint().distance(position) / (2 * getRadius()));
	}
	
	/**
	 * Finds the closest point on this {@link ArcSegment} to the <code>referencePosition</code> that is
	 * <code>distanceShift</code> away from the <code>referencePosition</code>.
	 * <p>
	 * This is done by finding the circle with a center of <code>referencePosition</code> that has a radius of
	 * <code>distanceShift</code> and finding the points that it intersects with this {@link Circle}. It then gets the
	 * point that falls within the {@link ArcSegment}.
	 * <p>
	 * If two intersection points fall within the {@link ArcSegment},
	 * it will pick the point closest to <code>distanceShift</code> away from the <code>referencePosition</code>.
	 * <p>
	 * If no intersection points fall within the {@link ArcSegment} or exist, it will return null.
	 *
	 * @param referenceTransform the {@link Position} to find the closest point to.
	 * @param distanceShift     the distance away from the <code>referencePosition</code> to find the closest point to.
	 * @param roundMode         the {@link RoundMode}
	 * @return the closest {@link Position} to the <code>referencePosition</code> that is <code>distanceShift</code> away.
	 */
	public Optional<Transform> getClosestPointOnSegment(Transform referenceTransform, double distanceShift, RoundMode roundMode) {
		//Get the actual closest point on the arc
		Position actualClosestPoint = getClosestPoint(referenceTransform.getPosition());
		
		//If the actual closest point is not on the segment, set it to the closest end point
		if (!isOnSegment(actualClosestPoint)) {
			actualClosestPoint = getClosestEndPoint(referenceTransform.getPosition());
		}
		
		Transform actualClosestTransform = new Transform(actualClosestPoint,getTangentLineAtPoint(actualClosestPoint).getLineAngle());
		
		actualClosestTransform = new Transform(actualClosestTransform.getPosition(),actualClosestTransform.getRotation());
		
		//If the distance shift is zero, meaning we want to find the actual closest point to the reference position,
		//return the actual closest point on the segment.
		if (distanceShift == 0) {
			return Optional.of(actualClosestTransform);
		}
		
		//Get points that intersect the circle
		Position[] positions = circleCircleIntersection(new Circle(referenceTransform.getPosition(), distanceShift));
		
		//If no points intersect the two circles, return the actual closest point on the segment
		if (positions.length == 0) {
			return Optional.of(actualClosestTransform);
		}
		
		double pointsOnSegment = 0;
		int latestPointIndex = 0;
		//Loop through all points and determine how many points intersect with the arc
		for (int i = 0; i < positions.length; i++) {
			if (isOnSegment(positions[i])) {
				pointsOnSegment++;
				latestPointIndex = i;
			}
		}
		
		//If no intersection points fall on the segment, return either the start or end point
		if (pointsOnSegment == 0) {
			Position pos = getClosestEndPoint(referenceTransform.getPosition());
			return Optional.of(new Transform(pos,getTangentLineAtPoint(pos).getLineAngle()));
		}
		
		//If only one point falls on the segment, return that point.
		else if (pointsOnSegment == 1) {
			Position pos = positions[latestPointIndex];
			return Optional.of(new Transform(pos,getTangentLineAtPoint(pos).getLineAngle()));
		}
		
		//If two points fall on the segment, return the point depending on the round mode
		double currentClosest = Double.NaN;
		Position currentClosestPosition = null;
		for (int i = 0; i < positions.length; i++) {
			//If we round down, return the point behind the reference position
			if (roundMode == RoundMode.ROUND_DOWN) {
				//Get the intersection point relative to the actual closest point on the arc
				Position relative = new Transform(positions[i]).relativeTo(referenceTransform).getPosition();
				//Check if relative point is in front of the origin, meaning it is behind
				if (relative.getX() <= 0) {
					currentClosest = positions[i].distance(referenceTransform.getPosition());
					currentClosestPosition = positions[i];
				}
			} else if (roundMode == RoundMode.ROUND_UP) {
				//Get the intersection point relative to the actual closest point on the arc
				Position relative = new Transform(positions[i]).relativeTo(referenceTransform).getPosition();
				//Check if relative point is in front of the origin, meaning it is ahead of the reference point
				if (relative.getX() >= 0) {
					currentClosest = positions[i].distance(referenceTransform.getPosition());
					currentClosestPosition = positions[i];
				}
			} else {
				if (Double.isNaN(currentClosest) || Math.abs(positions[i].distance(referenceTransform.getPosition()) - distanceShift) < currentClosest) {
					currentClosest = positions[i].distance(referenceTransform.getPosition());
					currentClosestPosition = positions[i];
				}
			}
		}
		
		return Optional.of(new Transform(currentClosestPosition,getTangentLineAtPoint(currentClosestPosition).getLineAngle()));
	}
	
	/**
	 * Gets the closest end point of the {@link ArcSegment} to the <code>referencePosition</code>.
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
	 * Determines whether or not a point is on this {@link ArcSegment} segment.
	 * <p>
	 * This checks for both if the point is on the circle and if the point is within the defining end points of the arc.
	 *
	 * @param point the {@link Position} to determine if it is on the arc or not
	 * @return whether or not <code>point</code> is on this {@link ArcSegment} segment.
	 */
	public boolean isOnSegment(Position point) {
		double intermediateToPoint = intermediatePoint.distance(point);
		double pointToStart = point.distance(startPoint);
		double pointToEnd = point.distance(endPoint);
		
		boolean isWithinPoints = false;
		
		if (pointToStart < pointToEnd) {
			double intermediateToStartDist = intermediatePoint.distance(startPoint);
			isWithinPoints = intermediateToPoint <= intermediateToStartDist;
		} else {
			double intermediateToEndDist = intermediatePoint.distance(endPoint);
			isWithinPoints = intermediateToPoint <= intermediateToEndDist;
		}
		
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
