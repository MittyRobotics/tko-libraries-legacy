package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;

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
	public double getArcLength() {
		return 0;
	}
	
	/**
	 * Finds the closest point on this {@link ArcSegment} to the <code>referencePosition</code> that is
	 * <code>distanceShift</code> away from the <code>referencePosition</code>.
	 * <p>
	 * This is done by finding the circle with a center of <code>referencePosition</code> that has a radius of
	 * <code>distanceShift</code> and finding the points that it intersects with this {@link Circle}. It then gets the
	 * point that falls within the {@link ArcSegment}.
	 *
	 * If two intersection points fall within the {@link ArcSegment},
	 * it will pick the point closest to <code>distanceShift</code> away from the <code>referencePosition</code>.
	 *
	 * If no intersection points fall within the {@link ArcSegment} or exist, it will return null.
	 *
	 * @param referencePosition the {@link Position} to find the closest point to.
	 * @param distanceShift     the distance away from the <code>referencePosition</code> to find the closest point to.
	 * @param roundMode
	 * @return the closest {@link Position} to the <code>referencePosition</code> that is <code>distanceShift</code> away.
	 */
	public Position getClosestPointOnSegment(Position referencePosition, double distanceShift, RoundMode roundMode) {
		//Get points that intersect the circle
		Position[] positions = circleCircleIntersection(new Circle(referencePosition,distanceShift));
		
		//If no points intersect the two circles, return either the start or end point of the segment.
		if(positions.length == 0){
			double distanceToStartPoint = referencePosition.distance(startPoint);
			double distanceToEndPoint = referencePosition.distance(endPoint);
			if(distanceToStartPoint < distanceToEndPoint){
				return startPoint;
			}
			else{
				return endPoint;
			}
		}
		
		double pointsOnSegment = 0;
		int latestPointIndex = 0;
		//Loop through all points and determine how many points intersect with the arc
		for(int i = 0; i  < positions.length; i++){
			if(isOnSegment(positions[i])){
				pointsOnSegment ++;
				latestPointIndex = i;
			}
		}
		
		//If no points fall on the segment, return null.
		if(pointsOnSegment == 0){
			double distanceToStartPoint = referencePosition.distance(startPoint);
			double distanceToEndPoint = referencePosition.distance(endPoint);
			if(distanceToStartPoint < distanceToEndPoint){
				return startPoint;
			}
			else{
				return endPoint;
			}
		}
		//If only one point falls on the segment, return that point.
		else if(pointsOnSegment == 1){
			return positions[latestPointIndex];
		}
		
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
