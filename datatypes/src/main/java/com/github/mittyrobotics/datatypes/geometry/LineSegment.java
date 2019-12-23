/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.positioning.Position;
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
	public double getSegmentLength() {
		return getDistanceToPoint(endPoint);
	}
	
	/**
	 * Gets the distance along the segment from the start point to an end point.
	 *
	 * @param position the ending point
	 * @return the distance
	 */
	public double getDistanceToPoint(Position position) {
		return startPoint.distance(position);
	}
	
	/**
	 * Finds the closest {@link Transform} on this {@link LineSegment} to the <code>referencePosition</code> that is
	 * <code>distanceShift</code> away from the <code>referencePosition</code>. Returns it in the form of an
	 * {@link Optional} due to the fact that it may not exist.
	 * <p>
	 * This is done by finding the circle with a center of <code>referencePosition</code> that has a radius of
	 * <code>distanceShift</code> and finding the points that it intersects with this {@link Line}. It then gets the
	 * point that falls within the {@link LineSegment}.
	 * <p>
	 * If two intersection points fall within the {@link LineSegment}, it will pick the point either in front of,
	 * behind, or closest to the <code>referenceTransform</code> based on the {@link RoundMode}.
	 * <p>
	 * If no intersection points fall within the {@link LineSegment} or exist, it will return an empty {@link Optional}.
	 *
	 * @param referenceTransform the {@link Position} to find the closest point to.
	 * @param distanceShift      the distance away from the <code>referencePosition</code> to find the closest point to.
	 * @param roundMode          the {@link RoundMode}
	 * @return an {@link Optional} containing the closest {@link Position} to the <code>referencePosition</code> that is
	 * <code>distanceShift</code> away.
	 */
	public Optional<Transform> getClosestPointOnSegment(Transform referenceTransform, double distanceShift, RoundMode roundMode) {
		//Get the actual closest point on the line segment
		Position actualClosestPoint = getClosestPoint(referenceTransform.getPosition());
		
		//If the actual closest point is not on the segment, set it to the closest end point
		if (!isOnSegment(actualClosestPoint)) {
			actualClosestPoint = getClosestEndPoint(referenceTransform.getPosition());
		}
		
		Transform actualClosestTransform = new Transform(actualClosestPoint, actualClosestPoint.angleTo(endPoint));
		
		actualClosestTransform = new Transform(actualClosestTransform.getPosition(), actualClosestTransform.getRotation());
		
		//If the distance shift is zero, meaning we want to find the actual closest point to the reference position,
		//return the actual closest point on the segment.
		if (distanceShift == 0) {
			return Optional.of(actualClosestTransform);
		}
		
		//Get points that intersect the circle
		Position[] positions = new Circle(referenceTransform.getPosition(), distanceShift).circleLineIntersection(this);
		
		//If no points intersect the two circles, return the actual closest point on the segment
		if (positions.length == 0) {
			return Optional.of(actualClosestTransform);
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
		
		//If no intersection points fall on the segment, return empty
		if (pointsOnSegment == 0) {
			return Optional.empty();
		}
		
		//If only one point falls on the segment, return that point.
		else if (pointsOnSegment == 1) {
			Position pos = positions[latestPointIndex];
			
			return Optional.of(new Transform(pos, pos.angleTo(endPoint)));
		}
		
		//If two points fall on the segment, return the point depending on the round mode
		double currentClosest = Double.NaN;
		Position currentClosestPosition = null;
		for (int i = 0; i < positions.length; i++) {
			//If we round down, return the point behind the reference position
			if (roundMode == RoundMode.ROUND_DOWN) {
				//Get the intersection point relative to the actual closest point on the line segment
				Position relative = new Transform(positions[i]).relativeTo(referenceTransform).getPosition();
				//Check if relative point is in front of the origin, meaning it is behind
				if (relative.getX() <= 0) {
					currentClosest = positions[i].distance(referenceTransform.getPosition());
					currentClosestPosition = positions[i];
				}
			} else if (roundMode == RoundMode.ROUND_UP) {
				//Get the intersection point relative to the actual closest point on the line segment
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
		
		return Optional.of(new Transform(currentClosestPosition, currentClosestPosition.angleTo(endPoint)));
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
