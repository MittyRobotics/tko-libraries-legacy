package com.amhsrobotics.libs.util.path;

import com.amhsrobotics.libs.util.geometry.Line;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;

public abstract class PathSegment {
	/**
	 * Returns the intersection of the line perpendicular to the segment that intersects with the robot position
	 *
	 * @param robotTransform the {@link Transform} of the robot
	 * @return the point intersecting with the perpendicular robot line and the segment
	 */
	public abstract Position getIntersection(Transform robotTransform);
	
	/**
	 * Checks to see whether a point lies on the path segment and between the beginning and ending point.
	 *
	 * @param point the point to check if it is on the segment
	 * @return whether the point lies on the path segment and is between the beginning and ending point
	 */
	public abstract boolean onSegment(Position point);
	
	/**
	 * Returns the velocity of the path segment. This is how fast the robot should be going at this segment
	 */
	public abstract double getVelocity();
	
	public abstract Transform getStartPoint();
	
	public abstract Transform getEndPoint();
	
	public abstract double getSegmentDistance();
	
	public abstract double getAbsoluteStartDistance();
	
	public abstract double getAbsoluteEndDistance();
	
	public abstract double getSegmentTime();
	
	public abstract double getStartTime();
	
	public abstract double getEndTime();
	
	public abstract double getRemainingDistance(Position intersectionPoint);
	
	public abstract PathSegmentType getType();
	
	public abstract ArcPathSegment getArcSegment();
	
	public abstract LinePathSegment getLineSegment();
}
