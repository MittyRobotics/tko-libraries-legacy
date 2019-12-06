package com.amhsrobotics.libs.util.path;

import com.amhsrobotics.libs.auton.motionprofile.TrapezoidalMotionProfile;
import com.amhsrobotics.libs.util.geometry.Arc;
import com.amhsrobotics.libs.util.geometry.Line;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;

public abstract class PathSegment {
	
	private double startTime;
	private double endTime;
	private double maxVelocity;
	private double startVelocity;
	private double endVelocity;
	private TrapezoidalMotionProfile motionProfile;
	
	private Transform startPoint;
	private Transform endPoint;
	
	
	/**
	 * Returns the intersection of the line perpendicular to the segment that intersects with the referenceTransform
	 *
	 * @param referenceTransform the {@link Transform} of the point you want to find the closest point to
	 * @return the point intersecting with the perpendicular robot line and the segment
	 */
	public abstract Position getParallelIntersection(Transform referenceTransform);
	
	/**
	 * Checks to see whether a point lies on the path segment and between the beginning and ending point.
	 *
	 * @param point the point to check if it is on the segment
	 * @return whether the point lies on the path segment and is between the beginning and ending point
	 */
	public abstract boolean onSegment(Position point);
	
	/**
	 * Returns the points of intersection of this segment and a specified circle. If there are multiple, it returns the closest one to the starting point of the segment.
	 * @param circle
	 * @return the point of intersection
	 */
	public abstract Position getIntersectionPointWithCircle(Arc circle);
	
	/**
	 * Returns the max velocity of the path segment. This is the absolute max speed the robot can be driving at this
	 * segment, and is used in calculating the motion profile of all segments.
	 */
	public double getMaxVelocity(){
		return maxVelocity;
	}
	
	public void setMaxVelocity(double maxVelocity){
		this.maxVelocity = maxVelocity;
	}
	
	public TrapezoidalMotionProfile getMotionProfile(){
		return motionProfile;
	}
	
	public void setMotionProfile(TrapezoidalMotionProfile motionProfile){
		this.motionProfile = motionProfile;
	}
	
	public void setStartPoint(Transform startPoint) {
		this.startPoint = startPoint;
	}
	
	public void setEndPoint(Transform endPoint) {
		this.endPoint = endPoint;
	}
	
	public abstract double getSegmentDistance();
	
	public abstract double getAbsoluteStartDistance();
	
	public abstract double getAbsoluteEndDistance();
	
	public double getSegmentTime(){
		return endTime-startTime;
	}
	
	public double getStartTime(){
		return startTime;
	}
	
	public double getEndTime(){
		return endTime;
	}
	
	public void setStartTime(double startTime) {
		this.startTime = startTime;
	}
	
	public void setEndTime(double endTime) {
		this.endTime = endTime;
	}
	
	
	public abstract double getRemainingDistance(Position intersectionPoint);
	
	public abstract PathSegmentType getType();
	
	public abstract ArcPathSegment getArcSegment();
	
	public abstract LinePathSegment getLineSegment();
	
	
	public double getStartVelocity() {
		return startVelocity;
	}
	
	public void setStartVelocity(double startVelocity) {
		this.startVelocity = startVelocity;
	}
	
	public double getEndVelocity() {
		return endVelocity;
	}
	
	public void setEndVelocity(double endVelocity) {
		this.endVelocity = endVelocity;
	}
	
	public Transform getEndPoint() {
		return endPoint;
	}
	
	public Transform getStartPoint() {
		return startPoint;
	}
}
