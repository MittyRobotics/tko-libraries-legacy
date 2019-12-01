package com.amhsrobotics.libs.util.path;

import com.amhsrobotics.libs.auton.motionprofile.TrapezoidalMotionProfile;
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
	
	public abstract Transform getStartPoint();
	
	public abstract Transform getEndPoint();
	
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
}
