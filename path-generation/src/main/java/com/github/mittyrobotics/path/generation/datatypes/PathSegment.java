package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.geometry.LineSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.TrapezoidalMotionProfile;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;

import java.util.Optional;

public abstract class PathSegment {
	private Position startPoint;
	private Position endPoint;
	private double segmentDistance;
	private double maxVelocity;
	private MotionState startMotionState;
	private MotionState endMotionState;
	private TrapezoidalMotionProfile velocityMotionProfile;
	private final PathSegmentType type;
	
	/**
	 * Constructs a {@Link PathSegment} given a {@link PathSegmentType}.
	 *
	 * @param type the {@link PathSegmentType} of the path segment.
	 */
	public PathSegment(PathSegmentType type, Position startPoint, Position endPoint) {
		this.type = type;
		this.startPoint = startPoint;
		this.endPoint = endPoint;
	}
	
	/**
	 * Interpolates the closest {@link Position} to the reference position on the path segment.
	 *
	 * @param referenceTransform the {@link Transform} to find the closest point to.
	 * @return the {@link Transform} closest to the <code>referencePosition</code>.
	 */
	public abstract Optional<Transform> getClosestPointOnSegment(Transform referenceTransform);
	
	/**
	 * Interpolates the closest {@link Position} that is <code>distanceShift</code> away from the
	 * <code>referencePosition</code> on the path segment.
	 * <p>
	 * It will round to either the closest point in front of the
	 * <code>referencePoint</code>, behind the <code>referencePoint</code>, or the closest to the
	 * <code>referencePoint</code> depending on the {@link RoundMode}.
	 *
	 * @param referenceTransform the {@link Transform} to find the closest point to.
	 * @param distanceShift     the distance away from the <code>referencePosition</code> to find the closest point to.
	 * @param roundMode         the {@link RoundMode}
	 * @return the {@link Transform} closest to the <code>referencePosition</code>.
	 */
	public abstract Optional<Transform> getClosestPointOnSegment(Transform referenceTransform, double distanceShift, RoundMode roundMode);
	
	/**
	 * Returns the distance of the {@link Position} along the segment.
	 *
	 * @param position the {@link Position} to get the distance to.
	 * @return the distance of the {@link Position} along the segment.
	 */
	public abstract double getDistanceAlongSegment(Position position);
	
	/**
	 * Returns the {@link PathSegmentType} of the path segment
	 *
	 * @return the {@link PathSegmentType} of the path segment
	 */
	public PathSegmentType getPathSegmentType() {
		return type;
	}
	
	/**
	 * Returns whether or not the {@link Position} is on the segment.
	 *
	 * @param position the {@link Position} to check
	 * @return whether or not the {@link Position} is on the segment.
	 */
	public abstract boolean isOnSegment(Position position);
	
	/**
	 * Returns the {@link TrapezoidalMotionProfile} that controls the velocity of the robot during the path segment.
	 *
	 * @return the {@link TrapezoidalMotionProfile} of the path segment.
	 */
	public TrapezoidalMotionProfile getVelocityMotionProfile() {
		return velocityMotionProfile;
	}
	
	/**
	 * Sets the {@link TrapezoidalMotionProfile} for the path segment. Set this to the generated profile for the segment.
	 *
	 * @param velocityMotionProfile the {@link TrapezoidalMotionProfile} for the path segment.
	 */
	public void setVelocityMotionProfile(TrapezoidalMotionProfile velocityMotionProfile) {
		this.velocityMotionProfile = velocityMotionProfile;
	}
	
	
	/**
	 * Returns the maximum allowed velocity for this segment.
	 *
	 * @return the maximum allowed velocity for this segment.
	 */
	public double getMaxVelocity() {
		return maxVelocity;
	}
	
	/**
	 * Set the maximum allowed velocity for this segment.
	 *
	 * @param maxVelocity the maximum allowed velocity for this segment.
	 */
	public void setMaxVelocity(double maxVelocity) {
		this.maxVelocity = maxVelocity;
	}
	
	public ArcSegment getArcSegment() {
		return null;
	}
	
	public LineSegment getLineSegment() {
		return null;
	}
	
	public Position getStartPoint() {
		return startPoint;
	}
	
	public Position getEndPoint() {
		return endPoint;
	}
	
	public MotionState getStartMotionState() {
		return startMotionState;
	}
	
	public MotionState getEndMotionState() {
		return endMotionState;
	}
	
	public void setStartMotionState(MotionState startMotionState) {
		this.startMotionState = startMotionState;
	}
	
	public void setEndMotionState(MotionState endMotionState) {
		this.endMotionState = endMotionState;
	}
	
	public double getSegmentDistance() {
		return segmentDistance;
	}
	
	public void setSegmentDistance(double segmentDistance) {
		this.segmentDistance = segmentDistance;
	}
	
	
}
