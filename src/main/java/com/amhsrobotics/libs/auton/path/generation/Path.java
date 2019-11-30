package com.amhsrobotics.libs.auton.path.generation;

import com.amhsrobotics.libs.auton.motionprofile.JointVelocityMotionProfile;
import com.amhsrobotics.libs.auton.motionprofile.TrapezoidalMotionProfile;
import com.amhsrobotics.libs.util.path.PathSegment;
import com.amhsrobotics.libs.util.path.TrajectoryPoint;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.util.geometry.Arc;
import com.amhsrobotics.libs.util.geometry.Transform;
import sun.jvm.hotspot.debugger.sparc.SPARCThreadContext;

import java.util.ArrayList;

public abstract class Path {
	private final Transform[] waypoints;
	private final VelocityConstraints velocityConstraints;
	private TrapezoidalMotionProfile baseVelocityProfile;
	private double radiusSlowdownThreshold;
	private double flatSlowdownVelocity;
	private final int samples;
	
	private TrajectoryPoint[] trajectoryPoints;
	private ArrayList<PathSegment> pathSegments = new ArrayList<>();
	

	
	public Path(Transform[] waypoints, VelocityConstraints velocityConstraints, double radiusSlowdownThreshold, int samples) {
		this.samples = samples;
		this.waypoints = waypoints;
		this.velocityConstraints = velocityConstraints;
		this.radiusSlowdownThreshold = radiusSlowdownThreshold;
		this.flatSlowdownVelocity = velocityConstraints.getMinVelocity();
		
		generateTrajectoryPoints();
		calculateDistances();
		calculateCurvature();
		calculateVelocity();
		calculateAngles();
	}
	
	public abstract void generateTrajectoryPoints();
	
	/**
	 * Calculates the position of each point along the path.
	 */
	public void calculateDistances() {
		if (trajectoryPoints != null) {
			trajectoryPoints[0].setDistanceAlongPath(0);
			for (int i = 1; i < trajectoryPoints.length; i++) {
				trajectoryPoints[i].setDistanceAlongPath(trajectoryPoints[i - 1].getPosition().distance(trajectoryPoints[i].getPosition()) + trajectoryPoints[i - 1].getDistanceAlongPath());
			}
		}
	}
	
	
	/**
	 * Calculates the curvature at each point on the path
	 */
	public void calculateCurvature() {
//		trajectoryPoints[0].setRadius(0);
//		trajectoryPoints[trajectoryPoints.length - 1].setRadius(0);
//		for (int i = 1; i < trajectoryPoints.length - 1; i++) {
//			Arc arc = trajectoryPoints[i].findIntersectingArc(trajectoryPoints[i - 1], trajectoryPoints[i + 1]);
//
//			double r = arc.getRadius();
//
//			r = r * arc.getCenter().findSide(trajectoryPoints[i - 1].getPosition(), trajectoryPoints[i].getPosition());
//
//			//avoid infinite errors
//			if (Double.isInfinite(r)) {
//				r = 99999;
//			}
//
//			//avoid divide by 0 errors
//			if (r == 0) {
//				r = 0.00001;
//			}
//
//
//			trajectoryPoints[i].setRadius(r);
//		}
	}
	
	/**
	 * Calculates the {@link TrapezoidalMotionProfile} profile for the robot's linear following velocity.
	 */
	public void calculateBaseVelocityProfile() {
		double tTotal = trajectoryPoints[trajectoryPoints.length-1].getTime();
		baseVelocityProfile = new TrapezoidalMotionProfile(tTotal,velocityConstraints);
	}
	
	
	/**
	 * Calculates the velocity and time of each point along the path
	 */
	public void calculateVelocity() {
		calculateBaseVelocityProfile();
//		for (int i = trajectoryPoints.length - 1; i > -1; i--) {
//			if (i == trajectoryPoints.length - 1) {
//				trajectoryPoints[i].setVelocity(velocityConstraints.getEndVelocity());
//			} else {
//				double distance = trajectoryPoints[i + 1].getPosition().distance(trajectoryPoints[i].getPosition());
//				double velocity;
//				if (velocityConstraints.getMaxDeceleration() != 0) {
//					velocity = Math.sqrt(Math.pow(trajectoryPoints[i + 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxDeceleration() * distance);
//				} else {
//					velocity = Math.sqrt(Math.pow(trajectoryPoints[i + 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxAcceleration() * distance);
//				}
//
//				if (trajectoryPoints[i].getRadius() < radiusSlowdownThreshold) {
//					velocity = Math.min(flatSlowdownVelocity, velocity);
//				}
//
//				trajectoryPoints[i].setVelocity(velocity);
//			}
//		}
//
//		for (int i = 0; i < trajectoryPoints.length; i++) {
//			if (i == 0) {
//				trajectoryPoints[i].setTime(0);
//			}
//			if (i <= 0) {
//				trajectoryPoints[i].setVelocity(velocityConstraints.getStartVelocity());
//			} else {
//				double distance = trajectoryPoints[i - 1].getPosition().distance(trajectoryPoints[i].getPosition());
//				double velocity = Math.min(trajectoryPoints[i].getVelocity(), Math.sqrt(Math.pow(trajectoryPoints[i - 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxAcceleration() * distance));
//				double time = distance / velocity;
//
//				if (Double.isInfinite(time)) {
//					time = 0;
//				}
//
//				if (trajectoryPoints[i].getRadius() < radiusSlowdownThreshold) {
//					velocity = Math.min(flatSlowdownVelocity, velocity);
//				}
//
//				trajectoryPoints[i].setVelocity(velocity);
//				trajectoryPoints[i].setTime(trajectoryPoints[i - 1].getTime() + time);
//			}
//		}
	}
	
	/**
	 * Calculates the angles from each point to the next.
	 */
	public void calculateAngles() {
		for (int i = 0; i < trajectoryPoints.length - 1; i++) {
			trajectoryPoints[i].getRotation().setHeading(trajectoryPoints[i].getPosition().angleTo(trajectoryPoints[i + 1].getPosition()));
		}
	}
	

	public Transform[] getWaypoints() {
		return waypoints;
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
	
	
	public void setTrajectoryPoints(TrajectoryPoint[] trajectoryPoints) {
		this.trajectoryPoints = trajectoryPoints;
	}
	
	public int getSamples() {
		return samples;
	}
	
	public TrajectoryPoint[] getTrajectoryPoints() {
		return trajectoryPoints;
	}
	
	public double getRadiusSlowdownThreshold() {
		return radiusSlowdownThreshold;
	}
	
	public double getFlatSlowdownVelocity() {
		return flatSlowdownVelocity;
	}
	
	public TrapezoidalMotionProfile getBaseVelocityProfile() {
		return baseVelocityProfile;
	}
	
	public void setBaseVelocityProfile(TrapezoidalMotionProfile baseVelocityProfile) {
		this.baseVelocityProfile = baseVelocityProfile;
	}
}
