package com.amhsrobotics.libs.auton.path.generation;

import com.amhsrobotics.libs.auton.motionprofile.JointVelocityMotionProfile;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.math.geometry.Arc;
import com.amhsrobotics.libs.math.geometry.Transform;

import java.util.ArrayList;

public abstract class Path {
	private final Transform[] waypoints;
	private final VelocityConstraints velocityConstraints;
	private TrajectoryPoint[] trajectoryPoints;
	private JointVelocityMotionProfile velocityProfile;
	private double radiusSlowdownThreshold;
	private double flatSlowdownVelocity;
	private final int samples;
	
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
		calculateVelocityProfile();
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
		trajectoryPoints[0].setRadius(0);
		trajectoryPoints[trajectoryPoints.length - 1].setRadius(0);
		for (int i = 1; i < trajectoryPoints.length - 1; i++) {
			Arc arc = trajectoryPoints[i].findIntersectingArc(trajectoryPoints[i - 1], trajectoryPoints[i + 1]);
			
			double r = arc.getRadius();
			
			r = r * arc.getCenter().findSide(trajectoryPoints[i - 1].getPosition(), trajectoryPoints[i].getPosition());
			
			//avoid infinite errors
			if (Double.isInfinite(r)) {
				r = 99999;
			}
			
			//avoid divide by 0 errors
			if (r == 0) {
				r = 0.00001;
			}
			
			
			trajectoryPoints[i].setRadius(r);
		}
	}
	
	/**
	 * Calculates the velocity and time of each point along the path
	 */
	public void calculateVelocity() {
		for (int i = trajectoryPoints.length - 1; i > -1; i--) {
			if (i == trajectoryPoints.length - 1) {
				trajectoryPoints[i].setVelocity(velocityConstraints.getEndVelocity());
			} else {
				double distance = trajectoryPoints[i + 1].getPosition().distance(trajectoryPoints[i].getPosition());
				double velocity;
				if (velocityConstraints.getMaxDeceleration() != 0) {
					velocity = Math.sqrt(Math.pow(trajectoryPoints[i + 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxDeceleration() * distance);
				} else {
					velocity = Math.sqrt(Math.pow(trajectoryPoints[i + 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxAcceleration() * distance);
				}
				
				if (trajectoryPoints[i].getRadius() < radiusSlowdownThreshold) {
					velocity = Math.min(flatSlowdownVelocity, velocity);
				}
				
				trajectoryPoints[i].setVelocity(velocity);
			}
		}
		
		for (int i = 0; i < trajectoryPoints.length; i++) {
			if (i == 0) {
				trajectoryPoints[i].setTime(0);
			}
			if (i <= 0) {
				trajectoryPoints[i].setVelocity(velocityConstraints.getStartVelocity());
			} else {
				double distance = trajectoryPoints[i - 1].getPosition().distance(trajectoryPoints[i].getPosition());
				double velocity = Math.min(trajectoryPoints[i].getVelocity(), Math.sqrt(Math.pow(trajectoryPoints[i - 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxAcceleration() * distance));
				double time = distance / velocity;
				
				if (Double.isInfinite(time)) {
					time = 0;
				}
				
				if (trajectoryPoints[i].getRadius() < radiusSlowdownThreshold) {
					velocity = Math.min(flatSlowdownVelocity, velocity);
				}
				
				trajectoryPoints[i].setVelocity(velocity);
				trajectoryPoints[i].setTime(trajectoryPoints[i - 1].getTime() + time);
			}
		}
	}
	
	/**
	 * Calculates the angles from each point to the next.
	 */
	public void calculateAngles() {
		for (int i = 0; i < trajectoryPoints.length - 1; i++) {
			trajectoryPoints[i].getRotation().setHeading(trajectoryPoints[i].getPosition().angleTo(trajectoryPoints[i + 1].getPosition()));
		}
	}
	
	/**
	 * Calculates the {@link JointVelocityMotionProfile} profile for the robot's linear following velocity.
	 */
	public void calculateVelocityProfile() {
		ArrayList<JointVelocityMotionProfile.MotionSegment> motionSegments = new ArrayList<>();
		boolean recordingSlowdown = false;
		double currentSlowdownStartTime;
		double currentMotionProfileSegmentStartTime;
		
		for (int i = 0; i < trajectoryPoints.length; i++) {
			if (trajectoryPoints[i].getRadius() < radiusSlowdownThreshold && recordingSlowdown == false) {
				recordingSlowdown = true;
				if (motionSegments.size() == 0) {
					motionSegments.add(new JointVelocityMotionProfile.TrapezoidalMotionSegment(0, trajectoryPoints[i].getTime(),
							new VelocityConstraints(velocityConstraints.getMaxAcceleration(), velocityConstraints.getMaxDeceleration(), velocityConstraints.getMaxVelocity(), velocityConstraints.getStartVelocity(), flatSlowdownVelocity)));
				} else {
					motionSegments.add(new JointVelocityMotionProfile.TrapezoidalMotionSegment(motionSegments.get(motionSegments.size() - 1).getEndTime(), trajectoryPoints[i].getTime(),
							new VelocityConstraints(velocityConstraints.getMaxAcceleration(), velocityConstraints.getMaxDeceleration(), velocityConstraints.getMaxVelocity(), flatSlowdownVelocity, flatSlowdownVelocity)));
				}
			}
			if (trajectoryPoints[i].getRadius() > radiusSlowdownThreshold && recordingSlowdown == true) {
				recordingSlowdown = false;
				if (motionSegments.size() == 0) {
					motionSegments.add(new JointVelocityMotionProfile.FlatMotionSegment(flatSlowdownVelocity, 0, trajectoryPoints[i].getTime()));
				} else {
					motionSegments.add(new JointVelocityMotionProfile.FlatMotionSegment(flatSlowdownVelocity, motionSegments.get(motionSegments.size() - 1).getEndTime(), trajectoryPoints[i].getTime()));
				}
			}
		}
		
		JointVelocityMotionProfile.MotionSegment[] motionSegmentsArray = new JointVelocityMotionProfile.MotionSegment[motionSegments.size()];
		
		for (int i = 0; i < motionSegments.size(); i++) {
			motionSegmentsArray[i] = motionSegments.get(i);
		}
		velocityProfile = new JointVelocityMotionProfile(motionSegmentsArray, velocityConstraints);
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
	
	public JointVelocityMotionProfile getVelocityProfile() {
		return velocityProfile;
	}
	
	public double getRadiusSlowdownThreshold() {
		return radiusSlowdownThreshold;
	}
	
	public double getFlatSlowdownVelocity() {
		return flatSlowdownVelocity;
	}
	
}
