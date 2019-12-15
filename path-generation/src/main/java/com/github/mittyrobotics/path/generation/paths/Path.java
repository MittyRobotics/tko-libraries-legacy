package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datacollection.performance.TimeMonitor;
import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.geometry.LineSegment;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.TrapezoidalMotionProfile;
import com.github.mittyrobotics.path.generation.datatypes.ArcPathSegment;
import com.github.mittyrobotics.path.generation.datatypes.LinePathSegment;
import com.github.mittyrobotics.path.generation.datatypes.PathSegment;
import com.github.mittyrobotics.path.generation.datatypes.TransformWithSegment;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;

import java.util.ArrayList;
import java.util.Optional;

public abstract class Path {
	private final Transform[] waypoints;
	private final MotionState startMotionState;
	private final MotionState endMotionState;
	private final VelocityConstraints velocityConstraints;
	private final double samples;
	
	private ArrayList<PathSegment> segments = new ArrayList<>();
	
	/**
	 * Constructs a new {@link Path}.
	 *
	 * @param waypoints           the waypoint {@link Transform}s for the {@link Path} to pass through
	 * @param startMotionState    the starting {@link MotionState} of the {@link Path}.
	 * @param endMotionState      the ending {@link MotionState} of the {@link Path}.
	 * @param velocityConstraints the {@link VelocityConstraints} of the {@link Path}.
	 * @param samples             How many path segments to generate for the {@link Path}. In other words, the amount of samples
	 *                            the path generation algorithm uses to generate points.
	 */
	public Path(Transform[] waypoints, MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints, double samples) {
		this.waypoints = waypoints;
		this.startMotionState = startMotionState;
		this.endMotionState = endMotionState;
		this.velocityConstraints = velocityConstraints;
		this.samples = samples;
		generatePathSegments();
		initMaxVelocities();
		initMotionStates();
		generateAdjustedMotionStates();
		generateMotionProfiles();
	}
	
	/**
	 * Constructs a new {@link Path}. This also takes in a <code>curvatureGain</code> and a
	 * <code>minSlowedVelocity</code>, which are used in slowing down {@link ArcPathSegment}s based on how sharp their
	 * curvature is.
	 *
	 * @param waypoints           the waypoint {@link Transform}s for the {@link Path} to pass through
	 * @param startMotionState    the starting {@link MotionState} of the {@link Path}.
	 * @param endMotionState      the ending {@link MotionState} of the {@link Path}.
	 * @param velocityConstraints the {@link VelocityConstraints} of the {@link Path}.
	 * @param samples             How many path segments to generate for the {@link Path}. In other words, the amount of samples
	 *                            the path generation algorithm uses to generate points.
	 * @param curvatureGain       a gain to slow down {@link ArcPathSegment}s based on their curvature.
	 * @param minSlowedVelocity   the minimum velocity that each {@link ArcPathSegment} is allowed to be slowed down.
	 */
	public Path(Transform[] waypoints, MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints, double samples, double curvatureGain, double minSlowedVelocity) {
		this.waypoints = waypoints;
		this.startMotionState = startMotionState;
		this.endMotionState = endMotionState;
		this.velocityConstraints = velocityConstraints;
		this.samples = samples;
		generatePathSegments();
		initMaxVelocities();
		slowdownCurvedSegments(curvatureGain, minSlowedVelocity);
		initMotionStates();
		generateAdjustedMotionStates();
		generateMotionProfiles();
	}
	
	/**
	 * Finds the closest {@link Transform} and {@link PathSegment} that is <code>distanceSift</code> away from the
	 * <code>referenceTransform</code>. It returns both the {@link Transform} and {@link PathSegment} in a
	 * {@link TransformWithSegment}.
	 * <p>
	 * This will find the closest {@link Transform} and {@link PathSegment} either in front of or behind the
	 * <code>referenceTransform</code> depending on the <code>reversed</code> boolean. If <code>reversed</code> is
	 * true, it will fond the closest behind the <code>referenceTransform</code>.
	 * <p>
	 * The <code>distanceShift</code> determines how far away the desired {@link Transform} should be from the
	 * <code>referenceTransform</code>.
	 *
	 * @param referenceTransform the {@link Transform} to find the closest point to
	 * @param distanceShift      the distance away from the <code>referenceTransform</code> to find the closest
	 *                           {@link Transform} and {@link PathSegment} to.
	 * @param reversed           whether to find the closest {@link Transform} behind or in front of the robot.
	 * @return a {@link TransformWithSegment} containing the closest {@link Transform} and the {@link PathSegment} that
	 * contains it
	 */
	public TransformWithSegment getClosestTransformWithSegment(Transform referenceTransform, double distanceShift, boolean reversed) {
		TransformWithSegment actualClosestSegment = null;
		double closestDist = Double.NaN;
		for (int i = 0; i < segments.size(); i++) {
			PathSegment segment = segments.get(i);
			Optional<Transform> transform = segment.getClosestPointOnSegment(referenceTransform);
			double distance = transform.get().getPosition().distance(referenceTransform.getPosition());
			if (transform.isPresent() && (Double.isNaN(closestDist) || distance < closestDist)) {
				actualClosestSegment = new TransformWithSegment(transform.get(), segments.get(i));
				closestDist = distance;
			}
		}
		
		if (actualClosestSegment == null) {
			double distanceToStart = referenceTransform.getPosition().distance(segments.get(0).getStartPoint());
			double distanceToEnd = referenceTransform.getPosition().distance(segments.get(segments.size() - 1).getEndPoint());
			if (distanceToStart < distanceToEnd) {
				actualClosestSegment = new TransformWithSegment(waypoints[0], segments.get(0));
			} else {
				actualClosestSegment = new TransformWithSegment(waypoints[waypoints.length - 1], segments.get(segments.size() - 1));
			}
		}
		
		//If the distance to the actual closest point is greater than the distance shift, return the actual closest point
		if (distanceShift == 0) {
			return actualClosestSegment;
		}
		
		TransformWithSegment shiftedClosestSegment = null;
		closestDist = Double.NaN;
		for (int i = 0; i < segments.size(); i++) {
			PathSegment segment = segments.get(i);
			Optional<Transform> transform = segment.getClosestPointOnSegment(referenceTransform);
			double distance = Math.abs(transform.get().getPosition().distance(referenceTransform.getPosition()) - distanceShift);
			Transform relative = transform.get().relativeTo(actualClosestSegment.getTransform());
			boolean correctSide;
			if (reversed) {
				correctSide = relative.getPosition().getX() < 0;
				transform.get().setRotation(transform.get().getRotation().rotateBy(new Rotation(180)));
			} else {
				correctSide = relative.getPosition().getX() > 0;
			}
			if (transform.isPresent() && (Double.isNaN(closestDist) || distance < closestDist) && correctSide) {
				shiftedClosestSegment = new TransformWithSegment(transform.get(), segments.get(i));
				closestDist = distance;
				
			}
		}
		
		if (shiftedClosestSegment == null) {
			double distance = distanceShift + 5;
			LineSegment lineSegment;
			if (reversed) {
				Rotation rot = waypoints[0].getRotation().add(new Rotation(180));
				lineSegment = new LineSegment(segments.get(0).getStartPoint(), segments.get(0).getStartPoint().add(new Position(rot.cos() * distance, rot.sin() * distance)));
			} else {
				Rotation rot = waypoints[waypoints.length - 1].getRotation();
				lineSegment = new LineSegment(segments.get(segments.size() - 1).getEndPoint(), segments.get(segments.size() - 1).getEndPoint().add(new Position(rot.cos() * distance, rot.sin() * distance)));
			}
			RoundMode roundMode;
			if (reversed) {
				roundMode = RoundMode.ROUND_DOWN;
			} else {
				roundMode = RoundMode.ROUND_UP;
			}
			Transform transform = lineSegment.getClosestPointOnSegment(referenceTransform, distanceShift, roundMode).get();
			return new TransformWithSegment(transform, new LinePathSegment(lineSegment));
		}
		
		return shiftedClosestSegment;
	}
	
	public abstract void generatePathSegments();
	
	/**
	 * Loops through all {@link PathSegment}s and assigns them the max velocity from the {@link VelocityConstraints}
	 * for this path. This maximum velocity may be overridden with a smaller maximum velocity for specific points such
	 * as sharp arcs.
	 */
	public void initMaxVelocities() {
		for (int i = 0; i < segments.size(); i++) {
			PathSegment segment = segments.get(i);
			segment.setMaxVelocity(getVelocityConstraints().getMaxVelocity());
		}
	}
	
	/**
	 * Calculates the new slowed down velocity for {@link ArcPathSegment}s based on their radius.
	 * <p>
	 * Uses <code>curvatureGain</code> to get the new maximum velocity for the {@link ArcPathSegment}s by dividing it
	 * by the curvature <code>(1/radius)</code> of each {@link ArcPathSegment}.
	 *
	 * @param curvatureGain the gain used to find the maximum velocity for each {@link ArcPathSegment}.
	 * @param minVelocity   the minimum velocity that the points are allowed to be slowed down. This ensures the segment
	 *                      does not get slowed down too much.
	 */
	public void slowdownCurvedSegments(double curvatureGain, double minVelocity) {
		//Loop through all segments
		for (int i = 0; i < segments.size(); i++) {
			//Check if path segment type is an arc
			if (segments.get(i).getPathSegmentType() == PathSegmentType.ARC) {
				//Get the arc segment of the segment
				ArcSegment arcSegment = segments.get(i).getArcSegment();
				PathSegment pathSegment = segments.get(i);
				
				//Calculate the slowed velocity by curvatureGain/curvature (1/radius)
				double slowedVelocity = curvatureGain / (1 / arcSegment.getRadius());
				
				//Cap the slowed velocity between minVelocity and the max velocity from the velocity constraints
				double maxVelocity = Math.min(velocityConstraints.getMaxVelocity(), Math.max(minVelocity, slowedVelocity));
				pathSegment.setMaxVelocity(maxVelocity);
			}
		}
	}
	
	/**
	 * Initializes the{@link MotionState}s for each {@link PathSegment} with a position. The velocity and time for the
	 * {@link MotionState} can then be calculated and set later.
	 */
	public void initMotionStates() {
		//Find the position of each segment
		for (int i = 0; i < segments.size(); i++) {
			
			//If this is the first segment
			if (i == 0) {
				//Set the start motion state position to 0
				segments.get(i).setStartMotionState(new MotionState(0, 0));
				//Set the end motion state position to the distance of the segment (total distance traveled by the end of the segment)
				segments.get(i).setEndMotionState(new MotionState(segments.get(i).getSegmentDistance(), 0));
				;
			} else {
				//Set the start motion state position to the position of the previous ending motion state position
				segments.get(i).setStartMotionState(new MotionState(segments.get(i - 1).getEndMotionState().getPosition(), 0));
				//Set the end motion state position to the distance of the segment (total distance traveled by the end of the segment)
				segments.get(i).setEndMotionState(new MotionState(segments.get(i - 1).getSegmentDistance() + segments.get(i).getStartMotionState().getPosition(), 0));
				
			}
		}
	}
	
	/**
	 * Generates adjusted start and end {@link MotionState}s for each {@link PathSegment} based on the maximum
	 * acceleration and deceleration from the {@link VelocityConstraints}.
	 * <p>
	 * These adjusted {@link MotionState}s will then work with a {@link TrapezoidalMotionProfile} to generate the
	 * velocity motion profile for each path segment.
	 */
	public void generateAdjustedMotionStates() {
		PathSegment previousSegment = null;
		//D0 a forward pass through the segments to regulate acceleration
		for (int i = 0; i < segments.size(); i++) {
			double startVelocity;
			double endVelocity;
			//If this is the first segment
			if (previousSegment == null) {
				//Set start velocity to the path's starting velocity
				startVelocity = getStartMotionState().getVelocity();
			} else {
				//Set start velocity to the end velocity of the previous segment
				startVelocity = previousSegment.getEndMotionState().getVelocity();
			}
			
			//Set the end velocity to the final velocity after accelerating for the distance of the segment
			//using the formula vFinal = sqrt(vInitial^2 + 2*acceleration*distance)
			endVelocity = Math.sqrt(startVelocity * startVelocity +
					2 * velocityConstraints.getMaxAcceleration() * segments.get(i).getSegmentDistance());
			
			//Cap the velocity to be below the max velocity of the segment
			double cappedStartVelocity = Math.min(segments.get(i).getMaxVelocity(), startVelocity);
			double cappedEndVelocity = Math.min(segments.get(i).getMaxVelocity(), endVelocity);
			
			segments.get(i).getStartMotionState().setVelocity(cappedStartVelocity);
			segments.get(i).getEndMotionState().setVelocity(cappedEndVelocity);
			previousSegment = segments.get(i);
		}
		previousSegment = null;
		//Do a backward pass through the segments to regulate deceleration
		for (int i = segments.size() - 1; i > 0; i--) {
			double startVelocity;
			double endVelocity;
			//If this is the first segment
			if (previousSegment == null) {
				//Set end velocity to the path's starting velocity
				endVelocity = getEndMotionState().getVelocity();
			} else {
				//Set end velocity to the start velocity of the previous segment
				endVelocity = previousSegment.getStartMotionState().getVelocity();
			}
			
			//Set the start velocity to the final velocity after accelerating for the distance of the segment
			//using the formula vFinal = sqrt(vInitial^2 + 2*acceleration*distance)
			startVelocity = Math.sqrt(endVelocity * endVelocity + 2 * velocityConstraints.getMaxDeceleration() * segments.get(i).getSegmentDistance());
			
			//Cap the velocity to be below the max velocity of the segment and get the lowest value between this
			//velocity and the current velocity as generated from the forward pass.
			double cappedStartVelocity = Math.min(Math.min(segments.get(i).getMaxVelocity(), startVelocity), segments.get(i).getStartMotionState().getVelocity());
			double cappedEndVelocity = Math.min(Math.min(segments.get(i).getMaxVelocity(), endVelocity), segments.get(i).getEndMotionState().getVelocity());
			
			segments.get(i).getStartMotionState().setVelocity(cappedStartVelocity);
			segments.get(i).getEndMotionState().setVelocity(cappedEndVelocity);
			previousSegment = segments.get(i);
		}
	}
	
	/**
	 * Generates the {@link TrapezoidalMotionProfile} for each {@link PathSegment}.
	 * The {@link TrapezoidalMotionProfile} is what controls the velocity for each path segment, and since the
	 * {@link MotionState}s should have been adjusted to follow the {@link VelocityConstraints} of this path, the
	 * {@link TrapezoidalMotionProfile} is calculated by passing in the two motion states.
	 */
	public void generateMotionProfiles() {
		for (int i = 0; i < segments.size(); i++) {
			//Generate the motion profile given the segment's start motion state, end motion state, and the path's velocity constraints
			TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(segments.get(i).getStartMotionState(),
					segments.get(i).getEndMotionState(),
					new VelocityConstraints(velocityConstraints.getMaxAcceleration(), velocityConstraints.getMaxDeceleration(), segments.get(i).getMaxVelocity()));
			segments.get(i).setVelocityMotionProfile(motionProfile);
			segments.get(i).getStartMotionState().setT(segments.get(Math.max(0, i - 1)).getEndMotionState().getT());
			segments.get(i).getEndMotionState().setT(segments.get(Math.max(0, i - 1)).getEndMotionState().getT() + motionProfile.getTotalTime());
		}
	}
	
	public Transform[] getWaypoints() {
		return waypoints;
	}
	
	public MotionState getStartMotionState() {
		return startMotionState;
	}
	
	public MotionState getEndMotionState() {
		return endMotionState;
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
	
	
	public ArrayList<PathSegment> getSegments() {
		return segments;
	}
	
	public void setSegments(ArrayList<PathSegment> segments) {
		this.segments = segments;
	}
	
	public double getSamples() {
		return samples;
	}
}
