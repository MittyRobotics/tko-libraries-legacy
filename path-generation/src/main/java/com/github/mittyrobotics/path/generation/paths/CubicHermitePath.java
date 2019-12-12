package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.geometry.LineSegment;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.datatypes.ArcPathSegment;
import com.github.mittyrobotics.path.generation.datatypes.LinePathSegment;
import com.github.mittyrobotics.path.generation.datatypes.PathSegment;
import com.github.mittyrobotics.path.generation.splines.CubicHermiteSpline;

import java.util.ArrayList;

public class CubicHermitePath extends Path {
	
	/**
	 * Constructs a {@link CubicHermitePath}.
	 * <p>
	 * The {@link CubicHermitePath} is a {@link Path} where the {@link PathSegment}s are generated using a
	 * {@link CubicHermiteSpline} algorithm.
	 *
	 * @param waypoints           the waypoint {@link Transform}s for the {@link Path} to pass through
	 * @param startMotionState    the starting {@link MotionState} of the {@link Path}.
	 * @param endMotionState      the ending {@link MotionState} of the {@link Path}.
	 * @param velocityConstraints the {@link VelocityConstraints} of the {@link Path}.
	 * @param samples             How many path segments to generate for the {@link Path}. In other words, the amount of samples
	 *                            the path generation algorithm uses to generate points.
	 */
	public CubicHermitePath(Transform[] waypoints, MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints, double samples) {
		super(waypoints, startMotionState, endMotionState, velocityConstraints, samples);
	}
	
	/**
	 * Constructs a {@link CubicHermitePath}. This also takes in a <code>curvatureGain</code> and a
	 * <code>minSlowedVelocity</code>, which are used in slowing down {@link ArcPathSegment}s based on how sharp their
	 * curvature is.
	 * <p>
	 * The {@link CubicHermitePath} is a {@link Path} where the {@link PathSegment}s are generated using a
	 * {@link CubicHermiteSpline} algorithm.
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
	public CubicHermitePath(Transform[] waypoints, MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints, double samples, double curvatureGain, double minSlowedVelocity) {
		super(waypoints, startMotionState, endMotionState, velocityConstraints, samples, curvatureGain, minSlowedVelocity);
	}
	
	/**
	 * Generates the path segments for the {@link CubicHermitePath}.
	 * <p>
	 * This creates a cubic hermite spline in between every two <code>waypoint</code>s and stitches them together into one {@link Path}.
	 */
	@Override
	public void generatePathSegments() {
		ArrayList<PathSegment> segments = new ArrayList<>();
		
		//Find the rough total distance (the sum of all distance between two consecutive waypoints)
		double totalRoughDistance = 0;
		for (int i = 0; i < getWaypoints().length - 1; i++) {
			totalRoughDistance += getWaypoints()[i].getPosition().distance(getWaypoints()[i + 1].getPosition());
		}
		
		for (int i = 0; i < getWaypoints().length - 1; i++) {
			//Find the rough distance between each two waypoints and calculate how many samples each segment should
			//have based on the ratio of the rough distances.
			double roughDistance = getWaypoints()[i].getPosition().distance(getWaypoints()[i + 1].getPosition());
			double roughDistanceRatio = roughDistance / totalRoughDistance;
			double currentSamples = getSamples() * roughDistanceRatio;
			
			//Round up currentSamples and
			currentSamples = 3 * (Math.ceil(currentSamples));
			
			//Generate the CubicHermiteSpline for the points i and i+1
			Transform[] points = new CubicHermiteSpline(getWaypoints()[i], getWaypoints()[i + 1], (int) currentSamples).generateSpline();
			
			//Loop through all the points and create path segments between every 3 points
			//The segment type is determined by if the 3 points are collinear. If they are, we create a line between
			//them. If not, we create an arc between them.
			for (int a = 0; a < points.length - 2; a += 2) {
				PathSegment segment;
				if (new Line(points[a].getPosition(), points[a + 2].getPosition()).isCollinear(points[a + 1].getPosition(), 0.01)) {
					//Create a new line segment with a and a+2 as the end points
					LineSegment line = new LineSegment(points[a].getPosition(), points[a + 2].getPosition());
					segment = new LinePathSegment(line);
		
				} else {
					//Create a new arc segment with a and a+2 as the end points and a+1 and the intermediate point
					ArcSegment arc = new ArcSegment(points[a].getPosition(), points[a + 2].getPosition(), points[a + 1].getPosition());
					segment = new ArcPathSegment(arc);
				}
				segments.add(segment);
			}
		}
		setSegments(segments);
	}
}