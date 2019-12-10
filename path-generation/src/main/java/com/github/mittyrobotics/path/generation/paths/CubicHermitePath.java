package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.geometry.LineSegment;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.util.datatypes.ArcPathSegment;
import com.github.mittyrobotics.path.generation.util.datatypes.LinePathSegment;
import com.github.mittyrobotics.path.generation.util.datatypes.PathSegment;
import com.github.mittyrobotics.path.generation.util.splines.CubicHermiteSpline;

import java.util.ArrayList;

public class CubicHermitePath extends Path {
	private final double samples;
	
	public CubicHermitePath(Transform[] waypoints, MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints, double samples) {
		super(waypoints, startMotionState, endMotionState, velocityConstraints);
		this.samples = samples;
	}
	
	/**
	 * Generates the path segments for the {@link CubicHermitePath}.
	 * <p>
	 * This creates a cubic hermite spline in between every two <code>waypoint</code>s and stitches them together into one {@link Path}.
	 */
	@Override
	public void generatePathSegments() {
		ArrayList<PathSegment> segments = new ArrayList<>();
		
		double totalRoughDistance = 0;
		for (int i = 0; i < getWaypoints().length - 1; i++) {
			totalRoughDistance += getWaypoints()[i].getPosition().distance(getWaypoints()[i + 1].getPosition());
		}
		
		for (int i = 0; i < getWaypoints().length - 1; i++) {
			double roughDistance = getWaypoints()[i].getPosition().distance(getWaypoints()[i + 1].getPosition());
			double roughDistanceRatio = roughDistance / totalRoughDistance;
			double currentSamples = samples * roughDistanceRatio;
			
			currentSamples = 3 * (Math.ceil(currentSamples));
			Transform[] points = new CubicHermiteSpline(getWaypoints()[i], getWaypoints()[i + 1], (int) currentSamples).generateSpline();
			for (int a = 0; a < points.length - 2; a += 2) {
				if (new Line(points[a].getPosition(), points[a + 2].getPosition()).isCollinear(points[a + 1].getPosition(), 0.5)) {
					LineSegment line = new LineSegment(points[a].getPosition(), points[a + 2].getPosition());
					segments.add(new LinePathSegment(line));
				} else {
					ArcSegment arc = new ArcSegment(points[a].getPosition(), points[a + 1].getPosition(), points[a + 2].getPosition());
					segments.add(new ArcPathSegment(arc));
				}
			}
		}
		setSegments(segments);
	}
	

}
