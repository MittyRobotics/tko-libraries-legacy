package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.splines.CubicHermiteSpline;

public class CubicHermitePath extends Path {
	/**
	 * Constructs a {@link CubicHermitePath} given an array of waypoint {@link Transform}s.
	 *
	 * @param waypoints an array of {@link Transform}s that act as waypoints for the path to pass through.
	 */
	public CubicHermitePath(Transform[] waypoints) {
		super(waypoints);
	}
	
	/**
	 * Generates the {@link CubicHermiteSpline} parametric equations based on the waypoints.
	 * <p>
	 * It creates a new path for every 2 waypoints, connecting them together.
	 */
	@Override
	public void generateParametricEquations() {
		CubicHermiteSpline[] splines = new CubicHermiteSpline[getWaypoints().length - 1];
		for (int i = 0; i < getWaypoints().length - 1; i++) {
			splines[i] = new CubicHermiteSpline(getWaypoints()[i], getWaypoints()[i + 1]);
		}
		setParametrics(splines);
	}
	
}
