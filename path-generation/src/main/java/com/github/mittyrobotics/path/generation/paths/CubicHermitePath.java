package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.splines.CubicHermiteSpline;

public class CubicHermitePath extends Path {
	public CubicHermitePath(Transform[] waypoints) {
		super(waypoints);
	}
	
	@Override
	public void generateParametricEquations() {
		CubicHermiteSpline[] splines = new CubicHermiteSpline[getWaypoints().length - 1];
		for (int i = 0; i < getWaypoints().length - 1; i++) {
			splines[i] = new CubicHermiteSpline(getWaypoints()[i], getWaypoints()[i + 1]);
		}
		setParametrics(splines);
	}
}
