package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public abstract class Path {
	private Transform[] waypoints;
	private Parametric[] parametrics;
	
	public Path(Transform[] waypoints) {
		this.waypoints = waypoints;
		generateParametricEquations();
	}
	
	public abstract void generateParametricEquations();
	
	public Transform getTransform(double t) {
		//Convert t from 0 to 1 to 0 to waypoints.length-1 so that the t value represents all parametric equation
		//segments of the total path.
		t = t * parametrics.length;
		
		for (int i = 0; i < parametrics.length; i++) {
			//Find which parametric equation segment that t falls in
			if (t >= i && t <= i + 1) {
				//return the Transform from the segment that it falls in
				return parametrics[i].getTransform(t - i);
			}
		}
		return new Transform();
	}
	
	public Transform[] getWaypoints() {
		return waypoints;
	}
	
	public Parametric[] getParametrics() {
		return parametrics;
	}
	
	public void setParametrics(Parametric[] parametrics) {
		this.parametrics = parametrics;
	}
}
