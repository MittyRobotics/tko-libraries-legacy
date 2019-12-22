package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.positioning.Transform;

public class PathTransform extends Transform {
	private double t;
	
	public PathTransform(Transform transform, double t) {
		super(transform);
		this.t = t;
	}
	
	public double getTOnPath() {
		return t;
	}
	
	public void setTOnPath(double t) {
		this.t = t;
	}
}
