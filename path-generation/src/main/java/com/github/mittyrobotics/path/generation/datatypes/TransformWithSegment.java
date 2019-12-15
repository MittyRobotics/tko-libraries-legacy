package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.positioning.Transform;

public class TransformWithSegment {
	private Transform transform;
	private PathSegment segment;
	
	public TransformWithSegment(Transform transform,PathSegment segment){
		this.segment = segment;
		this.transform = transform;
	}
	
	public Transform getTransform() {
		return transform;
	}
	public PathSegment getSegment() {
		return segment;
	}
}
