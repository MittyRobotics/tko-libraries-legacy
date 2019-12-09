package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;

public class ArcPathSegment extends PathSegment {
	
	private ArcSegment arcSegment;
	
	public ArcPathSegment(ArcSegment arcSegment) {
		this.arcSegment = arcSegment;
	}
	
	public ArcSegment getArcSegment() {
		return arcSegment;
	}
}
