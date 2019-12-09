package com.github.mittyrobotics.path.generation.util.datatypes;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.path.generation.util.enums.PathSegmentType;

public class ArcPathSegment extends PathSegment {
	
	private ArcSegment arcSegment;
	
	public ArcPathSegment(ArcSegment arcSegment) {
		super(PathSegmentType.LINE);
		this.arcSegment = arcSegment;
	}
	
	public ArcSegment getArcSegment() {
		return arcSegment;
	}

	@Override
	public Position getClosestPointOnSegment(Position referencePosition) {
		return null;
	}
}
