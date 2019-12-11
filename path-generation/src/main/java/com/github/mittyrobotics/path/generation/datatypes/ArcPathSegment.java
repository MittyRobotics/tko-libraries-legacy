package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;

public class ArcPathSegment extends PathSegment {
	
	private ArcSegment arcSegment;
	
	public ArcPathSegment(ArcSegment arcSegment) {
		super(PathSegmentType.ARC, arcSegment.getStartPoint(), arcSegment.getEndPoint());
		this.arcSegment = arcSegment;
		setSegmentDistance(2*arcSegment.getRadius()*Math.asin(getStartPoint().distance(getEndPoint())/(2*arcSegment.getRadius())));
	}

	@Override
	public ArcSegment getArcSegment() {
		return arcSegment;
	}

	@Override
	public Position getClosestPointOnSegment(Position referencePosition) {
		return arcSegment.getClosestPoint(referencePosition);
	}
}
