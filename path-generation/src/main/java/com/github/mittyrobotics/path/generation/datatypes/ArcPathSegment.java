package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;

import java.util.Optional;

public class ArcPathSegment extends PathSegment {
	
	private ArcSegment arcSegment;
	
	public ArcPathSegment(ArcSegment arcSegment) {
		super(PathSegmentType.ARC, arcSegment.getStartPoint(), arcSegment.getEndPoint());
		this.arcSegment = arcSegment;
		setSegmentDistance(2 * arcSegment.getRadius() * Math.asin(getStartPoint().distance(getEndPoint()) / (2 * arcSegment.getRadius())));
	}
	
	@Override
	public ArcSegment getArcSegment() {
		return arcSegment;
	}
	
	@Override
	public Optional<Position> getClosestPointOnSegment(Position referencePosition) {
		return arcSegment.getClosestPointOnSegment(referencePosition, 0, RoundMode.ROUND_CLOSEST);
	}
	
	@Override
	public Optional<Position> getClosestPointOnSegment(Position referencePosition, double distanceShift, RoundMode roundMode) {
		return arcSegment.getClosestPointOnSegment(referencePosition, distanceShift, roundMode);
	}
	
	@Override
	public boolean isOnSegment(Position position) {
		return arcSegment.isOnSegment(position);
	}
}
