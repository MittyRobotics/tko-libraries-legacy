package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;

import java.util.Optional;

public class ArcPathSegment extends PathSegment {
	
	private ArcSegment arcSegment;
	
	public ArcPathSegment(ArcSegment arcSegment) {
		super(PathSegmentType.ARC, arcSegment.getStartPoint(), arcSegment.getEndPoint());
		this.arcSegment = arcSegment;
		setSegmentDistance(arcSegment.getSegmentLength());
	}
	
	@Override
	public ArcSegment getArcSegment() {
		return arcSegment;
	}
	
	
	@Override
	public Optional<Transform> getClosestPointOnSegment(Transform referenceTransform) {
		return arcSegment.getClosestPointOnSegment(referenceTransform, 0, RoundMode.ROUND_CLOSEST);
	}
	
	@Override
	public Optional<Transform> getClosestPointOnSegment(Transform referenceTransform, double distanceShift, RoundMode roundMode) {
		return arcSegment.getClosestPointOnSegment(referenceTransform, distanceShift, roundMode);
	}
	
	
	@Override
	public double getDistanceAlongSegment(Position position) {
		return arcSegment.getDistanceToPoint(position);
	}
	
	@Override
	public boolean isOnSegment(Position position) {
		return arcSegment.isOnSegment(position);
	}
}
