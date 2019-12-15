package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.geometry.LineSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;

import java.util.Optional;

public class LinePathSegment extends PathSegment {
	private LineSegment lineSegment;
	
	public LinePathSegment(LineSegment lineSegment) {
		super(PathSegmentType.LINE, lineSegment.getFirstPoint(), lineSegment.getSecondPoint());
		this.lineSegment = lineSegment;
		setSegmentDistance(lineSegment.getSegmentLength());
	}
	
	@Override
	public LineSegment getLineSegment() {
		return lineSegment;
	}
	
	@Override
	public Optional<Transform> getClosestPointOnSegment(Transform referenceTransform) {
		return lineSegment.getClosestPointOnSegment(referenceTransform, 0, RoundMode.ROUND_CLOSEST);
	}
	
	@Override
	public Optional<Transform> getClosestPointOnSegment(Transform referenceTransform, double distanceShift, RoundMode roundMode) {
		return lineSegment.getClosestPointOnSegment(referenceTransform, distanceShift, roundMode);
	}
	
	@Override
	public double getDistanceAlongSegment(Position position) {
		return lineSegment.getDistanceToPoint(position);
	}
	
	@Override
	public boolean isOnSegment(Position position) {
		return lineSegment.isOnSegment(position);
	}
}
