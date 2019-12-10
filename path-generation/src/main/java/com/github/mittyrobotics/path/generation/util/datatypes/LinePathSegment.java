package com.github.mittyrobotics.path.generation.util.datatypes;

import com.github.mittyrobotics.datatypes.geometry.LineSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.path.generation.util.enums.PathSegmentType;

public class LinePathSegment extends PathSegment{
	private LineSegment lineSegment;
	
	public LinePathSegment(LineSegment lineSegment) {
		super(PathSegmentType.LINE, lineSegment.getFirstPoint(), lineSegment.getSecondPoint());
		this.lineSegment = lineSegment;
		setSegmentDistance(getStartPoint().distance(getEndPoint()));
	}

	@Override
	public LineSegment getLineSegment() {
		return lineSegment;
	}

	@Override
	public Position getClosestPointOnSegment(Position referencePosition) {
		return lineSegment.getClosestPoint(referencePosition);
	}

}
