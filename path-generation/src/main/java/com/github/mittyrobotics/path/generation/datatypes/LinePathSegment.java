package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.geometry.LineSegment;

public class LinePathSegment extends PathSegment{
	private LineSegment lineSegment;
	
	public LinePathSegment(LineSegment lineSegment) {
		
		this.lineSegment = lineSegment;
	}
	
	public LineSegment getLineSegment() {
		return lineSegment;
	}
}
