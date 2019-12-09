package com.github.mittyrobotics.path.generation.datatypes;

import com.github.mittyrobotics.datatypes.positioning.Position;

public abstract class PathSegment {
	private Position startPoint;
	private Position endPoint;
	private double startVelocity;
	private double endVelocity;
}
