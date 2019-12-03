package com.amhsrobotics.libs.auton.path.follow;

import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;
import com.amhsrobotics.libs.util.path.PathSegment;

public class PurePursuitController {
	
	private final Path path;
	

	
	private PathSegment currentClosestSegment;
	
	public PurePursuitController(Path path){
		this.path = path;
	}
	
	
	public Position getClosestSegmentPoint(Transform robotTransform){
		return currentClosestSegment.getParrallelIntersection(robotTransform);
	}
	
	public PathSegment getCurrentClosestSegment() {
		return currentClosestSegment;
	}
	
	public Path getPath() {
		return path;
	}
}
