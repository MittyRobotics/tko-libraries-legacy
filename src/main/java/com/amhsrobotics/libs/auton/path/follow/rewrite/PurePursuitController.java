package com.amhsrobotics.libs.auton.path.follow.rewrite;

import com.amhsrobotics.libs.auton.path.generation.rewrite.Path;
import com.amhsrobotics.libs.util.geometry.Transform;
import com.amhsrobotics.libs.util.path.PathSegment;

public class PurePursuitController {
	
	private final Path path;
	
	public PurePursuitController(Path path){
		this.path = path;
	}
	
	public PathSegment getClosestPathSegment(Transform robotTransform){
		double currentClosest = 9999;
		PathSegment closestSegment = null;
		for(int i = 0; i < path.getSegments().size(); i++){
			Transform startPoint = path.getSegments().get(i).getStartPoint();
			if(startPoint.getPosition().distance(robotTransform.getPosition()) < currentClosest && startPoint.relativeTo(robotTransform).getPosition().getX() < 0){
				currentClosest = startPoint.getPosition().distance(robotTransform.getPosition());
				closestSegment = path.getSegments().get(i);
			}
		}
		return closestSegment;
	}
	
	public Path getPath() {
		return path;
	}
}
