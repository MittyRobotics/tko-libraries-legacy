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
	
	public PathSegment getClosestPathSegment(Transform robotTransform){
		double currentClosest = 9999;
		PathSegment closestSegment = null;
		for(int i = 0; i < path.getSegments().size(); i++){
			Transform startPoint = path.getSegments().get(i).getStartPoint();
			if(startPoint.getPosition().distance(robotTransform.getPosition()) < currentClosest && robotTransform.relativeTo(startPoint).getPosition().getX() >= 0){
				currentClosest = startPoint.getPosition().distance(robotTransform.getPosition());
				closestSegment = path.getSegments().get(i);
			}
		}
		if(closestSegment == null){
			closestSegment=path.getSegments().get(path.getSegments().size());
		}
		currentClosestSegment = closestSegment;
		return closestSegment;
	}
	
	public Position getClosestSegmentPoint(Transform robotTransform){
		return currentClosestSegment.getIntersection(robotTransform);
	}
	
	public PathSegment getCurrentClosestSegment() {
		return currentClosestSegment;
	}
	
	public Path getPath() {
		return path;
	}
}
