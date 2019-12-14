package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.paths.Path;

public class PathFollower {
	private static PathFollower instance = new PathFollower();
	
	private Path currentPath;
	
	private double purePursuitLookaheadDistance;
	
	public static PathFollower getInstance() {
		return instance;
	}
	
	private PathFollower(){
	
	}
	
	public void setupPurePursuit(Path path, double lookaheadDistance){
		this.currentPath = path;
		this.purePursuitLookaheadDistance = lookaheadDistance;
	}
	
	public DrivetrainVelocities updatePurePursuit(Transform robotTransform){
		Position closestPosition = PurePursuitController.getInstance().getClosestPositionOnPath(robotTransform,currentPath);
		Position targetPosition = PurePursuitController.getInstance().getTargetPositionOnPath(robotTransform,currentPath,purePursuitLookaheadDistance);
		return new DrivetrainVelocities(0,0);
	}
	
}
