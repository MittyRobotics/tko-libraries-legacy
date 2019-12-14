package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.paths.Path;


public class PurePursuitController {
	private static PurePursuitController instance = new PurePursuitController();
	
	public static PurePursuitController getInstance() {
		return instance;
	}
	
	private PurePursuitController(){
	
	}
	
	public DrivetrainVelocities calculate(Transform robotTransform, Position targetPosition, double robotVelocity){
		Circle pursuitCircle = new Circle(robotTransform,targetPosition);
		return DifferentialDriveKinematics.getInstance().calculateFromRadius(robotVelocity,pursuitCircle.getRadius());
	}
	
	public Position getClosestPositionOnPath(Transform robotTransform, Path path){
		return path.getClosestSegment(robotTransform.getPosition(),0).getClosestPointOnSegment(robotTransform.getPosition()).get();
	}
	
	public Position getTargetPositionOnPath(Transform robotTransform, Path path, double lookaheadDistance){
		return path.getClosestSegment(robotTransform.getPosition(),lookaheadDistance).getClosestPointOnSegment(robotTransform.getPosition(),lookaheadDistance, RoundMode.ROUND_UP).get();
	}
}
