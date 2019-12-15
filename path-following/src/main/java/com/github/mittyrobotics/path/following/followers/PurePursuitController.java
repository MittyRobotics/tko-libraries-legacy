package com.github.mittyrobotics.path.following.followers;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.following.datatypes.DifferentialDriveKinematics;


public class PurePursuitController {
	private static PurePursuitController instance = new PurePursuitController();
	
	public static double DEFAULT_LOOKAHEAD_DISTANCE = 20.0;
	
	public static PurePursuitController getInstance() {
		return instance;
	}
	
	private PurePursuitController(){
	
	}
	
	public DrivetrainVelocities calculate(Transform robotTransform, Transform targetTransform, double robotVelocity){
		Circle pursuitCircle = new Circle(robotTransform,targetTransform.getPosition());
		double side = new Line(robotTransform.getPosition(), robotTransform.getPosition().add(new Position(robotTransform.getRotation().cos()*5,robotTransform.getRotation().sin()*5))).findSide(pursuitCircle.getCenter());
		
		boolean reversed = robotTransform.relativeTo(targetTransform).getPosition().getX() > 0;
		
		if(reversed){
			return DifferentialDriveKinematics.getInstance().calculateFromRadius(-robotVelocity, pursuitCircle.getRadius() * side);
		}
		else {
			return DifferentialDriveKinematics.getInstance().calculateFromRadius(robotVelocity, pursuitCircle.getRadius() * side);
		}
	}
}
