package com.github.mittyrobotics.path.following.controllers;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.following.util.DifferentialDriveKinematics;

public class PurePursuitController {
	public static double DEFAULT_LOOKAHEAD_DISTANCE = 20.0;
	private static PurePursuitController instance = new PurePursuitController();
	
	private PurePursuitController() {
	
	}
	
	public static PurePursuitController getInstance() {
		return instance;
	}
	
	public DrivetrainVelocities calculate(Transform robotTransform, Transform targetTransform, double robotVelocity) {
		boolean reversed = robotVelocity < 0;
		if (reversed) {
			robotTransform.setRotation(robotTransform.getRotation().add(new Rotation(180)));
		}
		Circle pursuitCircle = new Circle(robotTransform, targetTransform.getPosition());
		System.out.println(robotTransform + " " + targetTransform + "asdf" + " " + robotVelocity + " " + pursuitCircle.getRadius() + " " + pursuitCircle.getCenter());
		double side = new Line(robotTransform.getPosition(), robotTransform.getPosition().add(new Position(robotTransform.getRotation().cos() * 5, robotTransform.getRotation().sin() * 5))).findSide(pursuitCircle.getCenter());
		
		return DifferentialDriveKinematics.getInstance().calculateFromRadius(robotVelocity, pursuitCircle.getRadius() * side);
	}
}
