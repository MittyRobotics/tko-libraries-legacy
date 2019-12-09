package com.github.mittyrobotics.libs.auton.path.follow;

import com.github.mittyrobotics.libs.util.geometry.DifferentialDriveKinematics;
import com.github.mittyrobotics.libs.util.geometry.Position;
import com.github.mittyrobotics.libs.util.geometry.Transform;
import com.github.mittyrobotics.libs.datatypes.DrivetrainVelocities;

public class PurePursuitController {

	private static PurePursuitController instance = new PurePursuitController();

	public static PurePursuitController getInstance() {
		return instance;
	}

	private PurePursuitController(){

	}



	public DrivetrainVelocities purePursuitController(Position targetPos, double robotVelocity){
		double radius = new Transform(0,0,0).findTangentIntersectionArc(targetPos).getRadius();
		return DifferentialDriveKinematics.getDrivetrainVelocitiesFromRadius(robotVelocity,radius,0);
	}
}