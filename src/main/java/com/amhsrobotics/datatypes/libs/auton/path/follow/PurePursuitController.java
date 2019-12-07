package com.amhsrobotics.datatypes.libs.auton.path.follow;

import com.amhsrobotics.datatypes.libs.datatypes.DrivetrainVelocities;
import com.amhsrobotics.datatypes.libs.util.geometry.DifferentialDriveKinematics;
import com.amhsrobotics.datatypes.libs.util.geometry.Position;
import com.amhsrobotics.datatypes.libs.util.geometry.Transform;

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