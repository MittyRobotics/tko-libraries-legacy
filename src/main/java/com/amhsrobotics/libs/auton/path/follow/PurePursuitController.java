package com.amhsrobotics.libs.auton.path.follow;

import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.datatypes.DrivetrainVelocities;
import com.amhsrobotics.libs.util.geometry.DifferentialDriveKinematics;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Rotation;
import com.amhsrobotics.libs.util.geometry.Transform;
import com.amhsrobotics.libs.util.path.PathSegment;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class PurePursuitController {

	private static PurePursuitController instance = new PurePursuitController();

	public static PurePursuitController getInstance() {
		return instance;
	}

	private PurePursuitController(){

	}

	public DrivetrainVelocities purePursuitController(Position targetPos, double robotVelocity){
		double radius = new Transform(0,0,0).findTangentIntersectionArc(targetPos).getRadius();
		return DifferentialDriveKinematics.getInstance().getDrivetrainVelocitiesFromRadius(robotVelocity,radius);
	}
}