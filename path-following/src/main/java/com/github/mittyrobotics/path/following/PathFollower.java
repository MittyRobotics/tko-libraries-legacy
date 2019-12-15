package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.TrapezoidalMotionProfile;
import com.github.mittyrobotics.path.following.enums.PathFollowingType;
import com.github.mittyrobotics.path.following.followers.PurePursuitController;
import com.github.mittyrobotics.path.following.followers.RamseteController;
import com.github.mittyrobotics.path.generation.datatypes.PathSegment;
import com.github.mittyrobotics.path.generation.datatypes.TransformWithSegment;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;
import com.github.mittyrobotics.path.generation.paths.Path;

public class PathFollower {
	private static PathFollower instance = new PathFollower();

	private Path currentPath;
	private double purePursuitLookaheadDistance;
	private PathFollowingType pathFollowingType;
	private boolean reversed;

	public static PathFollower getInstance() {
		return instance;
	}

	private PathFollower(){

	}

	public void setupPurePursuit(Path path, boolean reversed){
		setupPurePursuit(path,PurePursuitController.DEFAULT_LOOKAHEAD_DISTANCE,reversed);
	}

	public void setupPurePursuit(Path path, double lookaheadDistance, boolean reversed){
		this.pathFollowingType = PathFollowingType.PURE_PURSUIT_CONTROLLER;
		this.currentPath = path;
		this.purePursuitLookaheadDistance = lookaheadDistance;
		this.reversed = reversed;
	}

	public void setupRamseteController(Path path, boolean reversed){
		setupRamseteController(path,RamseteController.DEFAULT_AGGRESSIVE_GAIN,RamseteController.DEFAULT_DAMPING_GAIN,reversed);
	}

	public void setupRamseteController(Path path, double aggressiveGain, double dampingGain, boolean reversed){
		this.pathFollowingType = PathFollowingType.RAMSETE_CONTROLLER;
		this.currentPath = path;
		this.reversed = false;
		RamseteController.getInstance().setGains(aggressiveGain,dampingGain);
	}

	public DrivetrainVelocities updatePathFollower(Transform robotTransform){
		if(currentPath == null){
			System.out.println("WARNING: The current path follower path is null!");
			return new DrivetrainVelocities(0,0);
		}
		if(pathFollowingType == PathFollowingType.PURE_PURSUIT_CONTROLLER){
			return updatePurePursuit(robotTransform);
		}
		else if(pathFollowingType == PathFollowingType.RAMSETE_CONTROLLER){
			return updateRamsete(robotTransform);
		}
		else{
			System.out.println("WARNING: Unspecified path follower type");
			return new DrivetrainVelocities(0,0);
		}
	}

	private DrivetrainVelocities updatePurePursuit(Transform robotTransform){
		TransformWithSegment closestTransformWithSegment = currentPath.getClosestTransformWithSegment(robotTransform,0,reversed);
		PathSegment closestSegment = closestTransformWithSegment.getSegment();
		Transform closestPosition = closestTransformWithSegment.getTransform();
		
		TransformWithSegment targetTransformWithSegment = currentPath.getClosestTransformWithSegment(robotTransform,purePursuitLookaheadDistance,reversed);
		PathSegment targetSegment = targetTransformWithSegment.getSegment();
		Transform targetPosition = targetTransformWithSegment.getTransform();
		
		TrapezoidalMotionProfile velocityMotionProfile = closestSegment.getVelocityMotionProfile();

		double robotVelocity = velocityMotionProfile.getVelocityAtPosition(closestSegment.getDistanceAlongSegment(closestPosition.getPosition()));

		Transform targetTransform;
		if(closestSegment.getPathSegmentType() == PathSegmentType.ARC){
			targetTransform = targetPosition;
		}
		else{
			targetTransform = targetPosition;
		}

		return PurePursuitController.getInstance().calculate(robotTransform,targetTransform,robotVelocity);
	}

	private DrivetrainVelocities updateRamsete(Transform robotTransform){
		TransformWithSegment closestTransformWithSegment = currentPath.getClosestTransformWithSegment(robotTransform,0,reversed);
		PathSegment closestSegment = closestTransformWithSegment.getSegment();
		Transform closestPosition = closestTransformWithSegment.getTransform();
		TrapezoidalMotionProfile velocityMotionProfile = closestSegment.getVelocityMotionProfile();

		double robotVelocity = velocityMotionProfile.getVelocityAtPosition(closestSegment.getDistanceAlongSegment(closestPosition.getPosition()));

		Transform desiredTransform;
		double turningRadius;
		if(closestSegment.getPathSegmentType() == PathSegmentType.ARC){
			desiredTransform = closestPosition;
			turningRadius = closestSegment.getArcSegment().getRadius();
			double side = new Line(robotTransform.getPosition(),
					robotTransform.getPosition().add(
							new Position(robotTransform.getRotation().cos()*5,
									robotTransform.getRotation().sin()*5))).findSide(
											closestSegment.getArcSegment().getCenter());
			turningRadius = turningRadius * side;
		}
		else{
			desiredTransform = closestPosition;
			turningRadius = 2e16;
		}

		return RamseteController.getInstance().calculate(robotTransform,desiredTransform,robotVelocity,turningRadius);
	}
}
