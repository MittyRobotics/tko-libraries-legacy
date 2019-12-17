package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.following.controllers.PurePursuitController;
import com.github.mittyrobotics.path.following.controllers.RamseteController;
import com.github.mittyrobotics.path.following.enums.PathFollowingType;
import com.github.mittyrobotics.path.generation.paths.Path;

public class PathFollower {
	private static PathFollower instance = new PathFollower();
	
	private double purePursuitLookaheadDistance;
	private PathFollowingType pathFollowingType;
	private boolean reversed;
	private VelocityConstraints velocityConstraints;
	private double curvatureSlowdownGain;
	private PathVelocityController pathVelocityController;
	private Path path;
	
	private PathFollower() {
	
	}
	
	public static PathFollower getInstance() {
		return instance;
	}
	
	public void setupPurePursuit(Path path, boolean reversed, PathVelocityController pathVelocityController) {
		setupPurePursuit(path, PurePursuitController.DEFAULT_LOOKAHEAD_DISTANCE, PurePursuitController.DEFAULT_CURVATURE_SLOWDOWN_GAIN, PurePursuitController.DEFAULT_MIN_SLOWDOWN_VELOCITY, reversed,pathVelocityController);
	}
	
	public void setupPurePursuit(Path path, double curvatureSlowdownGain, boolean reversed, PathVelocityController pathVelocityController) {
		setupPurePursuit(path, PurePursuitController.DEFAULT_LOOKAHEAD_DISTANCE, curvatureSlowdownGain, PurePursuitController.DEFAULT_MIN_SLOWDOWN_VELOCITY, reversed,pathVelocityController);
	}
	
	public void setupPurePursuit(Path path, double lookaheadDistance, double curvatureSlowdownGain, double minSlowdownVelocity, boolean reversed, PathVelocityController pathVelocityController) {
		this.curvatureSlowdownGain = curvatureSlowdownGain;
		this.pathFollowingType = PathFollowingType.PURE_PURSUIT_CONTROLLER;
		this.purePursuitLookaheadDistance = lookaheadDistance;
		PurePursuitController.getInstance().setGains(curvatureSlowdownGain,minSlowdownVelocity);
		followerSetup(path,reversed,pathVelocityController);
	}
	
	public void setupRamseteController(Path path, boolean reversed,PathVelocityController pathVelocityController) {
		setupRamseteController(path, RamseteController.DEFAULT_AGGRESSIVE_GAIN, RamseteController.DEFAULT_DAMPING_GAIN, reversed,pathVelocityController);
	}
	
	public void setupRamseteController(Path path, double aggressiveGain, double dampingGain, boolean reversed, PathVelocityController pathVelocityController) {
		this.pathFollowingType = PathFollowingType.RAMSETE_CONTROLLER;
		RamseteController.getInstance().setGains(aggressiveGain, dampingGain);
		
		followerSetup(path,reversed,pathVelocityController);
	}
	
	private void followerSetup(Path path, boolean reversed, PathVelocityController pathVelocityController){
		this.path = path;
		this.reversed = reversed;
		this.pathVelocityController = pathVelocityController;
	}
	
	public DrivetrainVelocities updatePathFollower(Transform robotTransform, double currentVelocity, double deltaTime) {
		if (path == null) {
			System.out.println("WARNING: The current path follower path is null!");
			return new DrivetrainVelocities(0, 0);
		}
		if (pathFollowingType == PathFollowingType.PURE_PURSUIT_CONTROLLER) {
			return updatePurePursuit(robotTransform, currentVelocity, deltaTime);
		} else if (pathFollowingType == PathFollowingType.RAMSETE_CONTROLLER) {
			return updateRamsete(robotTransform, currentVelocity, deltaTime);
		} else {
			System.out.println("WARNING: Unspecified path follower type");
			return new DrivetrainVelocities(0, 0);
		}
	}
	
	private DrivetrainVelocities updatePurePursuit(Transform robotTransform, double currentVelocity, double deltaTime) {
		Position closestPosition = path.getClosestPoint(robotTransform.getPosition(),0,reversed,10,1000);
		Position targetPosition = path.getClosestPoint(closestPosition,purePursuitLookaheadDistance,reversed,10,1000);
		
		//Find the rough distance to the end of the path
		double distanceToEnd = getDistanceToEnd(robotTransform);
		
		//Calculate the robot velocity using the path velocity controller
		double robotVelocity = pathVelocityController.getVelocity(currentVelocity,distanceToEnd,deltaTime);
		
		//Calculate the pure pursuit controller
		return PurePursuitController.getInstance().calculate(robotTransform,targetPosition,robotVelocity);
	}
	
	private DrivetrainVelocities updateRamsete(Transform robotTransform, double currentVelocity, double deltaTime) {
		return new DrivetrainVelocities(0,0);
	}
	
	private double getDistanceToEnd(Transform robotTransform){
		double distance = robotTransform.getPosition().distance(path.getWaypoints()[path.getWaypoints().length-1].getPosition());
		if(path.getWaypoints()[path.getWaypoints().length-1].relativeTo(robotTransform).getPosition().getX() <= 0){
			return 0;
		}
		return distance;
	}
	
	public double getCurvatureSlowdownGain() {
		return curvatureSlowdownGain;
	}
}
