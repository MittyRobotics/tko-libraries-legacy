package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.following.controllers.PurePursuitController;
import com.github.mittyrobotics.path.following.controllers.RamseteController;
import com.github.mittyrobotics.path.following.enums.PathFollowingType;
import com.github.mittyrobotics.path.following.util.PathFollowerProperties;
import com.github.mittyrobotics.path.generation.paths.Path;
import com.github.mittyrobotics.simulation.util.SimSampleDrivetrain;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import javax.swing.*;
import java.awt.*;

public class PathFollower {
	private static PathFollower instance = new PathFollower();
	
	private PathFollowingType pathFollowingType;
	
	private PathFollowerProperties pathFollowerProperties;
	private PathFollowerProperties.PurePursuitProperties purePursuitProperties;
	private PathFollowerProperties.RamseteProperties ramseteProperties;
	
	private Path currentPath;
	
	private PathFollower() {
	
	}
	
	public static PathFollower getInstance() {
		return instance;
	}
	
	public void setupPurePursuit(PathFollowerProperties.PurePursuitProperties properties){
		pathFollowingType = PathFollowingType.PURE_PURSUIT_CONTROLLER;
		
		setupPathFollower(properties);
		this.purePursuitProperties = properties;
		
		PurePursuitController.getInstance().setGains(purePursuitProperties.curvatureSlowdownGain,purePursuitProperties.minSlowdownVelocity);
	}
	
	public void setupRamseteController(PathFollowerProperties.RamseteProperties properties) {
		this.pathFollowingType = PathFollowingType.RAMSETE_CONTROLLER;
		
		setupPathFollower(properties);
		this.ramseteProperties = properties;
		
		RamseteController.getInstance().setGains(ramseteProperties.aggressiveGain,ramseteProperties.dampingGain);
	}
	
	private void setupPathFollower(PathFollowerProperties properties){
		this.pathFollowerProperties = properties;
		this.currentPath = properties.path;
	}
	
	public void changePath(Path newPath) {
		this.currentPath = newPath;
	}
	
	public DrivetrainVelocities updatePathFollower(Transform robotTransform, double currentVelocity, double deltaTime) {
		calculateAdaptivePath(robotTransform);
		
		if (currentPath == null) {
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
		double lookaheadDistance = purePursuitProperties.lookaheadDistance;
		boolean reversed = pathFollowerProperties.reversed;
		
		Position lookaheadCalculationStartPosition;
		
		if(purePursuitProperties.adaptiveLookahead){
			Position closestPosition = currentPath.getClosestTransform(robotTransform.getPosition(),  10, 3).getPosition();
			lookaheadCalculationStartPosition = closestPosition;
		}
		else{
			lookaheadCalculationStartPosition = robotTransform.getPosition();
		}
		
		Position targetPosition = currentPath.getClosestTransform(lookaheadCalculationStartPosition, lookaheadDistance, reversed, 10, 3).getPosition();
		
		//Find the rough distance to the end of the path
		double distanceToEnd = getDistanceToEnd(robotTransform,20);
		
		//Calculate the robot velocity using the path velocity controller
		double robotVelocity = pathFollowerProperties.velocityController.getVelocity(currentVelocity, distanceToEnd, deltaTime);
		
		System.out.println(robotVelocity + " robot");
		
		//Calculate the pure pursuit controller
		return PurePursuitController.getInstance().calculate(robotTransform, targetPosition, robotVelocity);
	}
	
	private DrivetrainVelocities updateRamsete(Transform robotTransform, double currentVelocity, double deltaTime) {
		return new DrivetrainVelocities(0, 0);
	}
	
	private void calculateAdaptivePath(Transform robotTransform){
		if(pathFollowerProperties.adaptivePath){
			changePath(currentPath.calculateAdaptedPath(robotTransform,pathFollowerProperties.robotToPathAdaptiveDistance,pathFollowerProperties.reversed));
		}
	}
	
	public boolean isFinished(Transform robotTransform, double distanceTolerance){
		return getDistanceToEnd(robotTransform,0) < distanceTolerance;
	}
	
	private double getDistanceToEnd(Transform robotTransform, double stopDistanceTolerance) {
		double distance = robotTransform.getPosition().distance(currentPath.getWaypoints()[currentPath.getWaypoints().length - 1].getPosition());
		if (currentPath.getWaypoints()[currentPath.getWaypoints().length - 1].relativeTo(robotTransform).getPosition().getX() <= 0 && distance <= stopDistanceTolerance) {
			return 0;
		}
		return distance;
	}
	
	public Path getCurrentPath() {
		return currentPath;
	}
	
}
