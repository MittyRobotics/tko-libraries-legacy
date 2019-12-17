package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.following.controllers.PurePursuitController;
import com.github.mittyrobotics.path.following.controllers.RamseteController;
import com.github.mittyrobotics.path.following.enums.PathFollowingType;
import com.github.mittyrobotics.path.generation.paths.Path;

public class PathFollower {
	private static PathFollower instance = new PathFollower();
	
	private Path currentPath;
	private double purePursuitLookaheadDistance;
	private PathFollowingType pathFollowingType;
	private boolean reversed;
	
	private PathFollower() {
	
	}
	
	public static PathFollower getInstance() {
		return instance;
	}
	
	public void setupPurePursuit(Path path, boolean reversed) {
		setupPurePursuit(path, PurePursuitController.DEFAULT_LOOKAHEAD_DISTANCE, reversed);
	}
	
	public void setupPurePursuit(Path path, double lookaheadDistance, boolean reversed) {
		this.pathFollowingType = PathFollowingType.PURE_PURSUIT_CONTROLLER;
		this.currentPath = path;
		this.purePursuitLookaheadDistance = lookaheadDistance;
		this.reversed = reversed;
	}
	
	public void setupRamseteController(Path path, boolean reversed) {
		setupRamseteController(path, RamseteController.DEFAULT_AGGRESSIVE_GAIN, RamseteController.DEFAULT_DAMPING_GAIN, reversed);
	}
	
	public void setupRamseteController(Path path, double aggressiveGain, double dampingGain, boolean reversed) {
		this.pathFollowingType = PathFollowingType.RAMSETE_CONTROLLER;
		this.currentPath = path;
		this.reversed = false;
		RamseteController.getInstance().setGains(aggressiveGain, dampingGain);
	}
	
	public DrivetrainVelocities updatePathFollower(Transform robotTransform) {
		if (currentPath == null) {
			System.out.println("WARNING: The current path follower path is null!");
			return new DrivetrainVelocities(0, 0);
		}
		if (pathFollowingType == PathFollowingType.PURE_PURSUIT_CONTROLLER) {
			return updatePurePursuit(robotTransform);
		} else if (pathFollowingType == PathFollowingType.RAMSETE_CONTROLLER) {
			return updateRamsete(robotTransform);
		} else {
			System.out.println("WARNING: Unspecified path follower type");
			return new DrivetrainVelocities(0, 0);
		}
	}
	
	private DrivetrainVelocities updatePurePursuit(Transform robotTransform) {
		return new DrivetrainVelocities(0,0);
	}
	
	private DrivetrainVelocities updateRamsete(Transform robotTransform) {
		return new DrivetrainVelocities(0,0);
	}
}
