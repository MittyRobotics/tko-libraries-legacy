package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.following.datatypes.DifferentialDriveKinematics;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.paths.Path;

public class Main {
	public static void main(String[] args) {
		Transform[] waypoints = new Transform[]{
				new Transform(0,0,0),
				new Transform(100,24,0)
		};

		Path path = new CubicHermitePath(waypoints,new MotionState(0), new MotionState(0), new VelocityConstraints(5,5,20),20);

		Transform robotTransform = new Transform(90,24,0);

		DifferentialDriveKinematics.getInstance().setTrackWidth(20);

		PathFollower.getInstance().setupPurePursuit(path,false);
		System.out.println(PathFollower.getInstance().updatePathFollower(robotTransform));
	}
}
