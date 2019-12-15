package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.following.datatypes.DifferentialDriveKinematics;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.paths.Path;
import com.github.mittyrobotics.simulation.sim.RobotSimManager;
import com.github.mittyrobotics.simulation.util.SimSampleDrivetrain;
import com.github.mittyrobotics.simulation.util.SimSampleRobot;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;

public class Main {
	public static void main(String[] args) {
		Transform[] waypoints = new Transform[]{
				new Transform(0, 0, 0),
				//new Transform(50,100,0),
				new Transform(100, 100, 0)
		};
		
		//Setup path follower stuff
		Path path = new CubicHermitePath(waypoints, new MotionState(2), new MotionState(0), new VelocityConstraints(20, 20, 50), 20);
		DifferentialDriveKinematics.getInstance().setTrackWidth(20);
		PathFollower.getInstance().setupPurePursuit(path, false);
		
		//Setup robot sim stuff
		SimSampleRobot robot = new SimSampleRobot();
		RobotSimManager.getInstance().setupRobotSimManager(robot, SimSampleDrivetrain.getInstance(), 125, 7, 2, 20, 30, 0.02);
		SimSampleDrivetrain.getInstance().setupPIDFValues(0.001, 0, 0, 0.1);
		
		//Setup graph stuff
		RobotGraph.getInstance().resizeGraph(-20, 120, -20, 120);
		SimSampleDrivetrain.getInstance().setOdometry(0, 100, 0);
		
		while (true) {
			DrivetrainVelocities output = PathFollower.getInstance().updatePathFollower(SimSampleDrivetrain.getInstance().getRobotTransform());
			SimSampleDrivetrain.getInstance().setVelocities(output.getLeftVelocity(), output.getRightVelocity());
			System.out.println(output + " " + SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
