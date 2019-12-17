package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.following.controllers.PurePursuitController;
import com.github.mittyrobotics.path.following.util.DifferentialDriveKinematics;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.paths.Path;
import com.github.mittyrobotics.simulation.sim.RobotSimManager;
import com.github.mittyrobotics.simulation.util.SimSampleDrivetrain;
import com.github.mittyrobotics.simulation.util.SimSampleRobot;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;
import edu.wpi.first.wpilibj.SampleRobot;

public class Main {
	public static void main(String[] args) {
		Path path = new CubicHermitePath(new Transform[]{new Transform(0,0,0), new Transform(100,24,0), new Transform(150,24,0)});
		PathFollower.getInstance().setupPurePursuit(
				path,
				20,
				1.2,
				20,
				false,
				new PathVelocityController(
						new VelocityConstraints(
								50,
								50,
								100),
						5,
						0
				)
		);
		
		SimSampleRobot robot = new SimSampleRobot();
		RobotSimManager.getInstance().setupRobotSimManager(robot, SimSampleDrivetrain.getInstance(), 125, 7, 2, 20, 30, 0.06);
		
		SimSampleDrivetrain.getInstance().setupPIDFValues(0.001,0,0,0.1);
		DifferentialDriveKinematics.getInstance().setTrackWidth(20);
		
		while(true){
			DrivetrainVelocities wheelVelocities = PathFollower.getInstance().updatePathFollower(SimSampleDrivetrain.getInstance().getRobotTransform(),SimSampleDrivetrain.getInstance().getAverageVelocity(),RobotSimManager.getInstance().getPeriodTime());
			
			System.out.println(wheelVelocities);
			
			SimSampleDrivetrain.getInstance().setVelocities(wheelVelocities.getLeftVelocity(),wheelVelocities.getRightVelocity());
			
			try {
				Thread.sleep((long)RobotSimManager.getInstance().getPeriodTime()*1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
