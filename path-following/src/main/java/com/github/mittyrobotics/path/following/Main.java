package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.following.util.DifferentialDriveKinematics;
import com.github.mittyrobotics.path.following.util.PathFollowerProperties;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.paths.Path;
import com.github.mittyrobotics.simulation.sim.RobotSimManager;
import com.github.mittyrobotics.simulation.util.SimOI;
import com.github.mittyrobotics.simulation.util.SimSampleDrivetrain;
import com.github.mittyrobotics.simulation.util.SimSampleRobot;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import javax.swing.*;
import java.awt.*;
import java.util.Random;

public class Main {
	public static void main(String[] args) {
		//Setup the robot sim
		SimSampleRobot robot = new SimSampleRobot();
		RobotSimManager.getInstance().setupRobotSimManager(robot, SimSampleDrivetrain.getInstance(), 125, 7, 2, 20, 30, 0.02);
		RobotGraph.getInstance().getChart().removeLegend();
		SimSampleDrivetrain.getInstance().setupPIDFValues(0.01, 0, 0, 0.08);
		
		//Set track width of differential drive kinematics
		DifferentialDriveKinematics.getInstance().setTrackWidth(20);
		
		//Get random values for robot transform
		Random random = new Random();
		double x = random.nextInt(200) - 200;
		double y = random.nextInt(200) - 100.0;
		double heading = random.nextInt(90) - 45;
		//Set robot transform to random values
		SimSampleDrivetrain.getInstance().setOdometry(x, y, heading);
		
		boolean reversed = false;
		
		//Create the original path from the robot position to the point
		Path originalPath = new CubicHermitePath(new Transform[]{SimSampleDrivetrain.getInstance().getRobotTransform(), new Transform(100, -24, 0)});
		
		if(reversed){
			originalPath = new CubicHermitePath(originalPath.getReversedWaypoints());
			SimSampleDrivetrain.getInstance().setOdometry(originalPath.getStartWaypoint().getPosition().getX(),originalPath.getStartWaypoint().getPosition().getY(),SimSampleDrivetrain.getInstance().getHeading());
		}
		
		//Create velocity controller
		PathVelocityController velocityController = new PathVelocityController(new VelocityConstraints(200, 50, 150), 10, 0);
		
		//Create path properties
		PathFollowerProperties.PurePursuitProperties purePursuitProperties = new PathFollowerProperties.PurePursuitProperties(
				originalPath,
				velocityController,
				reversed,
				20,
				1.2,
				20,
				true,
				false,
				0
		);
		
		PathFollowerProperties.RamseteProperties ramseteProperties = new PathFollowerProperties.RamseteProperties(
				originalPath,
				velocityController,
				reversed,
				2.0,
				.1
		);
		
		//Setup the path follower
		PathFollower.getInstance().setupRamseteController(ramseteProperties);
		
		//Add original path to graph
		RobotGraph.getInstance().addPath((GraphManager.getInstance().graphParametric(originalPath, .1, 2, .1, "spline", Color.green)));
		
		//Delay before starting
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		//Loop
		while (true) {
			//Graph
			SwingUtilities.invokeLater(() -> {
				RobotGraph.getInstance().clearGraph();
				RobotGraph.getInstance().addDataset(GraphManager.getInstance().graphParametricFast(PathFollower.getInstance().getCurrentPath(), .05, "spline", Color.cyan));
			});
			
			//Update pure pursuit controller and set velocities
			DrivetrainVelocities wheelVelocities = PathFollower.getInstance().updatePathFollower(SimSampleDrivetrain.getInstance().getRobotTransform(), SimSampleDrivetrain.getInstance().getAverageVelocity(), RobotSimManager.getInstance().getPeriodTime());
			SimSampleDrivetrain.getInstance().setVelocities(wheelVelocities.getLeftVelocity(), wheelVelocities.getRightVelocity());
			
			try {
				Thread.sleep((long) RobotSimManager.getInstance().getPeriodTime() * 1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
}
