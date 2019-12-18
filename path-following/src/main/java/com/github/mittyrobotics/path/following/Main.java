package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datacollection.performance.TimeMonitor;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.following.util.DifferentialDriveKinematics;
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
		Path path = new CubicHermitePath(new Transform[]{new Transform(0, 0, 0), new Transform(100, 24, 0), new Transform(150, 24, 0)});
		PathFollower.getInstance().setupPurePursuit(
				path,
				30, 1.2,
				20,
				false,
				new PathVelocityController(
						new VelocityConstraints(
								50,
								20,
								100),
						5,
						0
				)
		);
		
		SimSampleRobot robot = new SimSampleRobot();
		RobotSimManager.getInstance().setupRobotSimManager(robot, SimSampleDrivetrain.getInstance(), 125, 7, 2, 20, 30, 0.06);
		
		SimSampleDrivetrain.getInstance().setupPIDFValues(0.001, 0, 0, 0.1);
		DifferentialDriveKinematics.getInstance().setTrackWidth(20);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		Random random = new Random();
		
		double x = random.nextInt(200) - 150.0;
		double y = random.nextInt(200) - 100.0;
		double heading = random.nextInt(90) - 45;
		
		SimSampleDrivetrain.getInstance().setOdometry(x, y, heading);
		
	
		

		
		RobotGraph.getInstance().getChart().removeLegend();
		

		
		double adjustPathCount = 0;
		
		while (true) {


			adjustPathCount++;
			if(adjustPathCount >=2000){
				RobotGraph.getInstance().clearGraph();
				
				adjustPathCount = 0;
				path = new CubicHermitePath(new Transform[]{SimSampleDrivetrain.getInstance().getRobotTransform(),new Transform(150, 0,0)});
				PathFollower.getInstance().changePath(path);
	
				Path finalPath = path;
				SwingUtilities.invokeLater(new Runnable() {
					@Override
					public void run() {
						RobotGraph.getInstance().addDataset(GraphManager.getInstance().graphParametricFast(finalPath, 0.1,  "spline", Color.cyan));
					}
				});
			}
			

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
