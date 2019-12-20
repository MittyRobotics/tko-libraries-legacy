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
import com.github.mittyrobotics.simulation.sim.SimDrivetrain;
import com.github.mittyrobotics.simulation.util.SimOI;
import com.github.mittyrobotics.simulation.util.SimSampleDrivetrain;
import com.github.mittyrobotics.simulation.util.SimSampleRobot;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
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
		SimSampleDrivetrain.getInstance().setOdometry(x,y,heading);
		
		//Create the original path from the robot position to the point
		Path originalPath = new CubicHermitePath(new Transform[]{SimSampleDrivetrain.getInstance().getRobotTransform(),new Transform(150, 0,90)});
		
		//Setup the pure pursuit controller
		PathFollower.getInstance().setupPurePursuit(
				originalPath,
				30, 1.2,
				20,
				false,
				new PathVelocityController(
						new VelocityConstraints(
								200,
								20,
								100),
						5,
						0
				)
		);
		
		//Init array containing all of the previous transforms
		ArrayList<Transform> lastTransforms = new ArrayList<>();
		
		//Create new adjusted path
		Path path = originalPath;
		PathFollower.getInstance().changePath(path);
		RobotGraph.getInstance().addPath((GraphManager.getInstance().graphParametric(originalPath, .1,2,.1,  "spline", Color.green)));
		
		//Delay before starting
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		//Count to delay the adjusted path calculations
		double adjustPathCount = 0;
		
		//Loop
		while (true) {
			//Increase the adjust path calculation
			adjustPathCount++;
			
			//if(adjustPathCount >= 4000){
				//path = new CubicHermitePath(new Transform[]{lastTransforms.get(0),new Transform(150, 0,0)});
				path = new CubicHermitePath(new Transform[]{new Transform(SimSampleDrivetrain.getInstance().getRobotTransform().getPosition(),
						SimSampleDrivetrain.getInstance().getRobotTransform().getPosition().angleTo(
								originalPath.getClosestPoint(SimSampleDrivetrain.getInstance().getRobotTransform().getPosition(),
										30,false,10,100))),
						new Transform(150, 0,90)});
				//lastTransforms.remove(0);
				PathFollower.getInstance().changePath(path);
		//	}


				SwingUtilities.invokeLater(new Runnable() {
					@Override
					public void run() {
						RobotGraph.getInstance().clearGraph();
						RobotGraph.getInstance().addDataset(GraphManager.getInstance().graphParametricFast(PathFollower.getInstance().getPath(), .01,  "spline", Color.cyan));
					}
				});
				
			
			

			//Update pure pursuit controller and set velocities
			DrivetrainVelocities wheelVelocities = PathFollower.getInstance().updatePathFollower(SimSampleDrivetrain.getInstance().getRobotTransform(), SimSampleDrivetrain.getInstance().getAverageVelocity(), RobotSimManager.getInstance().getPeriodTime());
			SimSampleDrivetrain.getInstance().setVelocities(wheelVelocities.getLeftVelocity(), wheelVelocities.getRightVelocity());
			
			System.out.println(wheelVelocities + " " + new DrivetrainVelocities(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity(),SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity()));
			
			
			lastTransforms.add(SimSampleDrivetrain.getInstance().getRobotTransform());

			try {
				Thread.sleep((long) RobotSimManager.getInstance().getPeriodTime() * 1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
}
