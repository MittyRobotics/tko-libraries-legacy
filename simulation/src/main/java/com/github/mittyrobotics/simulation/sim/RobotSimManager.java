/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.simulation.sim;


import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;

import javax.swing.*;

public class RobotSimManager implements Runnable {
	
	
	private static RobotSimManager instance = new RobotSimManager();
	private double periodTime;
	private double mass;
	private double driveGearRatio;
	private double driveWheelRadius;
	private double robotWidth;
	private double robotLength;
	private SimRobot robot;
	private SimDrivetrain drivetrain;
	private boolean calledInit = false;
	
	
	private RobotSimManager() {
	
	}
	
	public static RobotSimManager getInstance() {
		return instance;
	}
	
	public void setupRobotSimManager(SimRobot robot, SimDrivetrain drivetrain, double robotMass, double driveGearRatio, double driveWheelRadius, double robotWidth, double robotLength, double periodTime) {
		calledInit = false;
		setupDrivetrainProperties(robotMass, driveGearRatio, driveWheelRadius, robotWidth, robotLength);
		setupRobot(robot, drivetrain);
		setPeriodTime(periodTime);
		initHardware();
		init();
	}
	
	private void setupRobot(SimRobot robot, SimDrivetrain drivetrain) {
		this.robot = robot;
		this.drivetrain = drivetrain;
	}
	
	private void setupDrivetrainProperties(double mass, double gearRatio, double wheelRadius, double width, double length) {
		this.mass = mass;
		this.driveGearRatio = gearRatio;
		this.driveWheelRadius = wheelRadius;
		this.robotWidth = width;
		this.robotLength = length;
	}
	
	@Override
	public void run() {
		while (true) {
			while (!calledInit) {
			}
			periodic();
			try {
				Thread.sleep((long) (periodTime * 1000));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	private void initHardware() {
		drivetrain.initDrivetrain();
		new Thread(this).start();
	}
	
	private void init() {
		robot.robotInit();
		calledInit = true;
	}
	
	/**
	 * Place all of the updated functions in this
	 */
	private void periodic() {
		robot.robotPeriodic();
		drivetrain.odometry();
		SwingUtilities.invokeLater(() -> {
			RobotGraph.getInstance().graphRobot(new Transform(drivetrain.getRobotX(), drivetrain.getRobotY(), drivetrain.getHeading()), robotWidth, robotLength);
		});
	}
	
	public double getPeriodTime() {
		return periodTime;
	}
	
	private void setPeriodTime(double periodTime) {
		this.periodTime = periodTime;
	}
	
	public double getMass() {
		return mass;
	}
	
	public double getDriveGearRatio() {
		return driveGearRatio;
	}
	
	public double getDriveWheelRadius() {
		return driveWheelRadius;
	}
	
	public double getRobotWidth() {
		return robotWidth;
	}
	
	public double getRobotLength() {
		return robotLength;
	}
}
