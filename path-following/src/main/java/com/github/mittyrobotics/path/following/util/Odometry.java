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

package com.github.mittyrobotics.path.following.util;

import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class Odometry {
	private static Odometry instance;
	
	private Transform robotTransform;
	
	private double robotHeading = 0;
	private double robotX = 0;
	private double robotY = 0;
	private double lastLeftEncoderPos = 0;
	private double lastRightEncoderPos = 0;
	private double calibrateGyroVal = 0;
	
	private double ticksPerInch = 0;
	
	public static Odometry getInstance() {
		return instance;
	}
	
	/**
	 * Updates the {@link Odometry}. This should be updated frequently with the current gencoder and gyro values.
	 *
	 * @param leftEncoder   the left wheel encoder value of the drivetrain.
	 * @param rightEncocder the right wheel encoder value of the drivetrain.
	 * @param heading       the heading value of the gyro.
	 */
	public void update(double leftEncoder, double rightEncocder, double heading) {
		if (ticksPerInch == 0) {
			System.out.println("WARNING: Odometry.java ticks per inch is not setup! Call Odometry.getInstance().setTicksPerInch to set the value.");
		} else {
			//Update robot values based on encoder and gyro output
			
			//Get robot heading relative to the calibrated value
			robotHeading = heading - calibrateGyroVal;
			if (robotHeading < 0) {
				robotHeading = robotHeading + 360;
			}
			
			//Get robot rotation
			Rotation robotRotation = new Rotation(robotHeading).mapHeading180();
			
			//Get delta left and right encoder pos
			double deltaLeftPos = leftEncoder - lastLeftEncoderPos;
			double deltaRightPos = rightEncocder - lastRightEncoderPos;
			
			//Get average delta encoder pos in inches
			double deltaPosition = (deltaLeftPos + deltaRightPos) / 2 / ticksPerInch;
			
			//Get x and y values from heading and delta pos
			robotX += deltaPosition * robotRotation.cos();
			robotY += deltaPosition * robotRotation.sin();
			
			//Set last encoder positions
			lastLeftEncoderPos = leftEncoder;
			lastRightEncoderPos = rightEncocder;
			
			robotHeading = robotRotation.getHeading();
			
			robotTransform = new Transform(robotX, robotY, robotRotation);
		}
	}
	
	public void calibrateToZero(double leftEncoder, double rightEncoder, double heading) {
		calibrateGyroVal = robotTransform.getRotation().getHeading();
		lastLeftEncoderPos = leftEncoder;
		lastRightEncoderPos = rightEncoder;
		setRobotTransform(new Transform(0, 0, 0));
	}
	
	public Transform getRobotTransform() {
		return robotTransform;
	}
	
	public void setRobotTransform(Transform robotTransform) {
		this.robotTransform = robotTransform;
		robotX = robotTransform.getPosition().getX();
		robotY = robotTransform.getPosition().getY();
		robotHeading = robotTransform.getRotation().getHeading();
	}
	
	public double getTicksPerInch() {
		return ticksPerInch;
	}
	
	public void setTicksPerInch(double ticksPerInch) {
		this.ticksPerInch = ticksPerInch;
	}
}
