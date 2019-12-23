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

package com.github.mittyrobotics.path.following.controllers;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.path.following.util.DifferentialDriveKinematics;

public class RamseteController {
	public static double DEFAULT_AGGRESSIVE_GAIN = 2.0;
	public static double DEFAULT_DAMPING_GAIN = 0.2;
	private static RamseteController instance = new RamseteController();
	
	private double aggressiveGain; //(x > 0), makes movement more aggressive
	private double dampingGain; //(0 < x < 1) provides damping
	
	private RamseteController() {
		setGains();
	}
	
	public static RamseteController getInstance() {
		return instance;
	}
	
	public void setGains() {
		setGains(DEFAULT_AGGRESSIVE_GAIN, DEFAULT_DAMPING_GAIN);
	}
	
	/**
	 * Sets the gains for the {@link PurePursuitController}.
	 *
	 * @param aggressiveGain (x > 0), makes the robot's movement more aggressive to follow the path. Larger values
	 *                       result in more aggressive movement.
	 * @param dampingGain    (0 < x < 1), provides damping to the controller. Larger values results in more damping.
	 */
	public void setGains(double aggressiveGain, double dampingGain) {
		this.aggressiveGain = aggressiveGain;
		this.dampingGain = dampingGain;
	}
	
	/**
	 * Calculates the {@link DrivetrainVelocities} based on the {@link RamseteController} path following algorithm.
	 *
	 * @param robotTransform   the robot's current {@link Transform}.
	 * @param desiredTransform the current desired {@link Transform} of the robot, the position on the path you want
	 *                         the robot to be at.
	 * @param robotVelocity    the current desired linear velocity of the robot.
	 * @param turningRadius    the current desired radius the robot should be turning.
	 * @return
	 */
	public DrivetrainVelocities calculate(Transform robotTransform, Transform desiredTransform, double robotVelocity, double turningRadius) {
		//Get the transform error in meters.
		Transform error = desiredTransform.relativeTo(robotTransform).inToM();
		//Calculate linear velocity in meters per second given robot velocity in inches per second
		double linearVelocity = robotVelocity * Conversions.IN_TO_M;
		//Calculate the angular velocity in radians per second given the turning radius and the robot velocity.
		double angularVelocity = linearVelocity / (turningRadius * Conversions.IN_TO_M);
		
		double eX = error.getPosition().getX();
		double eY = error.getPosition().getY();
		double eTheta = error.getRotation().getRadians();
		
		//Calculate the Ramsete k value
		double k = 2.0 * dampingGain * Math.sqrt(Math.pow(angularVelocity, 2) + aggressiveGain * Math.pow(linearVelocity, 2));
		
		//Calculate the adjusted linear velocity from the Ramsete algorithm
		double adjustedLinearVelocity = linearVelocity * error.getRotation().cos() + k * eX;
		
		//Convert linear velocity back into inches per second
		adjustedLinearVelocity = adjustedLinearVelocity * Conversions.M_TO_IN;
		
		//Calculate the adjusted angular velocity from the Ramsete algorithm (stays in radians per second)
		double adjustedAngularVelocity = angularVelocity + k * eTheta + aggressiveGain * linearVelocity * error.getRotation().sinc() * eY;
		
		//Use differential drive kinematics given linear velocity in inches per second and angular velocity in radians per second
		return DifferentialDriveKinematics.getInstance().calculateFromAngularVelocity(adjustedLinearVelocity, adjustedAngularVelocity);
	}
	
	public double getAggressiveGain() {
		return aggressiveGain;
	}
	
	public double getDampingGain() {
		return dampingGain;
	}
}
