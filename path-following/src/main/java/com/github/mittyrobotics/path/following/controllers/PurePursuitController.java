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

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.following.util.DifferentialDriveKinematics;

public class PurePursuitController {
	public static final double DEFAULT_CURVATURE_SLOWDOWN_GAIN = 0;
	public static final double DEFAULT_MIN_SLOWDOWN_VELOCITY = 10;
	public static double DEFAULT_LOOKAHEAD_DISTANCE = 20.0;
	private static PurePursuitController instance = new PurePursuitController();
	private double curvatureSlowdownGain;
	private double minSlowdownVelocity;
	
	private PurePursuitController() {
	
	}
	
	public static PurePursuitController getInstance() {
		return instance;
	}
	
	public void setGains(double curvatureSlowdownGain, double minSlowdownVelocity) {
		this.curvatureSlowdownGain = curvatureSlowdownGain;
		this.minSlowdownVelocity = minSlowdownVelocity;
	}
	
	public DrivetrainVelocities calculate(Transform robotTransform, Position targetPosition, double robotVelocity) {
		//Determine if reversed
		boolean reversed = robotVelocity < 0;
		
		//If reversed, flip the robot's transform
		if (reversed) {
			robotTransform.setRotation(robotTransform.getRotation().add(new Rotation(180)));
		}
		
		//Calculate the pursuit circle to follow, calculated by finding the circle tangent to the robot transform that
		//intersects the target position.
		Circle pursuitCircle = new Circle(robotTransform, targetPosition);
		
		//Determine which side the robot transform is on the circle
		double side = new Line(robotTransform.getPosition(),
				robotTransform.getPosition().add(
						new Position(
								robotTransform.getRotation().cos() * 5,
								robotTransform.getRotation().sin() * 5)
				)).findSide(pursuitCircle.getCenter());
		
		robotVelocity = calculateSlowdownVelocity(1 / (pursuitCircle.getRadius()), robotVelocity, minSlowdownVelocity);
		
		//Use differential drive kinematics to calculate the left and right wheel velocity given the base robot
		//velocity and the radius of the pursuit circle
		return DifferentialDriveKinematics.getInstance().calculateFromRadius(
				robotVelocity,
				pursuitCircle.getRadius() * side * (reversed ? -1 : 1)
		);
	}
	
	private double calculateSlowdownVelocity(double curvature, double currentVelocity, double minSlowdownVelocity) {
		if (curvatureSlowdownGain == 0) {
			return currentVelocity;
		}
		double absVelocity = Math.abs(currentVelocity);
		double velSign = Math.signum(currentVelocity);
		double vel = Math.min(absVelocity, Math.max(minSlowdownVelocity, curvatureSlowdownGain / curvature));
		return vel * velSign;
	}
	
	public double getCurvatureSlowdownGain() {
		return curvatureSlowdownGain;
	}
	
	public double getMinSlowdownVelocity() {
		return minSlowdownVelocity;
	}
}
