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
import com.github.mittyrobotics.datatypes.units.Conversions;

public abstract class SimDrivetrain {
	
	
	private SimTalon[] leftTalons;
	private SimTalon[] rightTalons;
	
	private double prevLeftPos = 0;
	private double prevRightPos = 0;
	private double x = 0;
	private double y = 0;
	private double heading = 0;
	
	
	public abstract void initDrivetrain();
	
	public void odometry() {
		double deltaLeftPos = getLeftMasterTalon().getPosition() - prevLeftPos;
		double deltaRightPos = getRightMasterTalon().getPosition() - prevRightPos;
		
		double deltaPos = (deltaLeftPos + deltaRightPos) / 2;
		
		heading = Math.toRadians(heading);
		
		heading -= Math.atan2((deltaLeftPos - deltaRightPos), RobotSimManager.getInstance().getRobotWidth());
		
		x += Math.cos(heading) * deltaPos;
		y += Math.sin(heading) * deltaPos;
		
		heading = Math.toDegrees(heading);
		
		prevLeftPos = getLeftMasterTalon().getPosition();
		prevRightPos = getRightMasterTalon().getPosition();
	}
	
	public void setupSimDriveTalons(SimTalon[] leftTalons, SimTalon[] rightTalons) {
		double massPerSide = RobotSimManager.getInstance().getMass() * Conversions.LBS_TO_KG / 2;
		double massPerLeftSide = massPerSide / leftTalons.length;
		double massPerRightSide = massPerSide / rightTalons.length;
		for (int i = 0; i < leftTalons.length; i++) {
			new Thread(leftTalons[i]).start();
			leftTalons[i].getModel().initSystemModel(massPerLeftSide, RobotSimManager.getInstance().getDriveGearRatio(), RobotSimManager.getInstance().getDriveWheelRadius() * Conversions.IN_TO_M);
		}
		for (int i = 0; i < rightTalons.length; i++) {
			new Thread(rightTalons[i]).start();
			rightTalons[i].getModel().initSystemModel(massPerRightSide, RobotSimManager.getInstance().getDriveGearRatio(), RobotSimManager.getInstance().getDriveWheelRadius() * Conversions.IN_TO_M);
		}
		this.leftTalons = leftTalons;
		this.rightTalons = rightTalons;
	}
	
	public void setOdometry(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
	
	public Transform getRobotTransform() {
		return new Transform(getRobotX(), getRobotY(), getHeading());
	}
	
	public double getRobotX() {
		return x;
	}
	
	public double getRobotY() {
		return y;
	}
	
	public double getHeading() {
		return heading;
	}
	
	public double getAverageVelocity() {
		return (getLeftMasterTalon().getVelocity() + getRightMasterTalon().getVelocity()) / 2;
	}
	
	public SimTalon getLeftMasterTalon() {
		return leftTalons[0];
	}
	
	public SimTalon getRightMasterTalon() {
		return rightTalons[0];
	}
	
	public SimTalon[] getLeftTalons() {
		return leftTalons;
	}
	
	public SimTalon[] getRightTalons() {
		return rightTalons;
	}
	
}
