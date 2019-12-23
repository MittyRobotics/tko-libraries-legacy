package com.github.mittyrobotics.datatypes.motion;

public class DrivetrainVelocities {
	private final double leftVelocity;
	private final double rightVelocity;
	
	public DrivetrainVelocities(double leftVelocity, double rightVelocity) {
		if (Double.isNaN(leftVelocity) || Double.isInfinite(leftVelocity)) {
			this.leftVelocity = 0;
		} else {
			this.leftVelocity = leftVelocity;
		}
		if (Double.isNaN(rightVelocity) || Double.isInfinite(rightVelocity)) {
			this.rightVelocity = 0;
		} else {
			this.rightVelocity = rightVelocity;
		}
	}
	
	public double getLeftVelocity() {
		return leftVelocity;
	}
	
	public double getRightVelocity() {
		return rightVelocity;
	}
	
	public double getAvgVelocity() {
		return (rightVelocity + leftVelocity) / 2;
	}
	
	@Override
	public String toString() {
		return String.format("DrivetrainVelocities(left: %s, right: %s)", leftVelocity, rightVelocity);
	}
}
