package com.github.mittyrobotics.libs.datatypes;

public class DrivetrainVelocities {
	private final double leftVelocity;
	private final double rightVelocity;
	
	public DrivetrainVelocities(double leftVelocity, double rightVelocity){
		
		this.leftVelocity = leftVelocity;
		this.rightVelocity = rightVelocity;
	}
	
	public double getLeftVelocity() {
		return leftVelocity;
	}
	
	public double getRightVelocity() {
		return rightVelocity;
	}
	
	public double getAvgVelocity(){
		return (rightVelocity + leftVelocity)/2;
	}
}
