package com.amhsrobotics.libs.datatypes;

public class DrivetrainWheelVelocities {
	private final double leftVelocity;
	private final double rightVelocity;
	
	public DrivetrainWheelVelocities(double leftVelocity, double rightVelocity){
		
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
