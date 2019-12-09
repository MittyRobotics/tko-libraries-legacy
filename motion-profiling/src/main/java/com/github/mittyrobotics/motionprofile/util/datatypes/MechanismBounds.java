package com.github.mittyrobotics.motionprofile.util.datatypes;

public class MechanismBounds {
	private double minPosition;
	private double maxPosition;
	
	public MechanismBounds(double minPosition, double maxPosition) {
		this.minPosition = minPosition;
		this.maxPosition = maxPosition;
	}
	
	public double getMinPosition() {
		return minPosition;
	}
	
	public void setMinPosition(double minPosition) {
		this.minPosition = minPosition;
	}
	
	public double getMaxPosition() {
		return maxPosition;
	}
	
	public void setMaxPosition(double maxPosition) {
		this.maxPosition = maxPosition;
	}
	
}
