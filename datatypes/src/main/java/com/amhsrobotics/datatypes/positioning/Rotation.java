package com.amhsrobotics.datatypes.positioning;

public class Rotation {
	private double heading;
	
	public Rotation(){
		this(0);
	}
	
	public Rotation(double heading){
		this.heading = heading;
	}
	
	public double getHeading() {
		return heading;
	}
	
	public void setHeading(double heading) {
		this.heading = heading;
	}
}
