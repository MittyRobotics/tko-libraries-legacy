package com.amhsrobotics.libs.auton.path.generation;

import com.amhsrobotics.libs.math.geometry.Position;
import com.amhsrobotics.libs.math.geometry.Rotation;
import com.amhsrobotics.libs.math.geometry.Transform;

public class TrajectoryPoint extends Transform {
	
	private double distanceAlongPath;
	private double velocity;
	private double radius;
	private double time;
	
	
	public TrajectoryPoint(Position position, Rotation rotation) {
		super(position,rotation);
	}
	
	public double getDistanceAlongPath() {
		return distanceAlongPath;
	}
	
	public void setDistanceAlongPath(double distanceAlongPath) {
		this.distanceAlongPath = distanceAlongPath;
	}
	
	public double getRadius() {
		return radius;
	}
	
	public void setRadius(double radius) {
		this.radius = radius;
	}
	
	public double getCurvature(){
		return 1/radius;
	}
	
	
	public double getTime() {
		return time;
	}
	
	public void setTime(double time) {
		this.time = time;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}
}
