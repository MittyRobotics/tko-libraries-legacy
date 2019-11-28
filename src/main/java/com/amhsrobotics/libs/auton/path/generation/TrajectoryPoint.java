package com.amhsrobotics.libs.auton.path.generation;

import com.amhsrobotics.libs.math.geometry.Position;
import com.amhsrobotics.libs.math.geometry.Rotation;
import com.amhsrobotics.libs.math.geometry.Transform;

public class TrajectoryPoint extends Transform {
	
	private double distanceAlongPath;
	private double radius;
	private double angle;
	
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
	
	public double getAngle() {
		return angle;
	}
	
	public void setAngle(double angle) {
		this.angle = angle;
	}
}
