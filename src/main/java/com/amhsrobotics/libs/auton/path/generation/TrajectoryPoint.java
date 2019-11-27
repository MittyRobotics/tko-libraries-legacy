package com.amhsrobotics.libs.auton.path.generation;

import java.awt.geom.Point2D;

public class TrajectoryPoint {
	
	private double x;
	private double y;
	private double position;
	private double velocity;
	private double curvature;
	private double angle;
	private double time;
	
	public TrajectoryPoint(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public double distance(TrajectoryPoint trajectoryPoint){
		return Point2D.distance(getX(),getY(),trajectoryPoint.getX(),trajectoryPoint .getY());
	}
	
	public double getX() {
		return x;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public double getY() {
		return y;
	}
	
	public void setY(double y) {
		this.y = y;
	}
	
	public double getPosition() {
		return position;
	}
	
	public void setPosition(double position) {
		this.position = position;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}
	
	public double getCurvature() {
		return Math.abs(curvature);
	}
	public double getRawCurvature(){
		return curvature;
	}
	
	public void setCurvature(double curvature) {
		this.curvature = curvature;
	}
	
	public double getTime() {
		return time;
	}
	
	public void setTime(double time) {
		this.time = time;
	}
	
	public double getAngle() {
		return angle;
	}
	
	public void setAngle(double angle) {
		this.angle = angle;
	}
}
