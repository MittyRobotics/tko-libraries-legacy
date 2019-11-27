package com.amhsrobotics.libs.geometry;

/**
 * Position object, hold the x and y position of the robot's {@link Transform}. Based on WPILib's Translation2d object:
 * https://github.com/wpilibsuite/allwpilib/blob/master/wpilibj/src/main/java/edu/wpi/first/wpilibj/geometry/Translation2d.java
 */
public class Position {
	private double x;
	private double y;
	
	public Position(){
		this(0,0);
	}
	
	public Position(double x, double y){
		this.x = x;
		this.y = y;
	}
	
	public double distance(Position other){
		return Math.hypot(other.x - x, other.y - y);
	}
	
	public Position rotateBy(Rotation rotation){
		return new Position(x * rotation.cos() - y * rotation.sin(),x * rotation.sin() + y * rotation.cos());
	}
	
	public Position add(Position other){
		return new Position(x + other.x, y + other.y);
	}
	
	public Position subtract(Position other){
		return new Position(x - other.x, y - other.y);
	}
	
	public Position multiply(double scalar){
		return new Position(x*scalar, y*scalar);
	}
	
	public Position divide(double scalar){
		return new Position(x/scalar, y/scalar);
	}
	
	public Position inverse(){
		return new Position(-x,-y);
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
	
	@Override
	public String toString() {
		return String.format("Position(%s, %s)", x, y);
	}
}
