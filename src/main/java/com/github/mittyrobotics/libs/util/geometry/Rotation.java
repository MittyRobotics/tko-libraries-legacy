package com.github.mittyrobotics.libs.util.geometry;

/**
 * Rotation object, hold the heading of the robot's {@link Transform}. Our com.amhsrobotics.utils.geometry system is based off of 254's Pose2d system.
 * https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/geometry/Pose2d.java
 */
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
	
	public double getRadians(){
		return Math.toRadians(heading);
	}
	
	public double tan(){
		return Math.tan(getRadians());
	}
	
	public double sin(){
		return Math.sin(getRadians());
	}
	
	public double cos(){
		return Math.cos(getRadians());
	}
	
	public double sinc(){
		return Math.sin(getRadians())/getRadians();
	}
	
	public double inverseAngle(){
		return -getHeading();
	}
	
	public double inverseRadians(){
		return -getRadians();
	}
	
	public Rotation inverse(){
		return new Rotation(inverseAngle());
	}
	
	public Rotation add(Rotation other){
		return rotateBy(other);
	}
	
	public Rotation subtract(Rotation other){
		return rotateBy(other.inverseAngle());
	}
	
	public Rotation multiply(double scalar){
		return new Rotation(heading*scalar);
	}
	
	public Rotation divide(double scalar){
		return new Rotation(heading/scalar);
	}
	
	public Rotation rotateBy(double otherAngle){
		return rotateBy(new Rotation(otherAngle));
	}
	
	public Rotation rotateBy(Rotation other){
		double cos = cos() * other.cos() - sin() * other.sin();
		double sin = cos() * other.sin() + sin() * other.cos();
		return new Rotation(Math.toDegrees(Math.atan2(sin,cos)));
	}
	
	public void setHeading(double heading) {
		this.heading = heading;
	}
	
	public void setRadians(double radians){
		this.heading = Math.toDegrees(radians);
	}
	
	/**
	 * Maps the heading value of the {@link Rotation} between -180 to 180;
	 */
	public Rotation mapHeading(){
		double angle = getHeading();
		double sign = Math.signum(angle);
		angle = Math.abs(angle % 360);
		if (angle <= 180 && angle >= 0) {
			return new Rotation((angle * sign));
		} else {
			return new Rotation((sign * ((angle % 360) % 180 - 180)));
		}
	}
	
	@Override
	public String toString() {
		return String.format("Rotation(%s)", heading);
	}
}
