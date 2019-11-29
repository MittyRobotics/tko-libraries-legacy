package com.amhsrobotics.libs.math.geometry;

import java.awt.geom.Point2D;

/**
 * A Transform object that holds a {@link Position} and {@link Rotation} object, making up the
 * robot's overall position on a 2d coordinate plane. Unlike WPILib's Transform2d object, which is used for
 * transformations of Pose2d objects, this Transform object is used to store the robot's actual position.
 */
public class Transform {
	private Position position;
	private Rotation rotation;
	
	public Transform(){
		this(new Position(), new Rotation());
	}
	
	public Transform(Position position){
		this(position, new Rotation());
	}
	
	public Transform(Rotation rotation){
		this(new Position(), rotation);
	}
	
	public Transform(double x, double y){
		this(new Position(x,y), new Rotation());
	}
	
	public Transform(double x, double y, double heading){
		this(new Position(x,y), new Rotation(heading));
	}
	
	public Transform(double x, double y, Rotation rotation){
		this(new Position(x,y),rotation);
	}
	
	public Transform(Position position, double heading){
		this(position, new Rotation(heading));
	}
	
	public Transform(Position position, Rotation rotation){
		this.position = position;
		this.rotation = rotation;
	}
	
	public Position getPosition() {
		return position;
	}
	
	public Rotation getRotation() {
		return rotation;
	}
	
	/**
	 * Finds the {@link Arc} that intersects p0, this point, and p1.
	 *
	 * @param p0 another point on the circle
	 * @param p1 another point on the circle
	 * @return the {@link Arc} object intersecting with the three points.
	 */
	public Arc findIntersectingArc(Transform p0, Transform p1){
		return position.findIntersectingCircle(p0.getPosition(), p1.getPosition());
	}
	
	/**
	 * Finds the {@link Arc} that is tangent to this {@link Transform} and intersects with the other {@link Transform} position.
	 *
	 * @param other the point that the {@link Arc} intersects
	 * @return the {@link Arc} tangent to this and intersecting other
	 */
	public Arc findTangentIntersectionArc(Transform other){
		
		final Position vectorHead = new Position(getRotation().cos(), getRotation().sin());
		
		final double a = other.getPosition().getX() - getPosition().getX();
		final double b = other.getPosition().getY() -getPosition().getY();
		final double c = vectorHead.getX();
		final double d = vectorHead.getY();
		
		final double v = 2 * ((a * d) - (b * c));
		final double x = (d * ((a * a) + (b * b))) / v;
		final double y = -(c * (a * a + b * b)) / v;
		
		final Position relativeCircleCenter = new Position(x, y);
		
		Position center = new Position(relativeCircleCenter.getX() + getPosition().getX(), relativeCircleCenter.getY() + getPosition().getY());
		
		Position pB = new Position(getPosition().getX() + getRotation().cos() * 20, getPosition().getY() + getRotation().sin() * 20);
		
		double sign = center.findSide(getPosition(), pB);
		
		double radius = Math.sqrt(relativeCircleCenter.getX() * relativeCircleCenter.getX() + relativeCircleCenter.getY() * relativeCircleCenter.getY()) * sign;
		
		return new Arc(center,radius);
	}
	
	/**
	 * Finds the point that intersects the line defined by this {@link Transform} and the other {@link Transform}.
	 *
	 * @return the {@link Transform} containing the {@link Position} of the intersecting point.
	 */
	public Transform findLineIntersectionPoint(Transform other){
		Line l1 = new Line(this);
		Line l2 = new Line(other);
		
		double m1 = l1.getSlope();
		double b1 = l1.getYIntercept();
		double m2 = l2.getSlope();
		double b2 = l2.getYIntercept();
		
		//Parallel lines, no intersection
		if(m1 == m2){
			return new Transform();
		}
		
		System.out.println(m1 + " " + m2);
		
		double x = (b2-b1)/(m1-m2);
		double y = m1*x+b1;
		
		return new Transform(new Position(x,y));
	}
	
	/**
	 * Finds the point along the line defined by this {@link Transform} that is a certain distance away from this {@link Transform}s position
	 *
	 * @param distance the distance along the line to find the point
	 * @return the {@link Transform} containing the {@link Position} of the point that is distance away from this {@link Position}
	 */
	public Transform findPointDistanceAlongLine(double distance){
		double x = getRotation().cos() * distance;
		double y = getRotation().sin() * distance;
		return new Transform(x,y).transformBy(this);
	}
	
	/**
	 * Finds the side that this point is on the line defined by p0 and p1.
	 *
	 * @param p0 first point of the line
	 * @param p1 second point of the line
	 * @return side that this point is on the line, +1 for left, -1 for right
	 */
	public double findSide(Transform p0, Transform p1){
		return position.findSide(p0.getPosition(), p1.getPosition());
	}
	
	public Transform multiply(double scalar){
		Position pos = position.multiply(scalar);
		Rotation rot = rotation.multiply(scalar);
		
		return new Transform(pos,rot);
	}
	
	
	public Transform divide(double scalar){
		Position pos = position.divide(scalar);
		Rotation rot = rotation.divide(scalar);
		
		return new Transform(pos,rot);
	}
	
	public Transform add(Transform other){
		Position pos = position.add(other.getPosition());
		Rotation rot = rotation.add(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public Transform subtract(Transform other){
		Position pos = position.subtract(other.getPosition());
		Rotation rot = rotation.subtract(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public Transform transformBy(Transform other){
		Position pos = position.add(other.getPosition().rotateBy(rotation));
		Rotation rot = rotation.add(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public Transform relativeTo(Transform other){
		Position pos = position.subtract(other.getPosition()).rotateBy(other.getRotation().inverse());
		Rotation rot = rotation.subtract(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	/**
	 * Rotates this {@link Transform} around the given {@link Transform}.
	 *
	 * @param other the {@link Transform} to rotate around
	 * @return the rotated {@link Transform}
	 */
	public Transform rotateAround(Transform other){
		Position pos = position.subtract(other.getPosition()).rotateBy(other.getRotation()).add(other.getPosition());
		Rotation rot = rotation.add(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public void setPosition(Position position) {
		this.position = position;
	}
	
	public void setRotation(Rotation rotation) {
		this.rotation = rotation;
	}
	
	@Override
	public String toString() {
		return String.format("Transform(%s, %s)", position.toString(), rotation.toString());
	}
	
}
