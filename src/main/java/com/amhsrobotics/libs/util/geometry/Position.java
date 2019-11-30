package com.amhsrobotics.libs.util.geometry;

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
	
	/**
	 * Finds the {@link Arc} that intersects p0, this point, and p1.
	 *
	 * Equation: https://www.geeksforgeeks.org/equation-of-circle-when-three-points-on-the-circle-are-given/
	 *
	 * @param p0 another point on the circle
	 * @param p1 another point on the circle
	 * @return the {@link Arc} object intersecting with the three points.
	 */
	public Arc findIntersectingArc(Position p0, Position p1){
		double x1 = p0.getX();
		double y1 = p0.getY();
		double x2 = getX();
		double y2 = getY();
		double x3 = p1.getX();
		double y3 = p1.getY();
		
		double x12 = x1 - x2;
		double x13 = x1 - x3;
		
		double y12 = y1 - y2;
		double y13 = y1 - y3;
		
		double y31 = y3 - y1;
		double y21 = y2 - y1;
		
		double x31 = x3 - x1;
		double x21 = x2 - x1;
		
		double sx13 = Math.pow(x1, 2) -  Math.pow(x3, 2);
		
		double sy13 =  Math.pow(y1, 2) -  Math.pow(y3, 2);
		
		double sx21 =  Math.pow(x2, 2) -  Math.pow(x1, 2);
		double sy21 =  Math.pow(y2, 2) -  Math.pow(y1, 2);
		
		double f = ((sx13) * (x12)
				+ (sy13) * (x12)
				+ (sx21) * (x13)
				+ (sy21) * (x13))
				/ (2 * ((y31) * (x12) - (y21) * (x13)));
		double g = ((sx13) * (y12)
				+ (sy13) * (y12)
				+ (sx21) * (y13)
				+ (sy21) * (y13))
				/ (2 * ((x31) * (y12) - (x21) * (y13)));
		
		double c = -Math.pow(x1, 2) - Math.pow(y1, 2) - 2 * g * x1 - 2 * f * y1;
		
		double h = -g;
		double k = -f;
		double sqr_of_r = h * h + k * k - c;
		
		// r is the radius
		double r = Math.sqrt(sqr_of_r);
		
		return new Arc(new Position(h,k), r);
	}
	
	/**
	 * Finds the side that this point is on the line defined by p0 and p1.
	 *
	 * @param p0 first point of the line
	 * @param p1 second point of the line
	 * @return side that this point is on the line, +1 for left, -1 for right
	 */
	public double findSide(Position p0, Position p1){
		double x = getX();
		double y = getY();
		double x1 = p0.getX();
		double y1 = p0.getY();
		double x2 = p1.getX();
		double y2 = p1.getY();
		return Math.signum((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1));
	}
	
	/**
	 * Calculates the angle from this {@link Position} to other
	 * @param other
	 * @return angle to from this to other point in degrees
	 */
	public double angleTo(Position other){
		double x = getX() - other.getX();
		double y = getY() - other.getY();
		double angleToLookahead = 180+Math.toDegrees(Math.atan2(y,x));
		return angleToLookahead;
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
