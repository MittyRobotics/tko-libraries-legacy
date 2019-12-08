package com.amhsrobotics.datatypes.geometry;

import com.amhsrobotics.datatypes.positioning.Position;
import com.amhsrobotics.datatypes.positioning.Transform;

public class Circle {
	private Position center;
	private double radius;
	
	public Circle(Position center, double radius) {
		this.center = center;
		this.radius = radius;
	}
	
	public Circle(Position p1, Position p2, Position p3) {
		Circle circle = getCircleFromPoints(p1,p2,p3);
		this.center = circle.center;
		this.radius = circle.radius;
	}
	
	public Circle(Transform tangentPoint, Position intersection) {
	
	}
	
	/**
	 * Returns the {@link Circle} defined by the three {@link Position}s
	 *
	 * Finds the center of the circle using the circumcenter formula.
	 * https://www.desmos.com/calculator/tditgey5ot
	 *
	 * @param p1 the first {@link Position}
	 * @param p2 the second {@link Position}
	 * @param p3 the third {@link Position}
	 * @return a new {@link Circle} that intersects all three {@link Position}s
	 */
	public Circle getCircleFromPoints(Position p1, Position p2, Position p3) {
		double x1 = p1.getX();
		double y1 = p1.getY();
		double x2 = p2.getX();
		double y2 = p2.getY();
		double x3 = p3.getX();
		double y3 = p3.getY();
		
		double cX = (x1*x1*y2-x1*x1*y3-x2*x2*y1+x2*x2*y3+x3*x3*y1-x3*x3*y2+y1*y1*y2-y1*y2*y2-y1*y1*y3+y1*y3*y3+y2*y2*y3-y2*y3*y3)/(2*(x1*y2-x3*y2-x1*y3-x2*y1+x3*y1+x2*y3));
		double cY = (x1*x1*x2-x1*x1*x3-x1*x2*x2+x1*x3*x3-x1*y2*y2+x1*y3*y3+x2*x2*x3-x2*x3*x3+x2*y1*y1-x2*y3*y3-x3*y1*y1+x3*y2*y2)/(2*(-x1*y2+x1*y3+x2*y1-x2*y3-x3*y1+x3*y2));
		
		double radius = Math.sqrt(Math.pow(cX - x1, 2) + Math.pow(cY - y1, 2));
		return new Circle(new Position(cX, cY), radius);
	}
	
	/**
	 * Finds the two {@link Position}s of intersection between two {@link Circle}s, this {@link Circle} and <code>other</code>.
	 *
	 * This returns an array, which could either contain 0, 1, or 2 points. Make sure to check how many points of
	 * intersection there are in the array after using the function.
	 *
	 * https://www.desmos.com/calculator/ymkxfwsa1k
	 *
	 * @param other the other {@link Circle}
	 * @return an array containing the points of intersection between the two {@link Circle}s
	 */
	public Position[] circleCircleIntersection(Circle other){
		double a = getCenter().getX();
		double b = getCenter().getY();
		double c = other.getCenter().getX();
		double d = other.getCenter().getY();
		double r = getRadius();
		double s = other.getRadius();
		
		double R = getCenter().distance(other.getCenter());
		
		//If there are no points of intersection, return an empty array.
		if(R > r+s){
			return new Position[]{};
		}
		
		double a1 = Math.acos((r * r + R * R - s * s) / (2 * r * R));
		double v1 = Math.atan2(d - b, c - a);
		
		double v = r*Math.cos(a1+v1);
		double g = r*Math.sin(a1+v1);
		double k = r*Math.cos(a1-v1);
		double j = r*Math.sin(a1-v1);
		
		//this center is on the left of other center
		if(a < c){
			double x = a+v;
			double y = b+g;
			double x1 = a+k;
			double y1 = b-j;
			return new Position[]{
					new Position(x,y),
					new Position(x1,y1)
			};
		}
		//this center is on the right of other center
		else{
			double x = a-v;
			double y = b-g;
			double x1 = a-k;
			double y1 = b-j;
			return new Position[]{
					new Position(x,y),
					new Position(x1,y1)
			};
		}
	}
	
	/**
	 * Finds the two {@link Position}s of intersection between this {@link Circle} and {@link Line} <code>other</code>.
	 *
	 * This returns an array, which could either contain 0, 1, or 2 points. Make sure to check how many points of
	 * intersection there are in the array after using the function.
	 *
	 * https://www.desmos.com/calculator/dwwixupcxt
	 *
	 * @param line the other {@link Line}
	 * @return an array containing the points of intersection between the {@link Line} <code>other</code> and this {@link Circle}
	 */
	public Position[] circleLineIntersection(Line line){
		double a = getCenter().getX();
		double b = getCenter().getY();
		double r = getRadius();
		double c = 0;
		double d = line.getYIntercept();
		double f = 1;
		double g = line.getYIntercept()+line.getSlope();
		double m = line.getSlope();
		
		
		double v = (a + f * m * m + b * m - g * m) / (m * m + 1);
		double v1 = (m * a + b * m * m + g - f * m) / (m * m + 1);
		double sqrt = Math.sqrt(r * r - Math.pow(a - v, 2) - Math.pow(b - v1, 2));
		double sqrt1 = Math.sqrt(Math.pow(g - d, 2) + Math.pow(f - c, 2));
		Position pos1 = new Position(
				v +(f-c)* sqrt / sqrt1,
				v1 +(g-d)* sqrt / sqrt1
		);
		Position pos2 = new Position(
				v - (f-c)* sqrt / sqrt1,
				v1 - (g-d)* sqrt / sqrt1
		);
		
		return new Position[]{
				pos1,
				pos2
		};
	}
	
	public Position getCenter() {
		return center;
	}
	
	public double getRadius() {
		return radius;
	}
}
