package com.amhsrobotics.datatypes.geometry;

import com.amhsrobotics.datatypes.positioning.Position;
import com.amhsrobotics.datatypes.positioning.Transform;

public class Circle {
	private Position center;
	private double radius;
	
	public Circle(Position center, double radius){
		this.center = center;
		this.radius = radius;
	}
	public Circle(Position p1, Position p2, Position p3){
	
	}
	
	public Circle(Transform tangentPoint, Position intersection){
	
	}
	
	/**
	 * Returns the {@link Circle} defined by the three {@link Position}s
	 *
	 * @param p1 the first {@link Position}
	 * @param p2 the second {@link Position}
	 * @param p3 the third {@link Position}
	 * @return a new {@link Circle} that intersects all three {@link Position}s
	 */
	public Circle getCircleFromPoints(Position p1, Position p2, Position p3){
		
		
		return new Circle(new Position(),0);
	}
	
	public Position getCenter() {
		return center;
	}
	
	public double getRadius() {
		return radius;
	}
}
