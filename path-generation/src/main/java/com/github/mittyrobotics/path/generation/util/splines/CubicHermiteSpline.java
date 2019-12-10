package com.github.mittyrobotics.path.generation.util.splines;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class CubicHermiteSpline {
	
	private final Transform point0;
	private final Transform point1;
	private final int steps;
	
	/**
	 * Constructs a cubic hermite spline between two {@link Transform}s.
	 *
	 * @param point0 the first {@link Transform} of the spline
	 * @param point1 the second {@link Transform} of the spline
	 * @param steps  the samples of the spline (how many points to generate)
	 */
	public CubicHermiteSpline(Transform point0, Transform point1, int steps) {
		this.point0 = point0;
		this.point1 = point1;
		this.steps = steps;
	}
	
	
	/**
	 * Generates the points of the cubic hermite spline.
	 *
	 * <p>
	 * https://en.wikipedia.org/wiki/Cubic_Hermite_spline
	 *
	 * @return an array of {@link Transform}s that make up the points of the spline.
	 */
	public Transform[] generateSpline() {
		Transform[] points = new Transform[steps];
		double x0, x1, y0, y1, a0, a1, d, mx0, mx1, my0, my1;
		//Define x and y variables
		x0 = point0.getPosition().getX();
		x1 = point1.getPosition().getX();
		y0 = point0.getPosition().getY();
		y1 = point1.getPosition().getY();
		
		//Get angles in radians
		a0 = Math.toRadians(point0.getRotation().getHeading());
		a1 = Math.toRadians(point1.getRotation().getHeading());
		
		//Get distance between points
		d = point0.getPosition().distance(point1.getPosition());
		
		//Create tangent vectors proportional to the distance between points
		mx0 = Math.cos(a0) * d;
		my0 = Math.sin(a0) * d;
		mx1 = Math.cos(a1) * d;
		my1 = Math.sin(a1) * d;
		
		double t;
		for (int i = 0; i < steps; i++) {
			double a = 0;
			double b = steps - 1;
			
			//Find t value between 0 and 1 along spline
			t = ((double) i - a) / (b - a);
			if (i == steps - 1) {
				t = 1;
			}
			
			//Cubic hermite spline equations https://en.wikipedia.org/wiki/Cubic_Hermite_spline
			double h0, h1, h2, h3;
			h0 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
			h1 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
			h2 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
			h3 = Math.pow(t, 3) - Math.pow(t, 2);
			
			//Get x and y values from cubic hermite spline equations
			double x = h0 * x0 + h1 * mx0 + h2 * x1 + h3 * mx1;
			double y = h0 * y0 + h1 * my0 + h2 * y1 + h3 * my1;
			points[i] = new Transform(new Position(x, y), new Rotation(0));
		}
		
		//Set the rotations for the points
		for (int i = 0; i < points.length; i++) {
			if (i == points.length - 1) {
				points[i].setRotation(point1.getRotation());
			} else {
				points[i].setRotation(points[i].getPosition().angleTo(points[i + 1].getPosition()));
			}
		}
		
		return points;
	}
	
	public Transform getPoint0() {
		return point0;
	}
	
	public Transform getPoint1() {
		return point1;
	}
	
	public int getSteps() {
		return steps;
	}
}
