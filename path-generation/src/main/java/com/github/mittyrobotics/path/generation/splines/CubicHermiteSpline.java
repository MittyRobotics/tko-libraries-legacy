package com.github.mittyrobotics.path.generation.splines;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class CubicHermiteSpline implements Parametric {
	private double x0, x1, y0, y1, a0, a1, d, mx0, mx1, my0, my1;
	
	public CubicHermiteSpline(Transform startWaypoint, Transform endWaypoint) {
		x0 = startWaypoint.getPosition().getX();
		x1 = endWaypoint.getPosition().getX();
		y0 = startWaypoint.getPosition().getY();
		y1 = endWaypoint.getPosition().getY();
		
		//Get angles in radians
		a0 = Math.toRadians(startWaypoint.getRotation().getHeading());
		a1 = Math.toRadians(endWaypoint.getRotation().getHeading());
		
		//Get distance between points
		d = startWaypoint.getPosition().distance(endWaypoint.getPosition());
		
		//Create tangent vectors proportional to the distance between points
		mx0 = Math.cos(a0) * d;
		my0 = Math.sin(a0) * d;
		mx1 = Math.cos(a1) * d;
		my1 = Math.sin(a1) * d;
	}
	
	@Override
	public Position getPosition(double t) {
		//Cubic hermite spline equations https://en.wikipedia.org/wiki/Cubic_Hermite_spline
		double h0, h1, h2, h3;
		h0 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
		h1 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
		h2 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
		h3 = Math.pow(t, 3) - Math.pow(t, 2);
		
		//Get x and y values from cubic hermite spline equations
		double x = h0 * x0 + h1 * mx0 + h2 * x1 + h3 * mx1;
		double y = h0 * y0 + h1 * my0 + h2 * y1 + h3 * my1;
		return new Position(x, y);
	}
	
	@Override
	public Transform getTransform(double t) {
		Position position = getPosition(t);
		
		//To get tangent vector equations (first derivative of cubic hermite spline functions)
		double h0, h1, h2, h3;
		h0 = 6 * t * t - 6 * t;
		h1 = 3 * t * t - 4 * t + 1;
		h2 = -6 * t * t + 6 * t;
		h3 = 3 * t * t - 2 * t;
		
		//Get x and y values for tangent vector from the equations
		double x = h0 * x0 + h1 * mx0 + h2 * x1 + h3 * mx1;
		double y = h0 * y0 + h1 * my0 + h2 * y1 + h3 * my1;
		
		Rotation rotation = new Rotation(Math.toDegrees(Math.atan2(y, x)));
		
		return new Transform(position, rotation);
	}
}
