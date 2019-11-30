package com.amhsrobotics.libs.auton.path.generation;

import com.amhsrobotics.libs.util.path.TrajectoryPoint;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Rotation;
import com.amhsrobotics.libs.util.geometry.Transform;

public class CubicHermitePath extends Path {

	
	public CubicHermitePath(Transform[] waypoints, VelocityConstraints velocityConstraints, double radiusSlowdownThreshold, int samples){
		super(waypoints, velocityConstraints, radiusSlowdownThreshold,samples);
	}
	
	@Override
	public void generateTrajectoryPoints() {
		int steps = getSamples();
		
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		int prevSegmentLength = 0;
		int stepsPerSegment = steps/(getWaypoints().length-1);
		int addedSteps = 0;
		
		if(stepsPerSegment*(getWaypoints().length-1) < steps){
			addedSteps = (steps-(stepsPerSegment*(getWaypoints().length-1)));
		}
		
		for (int i = 0; i < getWaypoints().length - 1; i++) {
			if(i== getWaypoints().length-2){
				stepsPerSegment+=addedSteps;
			}
			
			TrajectoryPoint[] segment = generateSpline(getWaypoints()[i], getWaypoints()[i + 1], stepsPerSegment, i == 0, i == getWaypoints().length-2);
			for (int a = 0; a < segment.length; a++) {
				tradjectoryPoints[a +prevSegmentLength] = segment[a];
			}
			prevSegmentLength = segment.length+prevSegmentLength;
		}
		setTrajectoryPoints(tradjectoryPoints);
	}
	
	private TrajectoryPoint[] generateSpline(Transform coordinate, Transform coordinate1, int steps, boolean firstSegment, boolean lastSegment) {
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		double x0,x1,y0,y1,a0,a1,d,mx0,mx1,my0,my1;
		x0 = coordinate.getPosition().getX();
		x1 = coordinate1.getPosition().getX();
		y0 = coordinate.getPosition().getY();
		y1 = coordinate1.getPosition().getY();

		a0 = Math.toRadians(coordinate.getRotation().getHeading());
		a1 = Math.toRadians(coordinate1.getRotation().getHeading());

		d = coordinate.getPosition().distance(coordinate1.getPosition());
		mx0 = Math.cos(a0) * d;
		my0 = Math.sin(a0) * d;
		mx1 = Math.cos(a1) * d;
		my1 = Math.sin(a1) * d;
		double t;
		for (int i = 0; i < steps; i++) {
			double a = 0;
			double b = steps-1;
			t = ((double)i - a) / (b - a);
			if(!lastSegment){
				t = Math.max(0, t);
			}

			double h0,h1,h2,h3;

			h0 = 2*Math.pow(t,3)-3*Math.pow(t,2)+1;
			h1 = Math.pow(t,3)-2*Math.pow(t,2)+t;
			h2 = -2*Math.pow(t,3)+3*Math.pow(t,2);
			h3 = Math.pow(t,3)-Math.pow(t,2);

			double x = h0*x0+h1*mx0+h2*x1+h3*mx1;
			double y = h0*y0+h1*my0+h2*y1+h3*my1;
			tradjectoryPoints[i] = new TrajectoryPoint(new Position(x,y), new Rotation(0));
		}
		
		
		return tradjectoryPoints;
	}
	
}
