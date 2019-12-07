package com.amhsrobotics.datatypes.libs.auton.path.generation;

import com.amhsrobotics.datatypes.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.util.geometry.*;
import com.amhsrobotics.datatypes.libs.util.geometry.*;
import com.amhsrobotics.datatypes.libs.util.path.ArcPathSegment;
import com.amhsrobotics.datatypes.libs.util.path.LinePathSegment;
import com.amhsrobotics.datatypes.libs.util.path.PathSegment;

import java.util.ArrayList;

public class CubicHermitePath extends Path {
	
	
	public CubicHermitePath(Transform[] waypoints, VelocityConstraints velocityConstraints, int segmentSamples) {
		super(waypoints, velocityConstraints, segmentSamples);
	}
	
	@Override
	public void generatePath() {
		ArrayList<PathSegment> segments = new ArrayList<>();
		
		double totalRoughDistance = 0;
		for(int i = 0; i < getWaypoints().length-1; i++){
			totalRoughDistance += getWaypoints()[i].getPosition().distance(getWaypoints()[i+1].getPosition());
		}
		
		for(int i = 0; i < getWaypoints().length-1; i++){
			double roughDistance = getWaypoints()[i].getPosition().distance(getWaypoints()[i+1].getPosition());
			double roughDistanceRatio = roughDistance/totalRoughDistance;
			double samples = getSegmentSamples()*roughDistanceRatio;
			
			samples = 3*(Math.ceil(samples));
			Transform[] points = generateSpline(getWaypoints()[i], getWaypoints()[i+1], (int)samples, i == getWaypoints().length-2);
			for(int a = 0; a < points.length-2; a+=2){
				if(new Line(points[a].getPosition(),points[a+2].getPosition()).isColinear(points[a+1].getPosition(),0.5)){
					Line line = new Line(points[a].getPosition(),points[a+2].getPosition());
					segments.add(new LinePathSegment(line,new Transform(line.getP1()),new Transform(line.getP2())));
				}
				else{
					Arc arc = new Arc(points[a].getPosition(), points[a+1].getPosition(), points[a+2].getPosition());
					segments.add(new ArcPathSegment(arc, points[a],points[a+2]));
				}

			}
			//Arc arc = new Arc(points[points.length-3].getPosition(), points[points.length-2].getPosition(), points[points.length-2].getPosition());
			//segments.add(new ArcPathSegment(arc, points[i],points[i+2]));
		}
		

		setSegments(segments);
	}
	
	private Transform[] generateSpline(Transform coordinate, Transform coordinate1, int steps, boolean lastSegment) {
		Transform[] points = new Transform[steps];
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
			
			if(i == steps-1){
				t = 1;
			}
			
			
			double h0,h1,h2,h3;
			
			h0 = 2*Math.pow(t,3)-3*Math.pow(t,2)+1;
			h1 = Math.pow(t,3)-2*Math.pow(t,2)+t;
			h2 = -2*Math.pow(t,3)+3*Math.pow(t,2);
			h3 = Math.pow(t,3)-Math.pow(t,2);
			
			double x = h0*x0+h1*mx0+h2*x1+h3*mx1;
			double y = h0*y0+h1*my0+h2*y1+h3*my1;
			points[i] = new Transform(new Position(x,y), new Rotation(0));
		}
		
		
		return points;
	}
}
