package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;

import javax.xml.transform.Transformer;

public abstract class Path {
	private Transform[] waypoints;
	private Parametric[] parametrics;
	
	public Path(Transform[] waypoints) {
		this.waypoints = waypoints;
		generateParametricEquations();
	}
	
	public abstract void generateParametricEquations();
	
	public Position getPosition(double t) {
		//Convert t from 0 to 1 to 0 to waypoints.length-1 so that the t value represents all parametric equation
		//segments of the total path.
		t = t * parametrics.length;
		
		for (int i = 0; i < parametrics.length; i++) {
			//Find which parametric equation segment that t falls in
			if (t >= i && t <= i + 1) {
				//return the Transform from the segment that it falls in
				return parametrics[i].getPosition(t - i);
			}
		}
		return new Position();
	}
	
	public Transform getTransform(double t) {
		//Convert t from 0 to 1 to 0 to waypoints.length-1 so that the t value represents all parametric equation
		//segments of the total path.
		t = t * parametrics.length;
		
		for (int i = 0; i < parametrics.length; i++) {
			//Find which parametric equation segment that t falls in
			if (t >= i && t <= i + 1) {
				//return the Transform from the segment that it falls in
				return parametrics[i].getTransform(t - i);
			}
		}
		return new Transform();
	}
	
	public Position getClosestPoint(Position referencePosition, double distanceShift, boolean reversed, double steps){
		double minT = 0;
		double maxT = 0;
		double closestDistance = 9999;
		for(int i = 0; i < steps+1; i++){
			double t = i/steps;
			Transform transform = getTransform(t);
			double distance = transform.getPosition().distance(referencePosition);
			if(new Transform(referencePosition).relativeTo(transform).getPosition().getX() > 0){
				if(distance < closestDistance){
					closestDistance = distance;
					minT = t;
					maxT = t + 1/steps;
				}
			}
		}

		System.out.println(minT + " " + maxT);

		closestDistance = 9999;
		for(double t = minT; t < maxT; t += 0.001){
			double distance = getPosition(t).distance(referencePosition);
			if(distance < closestDistance){
				closestDistance = distance;
				minT = t;
			}
		}

		return getPosition(minT);
	}
	
	public Transform[] getWaypoints() {
		return waypoints;
	}
	
	public Parametric[] getParametrics() {
		return parametrics;
	}
	
	public void setParametrics(Parametric[] parametrics) {
		this.parametrics = parametrics;
	}
}
