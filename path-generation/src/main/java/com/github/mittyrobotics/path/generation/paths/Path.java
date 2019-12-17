package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;

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
	
	public Position getClosestPoint(Position referencePosition, double distanceShift, boolean reversed, double steps) {
		double minT = 0;
		double maxT = 0;
		double closestDistance = 9999;
		for (int i = 0; i < steps + 1; i++) {
			double t = i / steps;
			Transform transform = getTransform(t);
			double distance = Math.abs(transform.getPosition().distance(referencePosition));
			if (distance < closestDistance) {
				if (reversed) {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() < 0) {
						closestDistance = distance;
						maxT = t;
						minT = t - 1 / steps;
						while(referencePosition.distance(getPosition(minT)) < distanceShift){
							minT -= 1/steps;
						}
					}
				} else {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() > 0) {
						closestDistance = distance;
						minT = t;
						maxT = t + 1 / steps;
						while(referencePosition.distance(getPosition(maxT)) < distanceShift){
							maxT += 1/steps;
						}
					}
				}
			}
		}

		double tFinal = 0;
		
		double secondStepRate = 0.001;
		
		closestDistance = 9999;
		if(reversed){
			for (double t = minT; t < maxT; t += secondStepRate) {
				Transform transform = getTransform(t);
				double distance = Math.abs(transform.getPosition().distance(referencePosition) - distanceShift);
				if (distance < closestDistance) {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() > 0) {
						closestDistance = distance;
						tFinal = t;
					}
				}
			}
		}
		else{
			for (double t = minT; t < maxT; t += secondStepRate) {
				Transform transform = getTransform(t);
				double distance = Math.abs(transform.getPosition().distance(referencePosition) - distanceShift);
				if (distance < closestDistance) {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() < 0) {
						closestDistance = distance;
						tFinal = t;
					}
				}
			}
		}
		//System.out.println(tFinal);
		
		return getPosition(tFinal);
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
