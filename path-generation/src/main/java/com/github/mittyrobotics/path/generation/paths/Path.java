package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
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
	
	public Position getClosestPoint(Position referencePosition, double distanceShift, boolean reversed, double initialSteps, double refinedSteps) {
		double closestDistance;
		double distanceToStartWaypoint = referencePosition.distance(waypoints[0].getPosition());
		double distanceToEndWaypoint = referencePosition.distance(waypoints[waypoints.length - 1].getPosition());
		
		Transform relativeStart = new Transform(referencePosition).relativeTo(waypoints[0]);
		Transform relativeEnd = new Transform(referencePosition).relativeTo(waypoints[waypoints.length - 1]);
		
		if (reversed && relativeStart.getPosition().getX() < 0.3) {
			Transform waypoint = waypoints[0];
			return waypoint.getPosition().add(new Position(waypoint.getRotation().add(new Rotation(180)).cos()*distanceShift,waypoint.getRotation().add(new Rotation(180)).sin()*distanceShift));
		} else if (!reversed && relativeEnd.getPosition().getX() > -0.3) {
			Transform waypoint = waypoints[waypoints.length - 1];
			return waypoint.getPosition().add(new Position(waypoint.getRotation().cos()*distanceShift,waypoint.getRotation().sin()*distanceShift));
		}

		if(distanceToStartWaypoint <= distanceShift || distanceToEndWaypoint <= distanceShift ){
			Line line = null;
			Transform closestWaypoint = null;
			if(distanceToStartWaypoint <= distanceShift && reversed){
				line = new Line(waypoints[0].getPosition(),waypoints[0].getPosition().add(new Position(waypoints[0].getRotation().cos(),waypoints[0].getRotation().sin())));
				closestWaypoint = waypoints[0];
			}
			else if(distanceToEndWaypoint <= distanceShift && !reversed){
				line = new Line(waypoints[waypoints.length-1].getPosition(),waypoints[waypoints.length-1].getPosition().add(new Position(waypoints[waypoints.length-1].getRotation().cos(),waypoints[waypoints.length-1].getRotation().sin())));
				closestWaypoint = waypoints[waypoints.length-1];
			}

			if(line != null){
				Circle circle = new Circle(referencePosition,distanceShift);
				Position[] positions = circle.circleLineIntersection(line);
				Position position = new Position();
				for(int i = 0; i < positions.length; i++){
					double distance = Math.abs(positions[i].distance(referencePosition)-distanceShift);
					Position relative = new Transform(positions[i]).relativeTo(closestWaypoint).getPosition();
					if(reversed){
						if(relative.getX() < 0){
							position = positions[i];
						}
					}
					else{
						if(relative.getX() > 0){
							position = positions[i];
						}
					}
				}
				return position;
			}
		}

		
		double minT = 0;
		double maxT = 0;
		closestDistance = 9999;
		for (int i = 0; i < initialSteps + 1; i++) {
			double t = i / initialSteps;
			Transform transform = getTransform(t);
			double distance = Math.abs(transform.getPosition().distance(referencePosition));
			if (distance < closestDistance) {
				if (reversed) {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() < 0) {
						closestDistance = distance;
						maxT = t;
						minT = t - 1 / initialSteps;
						while (referencePosition.distance(getPosition(minT)) < distanceShift) {
							minT -= 1 / initialSteps;
						}
					}
				} else {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() > 0) {
						closestDistance = distance;
						minT = t;
						maxT = t + 1 / initialSteps;
						while (referencePosition.distance(getPosition(maxT)) < distanceShift) {
							maxT += 1 / initialSteps;
						}
					}
				}
			}
		}
		
		double tFinal = 0;
		
		double secondStepRate = 1/refinedSteps;
		
		closestDistance = 9999;
		if (reversed) {
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
		} else {
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
		Position position = getPosition(tFinal);
		return position;
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
