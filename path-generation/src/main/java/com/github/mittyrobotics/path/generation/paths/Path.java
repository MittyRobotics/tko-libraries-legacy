package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.datatypes.PathTransform;

public abstract class Path implements Parametric {
	private Transform[] waypoints;
	private Parametric[] parametrics;
	
	public Path(Transform[] waypoints) {
		this.waypoints = waypoints;
		generateParametricEquations();
	}
	
	public abstract void generateParametricEquations();
	
	@Override
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
	
	@Override
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
	
	@Override
	public double getCurvature(double t) {
		//Convert t from 0 to 1 to 0 to waypoints.length-1 so that the t value represents all parametric equation
		//segments of the total path.
		t = t * parametrics.length;
		
		for (int i = 0; i < parametrics.length; i++) {
			//Find which parametric equation segment that t falls in
			if (t >= i && t <= i + 1) {
				//return the Transform from the segment that it falls in
				return parametrics[i].getCurvature(t - i);
			}
		}
		
		return 1 / 2e16;
	}
	
	public PathTransform getClosestTransform(Position referencePosition, double searchIncrement, double searches) {
		return getClosestTransform(referencePosition, 0, false, searchIncrement, searches);
	}
	
	
	public PathTransform getClosestTransform(Position referencePosition, double distanceShift, boolean reversed, double searchIncrement, double searches) {
		double distanceToStartWaypoint = referencePosition.distance(getStartWaypoint().getPosition());
		double distanceToEndWaypoint = referencePosition.distance(getEndWaypoint().getPosition());
		
		if (reversed && distanceToStartWaypoint <= distanceShift) {
			double distanceOffset = distanceShift - distanceToStartWaypoint;
			Rotation rotation = getStartWaypoint().getRotation().add(new Rotation(180));
			Position position = getStartWaypoint().getPosition().add(new Position(rotation.cos() * distanceOffset, rotation.sin() * distanceOffset));
			return new PathTransform(new Transform(position), 1);
		} else if (!reversed && distanceToEndWaypoint <= distanceShift) {
			double distanceOffset = distanceShift - distanceToEndWaypoint;
			Rotation rotation = getEndWaypoint().getRotation();
			Position position = getEndWaypoint().getPosition().add(new Position(rotation.cos() * distanceOffset, rotation.sin() * distanceOffset));
			return new PathTransform(new Transform(position), 1);
		}
		
		double tFinal;
		if (distanceShift == 0) {
			tFinal = getClosestT(referencePosition, searchIncrement, searches);
		} else {
			tFinal = getClosestT(referencePosition, distanceShift, reversed, searchIncrement, searches);
		}
		
		Transform transform = getTransform(tFinal);
		return new PathTransform(transform, tFinal);
	}
	
	public double getClosestT(Position referencePosition, double searchIncrement, double searches) {
		double tFinal = 0;
		double minSearchT = 0;
		double maxSearchT = 1;
		double finalMinSearchT = 0;
		double finalMaxSearchT = 1;
		double closestDistance = Double.POSITIVE_INFINITY;
		for (int j = 1; j < searches + 1; j++) {
			double currentIncrement = 1 / Math.pow(searchIncrement, j);
			for (double t = finalMinSearchT; t < finalMaxSearchT; t += currentIncrement) {
				Transform transform = getTransform(t);
				double distance = Math.abs(transform.getPosition().distance(referencePosition));
				if (distance < closestDistance) {
					closestDistance = distance;
					minSearchT = t - currentIncrement;
					maxSearchT = t + currentIncrement;
					tFinal = t;
				}
			}
			finalMinSearchT = minSearchT;
			finalMaxSearchT = maxSearchT;
		}
		
		return tFinal;
	}
	
	public double getClosestT(Position referencePosition, double distanceShift, boolean reversed, double searchIncrement, double searches) {
		double actualClosestT = getClosestT(referencePosition, searchIncrement, searches);
		
		if (distanceShift == 0) {
			return actualClosestT;
		}
		
		double tFinal = 0;
		double closestDistance = Double.POSITIVE_INFINITY;
		double minSearchT = 0;
		double maxSearchT = 1;
		double finalMinSearchT = 0;
		double finalMaxSearchT = 1;
		for (int j = 1; j < searches + 1; j++) {
			double currentIncrement = 1 / Math.pow(searchIncrement, j);
			for (double t = finalMinSearchT; t < finalMaxSearchT; t += currentIncrement) {
				Transform transform = getTransform(t);
				double distance = Math.abs(transform.getPosition().distance(referencePosition) - distanceShift);
				if (distance < closestDistance && ((reversed && actualClosestT >= t) || (!reversed && actualClosestT <= t))) {
					closestDistance = distance;
					minSearchT = t - currentIncrement;
					maxSearchT = t + currentIncrement;
					tFinal = t;
				}
			}
			finalMinSearchT = minSearchT;
			finalMaxSearchT = maxSearchT;
		}
		
		return tFinal;
	}
	
	private double map(double val, double valMin, double valMax, double desiredMin, double desiredMax) {
		return (val - valMin) / (valMax - valMin) * (desiredMax - desiredMin) + desiredMin;
	}
	
	public Path calculateAdaptedPath(Transform newStartTransform, double adjustPathDistance, boolean reversed) {
		Transform onPathPoint = getClosestTransform(newStartTransform.getPosition(), adjustPathDistance, reversed, 10, 3);
		Transform[] waypoints = getWaypoints();
		
		double pathPointT = getClosestT(onPathPoint.getPosition(), 0, reversed, 10, 3);
		
		int startWaypointIndex = 0;
		double currentClosest = 9999;
		for (int i = 0; i < waypoints.length; i++) {
			double waypointT = getClosestT(waypoints[i].getPosition(), 0, reversed, 10, 3);
			double distance = waypoints[i].getPosition().distance(onPathPoint.getPosition());
			if (distance < currentClosest && waypointT > pathPointT) {
				currentClosest = distance;
				startWaypointIndex = i;
			}
		}
		
		Transform[] adjustedPathWaypoints;
		
		if (onPathPoint.getPosition().distance(getEndWaypoint().getPosition()) > adjustPathDistance) {
			adjustedPathWaypoints = new Transform[waypoints.length - startWaypointIndex + 2];
			adjustedPathWaypoints[0] = new Transform(newStartTransform.getPosition(), newStartTransform.getPosition().angleTo(onPathPoint.getPosition()));
			adjustedPathWaypoints[1] = onPathPoint;
			for (int i = 2; i < adjustedPathWaypoints.length; i++) {
				adjustedPathWaypoints[i] = waypoints[(i - 2) + startWaypointIndex];
			}
		} else {
			adjustedPathWaypoints = new Transform[waypoints.length - startWaypointIndex + 1];
			adjustedPathWaypoints[0] = new Transform(newStartTransform.getPosition(), newStartTransform.getPosition().angleTo(getEndWaypoint().getPosition()));
			for (int i = 1; i < adjustedPathWaypoints.length; i++) {
				adjustedPathWaypoints[i] = waypoints[(i - 1) + startWaypointIndex];
			}
		}
		
		return new CubicHermitePath(adjustedPathWaypoints);
	}
	
	public Transform[] getWaypoints() {
		return waypoints;
	}
	
	public Transform getStartWaypoint() {
		return waypoints[0];
	}
	
	public Transform getEndWaypoint() {
		return waypoints[waypoints.length - 1];
	}
	
	public Parametric[] getParametrics() {
		return parametrics;
	}
	
	public void setParametrics(Parametric[] parametrics) {
		this.parametrics = parametrics;
	}
}
