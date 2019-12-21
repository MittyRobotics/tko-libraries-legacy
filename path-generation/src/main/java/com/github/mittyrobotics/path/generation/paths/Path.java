package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

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
	
	public Transform getClosestTransform(Position referencePosition, double distanceShift, boolean reversed, double initialSteps, double refinedSteps) {
		double closestDistance;
		double distanceToStartWaypoint = referencePosition.distance(waypoints[0].getPosition());
		double distanceToEndWaypoint = referencePosition.distance(waypoints[waypoints.length - 1].getPosition());
		Transform relativeStart = new Transform(referencePosition).relativeTo(waypoints[0]);
		Transform relativeEnd = new Transform(referencePosition).relativeTo(waypoints[waypoints.length - 1]);
		
		if (reversed && relativeStart.getPosition().getX() < 0.3 && distanceToStartWaypoint < distanceShift) {
			Transform waypoint = waypoints[0];
			Position position = waypoint.getPosition().add(new Position(waypoint.getRotation().add(new Rotation(180)).cos() * distanceShift, waypoint.getRotation().add(new Rotation(180)).sin() * distanceShift));
			return new Transform(position, waypoint.getRotation().add(new Rotation(180)));
		} else if (!reversed && relativeEnd.getPosition().getX() > -0.3 && distanceToEndWaypoint < distanceShift) {
			Transform waypoint = waypoints[waypoints.length - 1];
			Position position = waypoint.getPosition().add(new Position(waypoint.getRotation().cos() * distanceShift, waypoint.getRotation().sin() * distanceShift));
			return new Transform(position, waypoint.getRotation());
		}
		
		if (distanceToStartWaypoint <= distanceShift || distanceToEndWaypoint <= distanceShift) {
			Line line = null;
			Transform closestWaypoint = null;
			if (distanceToStartWaypoint <= distanceShift && reversed) {
				line = new Line(waypoints[0].getPosition(), waypoints[0].getPosition().add(new Position(waypoints[0].getRotation().cos(), waypoints[0].getRotation().sin())));
				closestWaypoint = waypoints[0];
			} else if (distanceToEndWaypoint <= distanceShift && !reversed) {
				line = new Line(waypoints[waypoints.length - 1].getPosition(), waypoints[waypoints.length - 1].getPosition().add(new Position(waypoints[waypoints.length - 1].getRotation().cos(), waypoints[waypoints.length - 1].getRotation().sin())));
				closestWaypoint = waypoints[waypoints.length - 1];
			}
			
			if (line != null) {
				Circle circle = new Circle(referencePosition, distanceShift);
				Position[] positions = circle.circleLineIntersection(line);
				Position position = new Position();
				for (int i = 0; i < positions.length; i++) {
					double distance = Math.abs(positions[i].distance(referencePosition) - distanceShift);
					Position relative = new Transform(positions[i]).relativeTo(closestWaypoint).getPosition();
					if (reversed) {
						if (relative.getX() < 0) {
							position = positions[i];
						}
					} else {
						if (relative.getX() > 0) {
							position = positions[i];
						}
					}
				}
				return new Transform(position, line.getLineAngle());
			}
		}
		
		
		double tFinal = getClosestT(referencePosition, distanceShift, reversed, initialSteps, refinedSteps);
		
		Transform transform = getTransform(tFinal);
		return transform;
	}
	
	private double getClosestT(Position referencePosition, double distanceShift, boolean reversed, double initialSteps, double refinedSteps) {
		double minT = 0;
		double maxT = 0;
		double closestDistance = 9999;
		
		if (waypoints[0].getPosition().distance(referencePosition) < 0.001 && distanceShift == 0) {
			return 0;
		}
		if(waypoints[waypoints.length-1].getPosition().distance(referencePosition) < 0.001 && distanceShift == 0){
			return 1;
		}
		
		for (int i = 0; i < initialSteps + 1; i++) {
			double t = i / initialSteps;
			Transform transform = getTransform(t);
			double distance = Math.abs(transform.getPosition().distance(referencePosition));
			if (distance < closestDistance) {
				if (reversed) {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() <= 0) {
						closestDistance = distance;
						maxT = t;
						minT = t - 1 / initialSteps;
						while (referencePosition.distance(getPosition(minT)) < distanceShift) {
							minT -= 1 / initialSteps;
						}
					}
				} else {
					if (new Transform(referencePosition).relativeTo(transform).getPosition().getX() >= 0) {
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
		double secondStepRate = 1 / refinedSteps;
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
		return tFinal;
	}
	
	public Path calculateAdaptedPath(Transform newStartTransform, double adjustPathDistance, boolean reversed) {
		Transform onPathPoint = getClosestTransform(newStartTransform.getPosition(), adjustPathDistance, reversed, 10, 100);
		Transform[] waypoints = getWaypoints();
		
		double pathPointT = getClosestT(onPathPoint.getPosition(), 0, reversed, 10, 100);
		
		int startWaypointIndex = 0;
		double currentClosest = 9999;
		for (int i = 0; i < waypoints.length; i++) {
			double waypointT = getClosestT(waypoints[i].getPosition(), 0, reversed, 10, 100);
			double distance = waypoints[i].getPosition().distance(onPathPoint.getPosition());
			if (distance < currentClosest && waypointT > pathPointT) {
				currentClosest = distance;
				startWaypointIndex = i;
			}
		}
		
		Transform[] adjustedPathWaypoints;
		
		if(onPathPoint.getPosition().distance(waypoints[waypoints.length-1].getPosition()) > adjustPathDistance){
			adjustedPathWaypoints = new Transform[waypoints.length-startWaypointIndex+2];
			adjustedPathWaypoints[0] = new Transform(newStartTransform.getPosition(),newStartTransform.getPosition().angleTo(onPathPoint.getPosition()));
			adjustedPathWaypoints[1] = onPathPoint;
			for (int i = 2; i < adjustedPathWaypoints.length; i++) {
				adjustedPathWaypoints[i] = waypoints[(i-2) + startWaypointIndex];
			}
		}
		else{
			adjustedPathWaypoints = new Transform[waypoints.length-startWaypointIndex+1];
			adjustedPathWaypoints[0] = new Transform(newStartTransform.getPosition(),newStartTransform.getPosition().angleTo(waypoints[waypoints.length-1].getPosition()));
			for (int i = 1; i < adjustedPathWaypoints.length; i++) {
				adjustedPathWaypoints[i] = waypoints[(i-1) + startWaypointIndex];
			}
		}
		
		return new CubicHermitePath(adjustedPathWaypoints);
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
