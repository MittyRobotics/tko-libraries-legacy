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
	
	/**
	 * Returns the {@link Position} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
	 *
	 * @param t the parameter
	 * @return the {@link Position} at the parameter <code>t</code>.
	 */
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
	
	/**
	 * Returns the {@link Transform} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
	 * <p>
	 * The {@link Transform} contains the {@link Position} and {@link Rotation}, with the {@link Rotation} being the
	 * tangent angle at the {@link Position}.
	 *
	 * @param t the parameter
	 * @return the {@link Transform} at the parameter <code>t</code>.
	 */
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
	
	/**
	 * Returns the curvature at point <code>t</code> on the {@link Parametric}.
	 *
	 * @param t the parameter
	 * @return the curvature at the parameter <code>t</code>.
	 */
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
	
	/**
	 * Finds the closest {@link PathTransform} to the <code>referencePosition</code>.
	 * <p>
	 * The {@link PathTransform} contains the {@link Transform} of the point as well as the <code>t</code> value of it
	 * along the {@link Parametric}.
	 * <p>
	 * The closest point is found by sampling <code>searchIncrement</code> amount of points on the {@link Parametric}.
	 * After the initial points are sampled, it finds the closest point and creates a smaller bound to search within.
	 * It then repeats the same process within the smaller bound for a total of <code>searches</code> loops. Therefore,
	 * the total resolution of the closest point is <code>searchIncrement</code> to the power of <code>searches</code>.
	 * <p>
	 * For example, if <code>searchIncrement</code> is 10 and <code>searches</code> is 3, it will get 10 points,
	 * create a smaller boundary, get another 10 points within the boundary, create a smaller boundary again, and lastly
	 * find the closest point within 10 points within the final smaller boundary. This would find a point within the
	 * accuracy of 1000 points, although it only does 30 total samples.
	 * <p>
	 * If the point is outside the start and end of the path, either the start or end {@link Transform} will be picked.
	 *
	 * @param referencePosition the {@link Position} to find the closest {@link PathTransform} to.
	 * @param searchIncrement   the samples within each search.
	 * @param searches          the amount of searches to perform to get the final closest value.
	 * @return the closest {@link PathTransform} to the <code>referencePosition</code>.
	 */
	public PathTransform getClosestTransform(Position referencePosition, double searchIncrement, double searches) {
		return getClosestTransform(referencePosition, 0, false, searchIncrement, searches);
	}
	
	/**
	 * Finds the closest {@link PathTransform} to the <code>referencePosition</code> given a <code>distanceShift</code>.
	 * <p>
	 * The {@link PathTransform} contains the {@link Transform} of the point as well as the <code>t</code> value of it
	 * along the {@link Parametric}.
	 * <p>
	 * The closest point is found by sampling <code>searchIncrement</code> amount of points on the {@link Parametric}.
	 * After the initial points are sampled, it finds the closest point and creates a smaller bound to search within.
	 * It then repeats the same process within the smaller bound for a total of <code>searches</code> loops. Therefore,
	 * the total resolution of the closest point is <code>searchIncrement</code> to the power of <code>searches</code>.
	 * <p>
	 * For example, if <code>searchIncrement</code> is 10 and <code>searches</code> is 3, it will get 10 points,
	 * create a smaller boundary, get another 10 points within the boundary, create a smaller boundary again, and lastly
	 * find the closest point within 10 points within the final smaller boundary. This would find a point within the
	 * accuracy of 1000 points, although it only does 30 total samples.
	 * <p>
	 * If the point is outside the start and end of the path, either the start or end {@link Transform} will be picked.
	 * If the distance shifted point is off the path, it will be interpolated on a line extending either the start or
	 * end {@link Transform} of the path.
	 * <p>
	 * Since there is a <code>distanceShift</code> in this search, it first finds the actual closest point to the
	 * <code>referencePosition</code> and then performs the distance shifted search using the actual closest point as a
	 * guide of whether or not the point is in front or behind the <code>referencePosition</code>. It will only pick
	 * points in front of the actual closest point on the path if reversed is <code>false</code>, and only behind the
	 * actual closest point on the path if reversed is <code>true</code>.
	 *
	 * @param referencePosition the {@link Position} to find the closest {@link PathTransform} to.
	 * @param distanceShift     the distance away from the <code>referencePosition</code> the closest point should be.
	 * @param reversed          whether to find the closest {@link Position} behind or in front of the
	 *                          <code>referencePosition</code>.
	 * @param searchIncrement   the samples within each search.
	 * @param searches          the amount of searches to perform to get the final closest value.
	 * @return the closest {@link PathTransform} to the <code>referencePosition</code>.
	 */
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
	
	/**
	 * Finds the closest <code>t</code> value on the {@link Parametric} to the <code>referencePosition</code>.
	 * <p>
	 * The <code>t</code> value contains the position along the {@link Parametric} between 0 and 1.
	 * <p>
	 * The closest <code>t</code> value of the point is found by sampling <code>searchIncrement</code> amount of points on the
	 * {@link Parametric}. After the initial points are sampled, it finds the closest point and creates a smaller bound
	 * to search within. It then repeats the same process within the smaller bound for a total of <code>searches</code>
	 * loops. Therefore, the total resolution of the closest point is <code>searchIncrement</code> to the power of
	 * <code>searches</code>.
	 * <p>
	 * For example, if <code>searchIncrement</code> is 10 and <code>searches</code> is 3, it will get 10 points,
	 * create a smaller boundary, get another 10 points within the boundary, create a smaller boundary again, and lastly
	 * find the closest point within 10 points within the final smaller boundary. This would find a point within the
	 * accuracy of 1000 points, although it only does 30 total samples.
	 * <p>
	 * If the point is outside the start and end of the path, either 0 or 1 will be picked, representing the first or
	 * last point on the {@link Parametric}.
	 *
	 * @param referencePosition the {@link Position} to find the closest <code>t</code> value to.
	 * @param searchIncrement   the samples within each search.
	 * @param searches          the amount of searches to perform to get the final closest value.
	 * @return the closest <code>t</code> value on the {@link Parametric} that is closest to the
	 * <code>referencePosition</code>.
	 */
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
	
	/**
	 * Finds the closest <code>t</code> value on the {@link Parametric} to the <code>referencePosition</code> given a
	 * <code>distanceShift</code>.
	 * <p>
	 * The {@link PathTransform} contains the {@link Transform} of the point as well as the <code>t</code> value of it
	 * along the {@link Parametric}.
	 * <p>
	 * The closest point is found by sampling <code>searchIncrement</code> amount of points on the {@link Parametric}.
	 * After the initial points are sampled, it finds the closest point and creates a smaller bound to search within.
	 * It then repeats the same process within the smaller bound for a total of <code>searches</code> loops. Therefore,
	 * the total resolution of the closest point is <code>searchIncrement</code> to the power of <code>searches</code>.
	 * <p>
	 * For example, if <code>searchIncrement</code> is 10 and <code>searches</code> is 3, it will get 10 points,
	 * create a smaller boundary, get another 10 points within the boundary, create a smaller boundary again, and lastly
	 * find the closest point within 10 points within the final smaller boundary. This would find a point within the
	 * accuracy of 1000 points, although it only does 30 total samples.
	 * <p>
	 * If the point is outside the start and end of the path, either 0 or 1 will be picked, representing the first or
	 * last point on the {@link Parametric}. If the distance shifted point is off the path, it will also either choose
	 * 0 or 1.
	 * <p>
	 * Since there is a <code>distanceShift</code> in this search, it first finds the actual closest point to the
	 * <code>referencePosition</code> and then performs the distance shifted search using the actual closest point as a
	 * guide of whether or not the point is in front or behind the <code>referencePosition</code>. It will only pick
	 * points in front of the actual closest point on the path if reversed is <code>false</code>, and only behind the
	 * actual closest point on the path if reversed is <code>true</code>.
	 *
	 * @param referencePosition the {@link Position} to find the closest <code>t</code> value to.
	 * @param distanceShift     the distance away from the <code>referencePosition</code> the closest point should be.
	 * @param reversed          whether to find the closest {@link Position} behind or in front of the
	 *                          <code>referencePosition</code>.
	 * @param searchIncrement   the samples within each search.
	 * @param searches          the amount of searches to perform to get the final closest value.
	 * @return the closest <code>t</code> value on the {@link Parametric} that is closest to the
	 * <code>referencePosition</code>.
	 */
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
