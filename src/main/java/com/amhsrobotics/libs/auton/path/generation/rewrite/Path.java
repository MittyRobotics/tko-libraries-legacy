package com.amhsrobotics.libs.auton.path.generation.rewrite;

import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.util.geometry.Transform;
import com.amhsrobotics.libs.util.path.PathSegment;

import java.util.ArrayList;

public abstract class Path {
	private final Transform[] waypoints;
	private final VelocityConstraints velocityConstraints;
	private final int segmentSamples;
	private ArrayList<PathSegment> segments = new ArrayList<>();
	
	public Path(Transform[] waypoints, VelocityConstraints velocityConstraints, int segmentSamples){
		this.waypoints = waypoints;
		this.velocityConstraints = velocityConstraints;
		this.segmentSamples = segmentSamples;
		generatePath();
	}
	
	public abstract void generatePath();
	
	public ArrayList<PathSegment> getSegments() {
		return segments;
	}
	
	public void setSegments(ArrayList<PathSegment> segments) {
		this.segments = segments;
	}
	
	public Transform[] getWaypoints() {
		return waypoints;
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
	
	public int getSegmentSamples() {
		return segmentSamples;
	}
}
