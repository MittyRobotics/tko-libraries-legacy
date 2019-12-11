package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.datatypes.PathSegment;

import java.util.ArrayList;

public abstract class Path {
	private final Transform[] waypoints;
	private final MotionState startMotionState;
	private final MotionState endMotionState;
	private final VelocityConstraints velocityConstraints;
	private final double samples;
	
	private ArrayList<PathSegment> segments = new ArrayList<>();
	
	public Path(Transform[] waypoints, MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints, double samples){
		this.waypoints = waypoints;
		this.startMotionState = startMotionState;
		this.endMotionState = endMotionState;
		this.velocityConstraints = velocityConstraints;
		this.samples = samples;
		generatePathSegments();
		generateMotionProfiles();
	}
	
	public abstract void generatePathSegments();
	
	public void generateMotionProfiles(){
	
	}
	
	public Transform[] getWaypoints() {
		return waypoints;
	}
	
	public MotionState getStartMotionState() {
		return startMotionState;
	}
	
	public MotionState getEndMotionState() {
		return endMotionState;
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
	
	
	public ArrayList<PathSegment> getSegments() {
		return segments;
	}
	
	public void setSegments(ArrayList<PathSegment> segments) {
		this.segments = segments;
	}

	public double getSamples() {
		return samples;
	}
}
