package com.amhsrobotics.libs.auton.path.generation;

import com.amhsrobotics.libs.auton.motionprofile.TrapezoidalMotionProfile;
import com.amhsrobotics.libs.datatypes.MechanismBounds;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;
import com.amhsrobotics.libs.util.path.PathSegment;
import com.amhsrobotics.libs.util.path.PathSegmentType;

import java.util.ArrayList;
import java.util.Iterator;

public abstract class Path {
	public static enum RoundMode{
		ROUND_UP,
		ROUND_DOWN,
		ROUND_CLOSEST
	}
	
	private final Transform[] waypoints;
	private final VelocityConstraints velocityConstraints;
	private final int segmentSamples;
	private ArrayList<PathSegment> segments = new ArrayList<>();

	public Path(Transform[] waypoints, VelocityConstraints velocityConstraints, int segmentSamples){
		this.waypoints = waypoints;
		this.velocityConstraints = velocityConstraints;
		this.segmentSamples = segmentSamples;
		generatePath();
		generateMaxVelocities();
		generateStartEndVelocities();
		generateSegmentMotionProfiles();
	}
	
	public abstract void generatePath();
	
	public double getTotalDistance(){
		return segments.get(segments.size()-1).getAbsoluteEndDistance();
	}

	public Position getClosestPoint(Transform referenceTransform, RoundMode roundMode, double distanceShift){
		return new Position();
	}
	
	public Position getClosestPointInSegment(Transform referenceTransform, PathSegment pathSegment, double distanceShift){
		return new Position();
	}
	
	public PathSegment getClosestSegment(Transform referenceTransform, RoundMode roundMode, double distanceShift){
		double currentClosest = 9999;
		PathSegment closestSegment = null;
		for(int i = 0; i < getSegments().size(); i++){
			Transform startPoint = getSegments().get(i).getStartPoint();

			if(Math.abs(startPoint.getPosition().distance(referenceTransform.getPosition())-distanceShift) < currentClosest ){
				if(roundMode == RoundMode.ROUND_DOWN){
					if(referenceTransform.relativeTo(startPoint).getPosition().getX() >= 0){
						currentClosest = startPoint.getPosition().distance(referenceTransform.getPosition());
						closestSegment = getSegments().get(i);
					}
				}
				else if(roundMode == RoundMode.ROUND_UP){
					if(referenceTransform.relativeTo(startPoint).getPosition().getX() <= 0){
						currentClosest = startPoint.getPosition().distance(referenceTransform.getPosition());
						closestSegment = getSegments().get(i);
					}
				}
				else{
					currentClosest = startPoint.getPosition().distance(referenceTransform.getPosition());
					closestSegment = getSegments().get(i);
				}
			}
		}

		if(closestSegment == null){
			currentClosest = 9999;
			for(int i = 0; i < getSegments().size(); i++) {
				Transform startPoint = getSegments().get(i).getStartPoint();
				if(startPoint.getPosition().distance(referenceTransform.getPosition()) < currentClosest ) {
					currentClosest = startPoint.getPosition().distance(referenceTransform.getPosition());
					closestSegment = getSegments().get(i);
				}
			}
		}
		return closestSegment;
	}
	
	public void generateMaxVelocities(){
		Iterator<PathSegment> iterator = segments.iterator();
		PathSegment previousSegment;
		while(iterator.hasNext()){
			PathSegment segment = iterator.next();
			segment.setMaxVelocity(velocityConstraints.getMaxVelocity());
			if(segment.getType() == PathSegmentType.ARC && segment.getArcSegment().getArc().getRadius() < 50){
				segment.setMaxVelocity(velocityConstraints.getMinVelocity());
			}
		}
	}
	
	public void generateStartEndVelocities(){
		PathSegment previousSegment = null;
		for(int i = 0; i < segments.size(); i++){
		
			double startVelocity;
			double endVelocity;
			if(previousSegment == null){
				startVelocity = 0;
				double finalVelocity = Math.sqrt(2*velocityConstraints.getMaxAcceleration()*segments.get(i).getSegmentDistance());
				endVelocity = finalVelocity;
			}
			else{
				startVelocity = previousSegment.getEndVelocity();
				double fianlVelocity = Math.sqrt(startVelocity*startVelocity+2*velocityConstraints.getMaxAcceleration()*segments.get(i).getSegmentDistance());
				endVelocity = fianlVelocity;
			}
			segments.get(i).setStartVelocity(Math.min(segments.get(i).getMaxVelocity(),startVelocity));
			segments.get(i).setEndVelocity(Math.min(segments.get(i).getMaxVelocity(),endVelocity));
			previousSegment = segments.get(i);
		}
		previousSegment = null;
		for(int i = segments.size()-1; i > 0; i--){
			
			double startVelocity;
			double endVelocity;
			if(previousSegment == null){
				endVelocity = 0;
				double fianlVelocity = Math.sqrt(2*velocityConstraints.getMaxDeceleration()*segments.get(i).getSegmentDistance());
				startVelocity = fianlVelocity;
			}
			else{
				endVelocity = previousSegment.getStartVelocity();
				double fianlVelocity = Math.sqrt(endVelocity*endVelocity+2*velocityConstraints.getMaxDeceleration()*segments.get(i).getSegmentDistance());
				startVelocity =  fianlVelocity;
			}
			segments.get(i).setStartVelocity(Math.min(Math.min(segments.get(i).getMaxVelocity(),startVelocity),segments.get(i).getStartVelocity()));
			segments.get(i).setEndVelocity(Math.min(Math.min(segments.get(i).getMaxVelocity(),endVelocity),segments.get(i).getEndVelocity()));
			previousSegment = segments.get(i);
		}
	}
	
	public void generateSegmentMotionProfiles(){
		for(int i = 0; i < segments.size(); i++){
			TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(segments.get(i).getSegmentDistance(),
					new VelocityConstraints(
							velocityConstraints.getMaxAcceleration(),
							velocityConstraints.getMaxDeceleration(),Math.max(segments.get(i).getStartVelocity(),segments.get(i).getEndVelocity()),
							segments.get(i).getStartVelocity(),
							segments.get(i).getEndVelocity()),
					new MechanismBounds(0,0,0));
			
			segments.get(i).setMotionProfile(motionProfile);
		}
	}
	
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
