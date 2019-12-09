package com.github.mittyrobotics.libs.util.path;


import com.github.mittyrobotics.libs.util.geometry.Arc;
import com.github.mittyrobotics.libs.util.geometry.Position;
import com.github.mittyrobotics.libs.util.geometry.Rotation;
import com.github.mittyrobotics.libs.util.geometry.Transform;

public class ArcPathSegment extends PathSegment {
	private Arc arc;
	private double startDistance;
	
	public ArcPathSegment(Arc arc, Transform startPoint, Transform endPoint){
		this.arc = arc;
		setStartPoint(startPoint);
		setEndPoint(endPoint);
		
		
		arc.setMinAngle(Math.min(arc.getCenter().angleTo(startPoint.getPosition()),arc.getCenter().angleTo(endPoint.getPosition())));
		arc.setMaxAngle(Math.max(arc.getCenter().angleTo(startPoint.getPosition()),arc.getCenter().angleTo(endPoint.getPosition())));
	}
	
	@Override
	public Position getParallelIntersection(Transform referenceTransform) {
		Rotation angleTo = new Rotation(arc.getCenter().angleTo(referenceTransform.getPosition()));
		
		return  new Position(angleTo.cos()*arc.getRadius(), angleTo.sin()*arc.getRadius()).add(arc.getCenter());
	}
	
	@Override
	public boolean onSegment(Position point) {
		boolean withinMinAndMax = arc.getCenter().angleTo(point) > arc.getMinAngle() && arc.getCenter().angleTo(point) < arc.getMaxAngle();
		return arc.isOnCircle(point) && withinMinAndMax;
	}
	
	@Override
	public Position getIntersectionPointWithCircle(Arc circle) {
		Position[] positions = arc.intersectionPointsWithCircle(circle);
		double currentClosest = 9999;
		Position currentPosition = null;
		for(int i = 0; i < positions.length; i++){
			if(positions[i].distance(getStartPoint().getPosition()) < currentClosest && onSegment(positions[i])){
				currentClosest = positions[i].distance(getStartPoint().getPosition());
				currentPosition = positions[i];
			}
		}
		if(currentPosition == null){
			currentPosition = getStartPoint().getPosition();
		}
		return currentPosition;
	}
	
	
	@Override
	public PathSegmentType getType() {
		return PathSegmentType.ARC;
	}
	
	@Override
	public ArcPathSegment getArcSegment() {
		return this;
	}
	
	@Override
	public LinePathSegment getLineSegment() {
		System.out.println("Tried to get line segment, but the segment is an arc!");
		return null;
	}
	
	
	public Arc getArc() {
		return arc;
	}
	
	public void setArc(Arc arc) {
		this.arc = arc;
	}
	

	
	@Override
	public double getSegmentDistance() {
		double circumference = 2*Math.PI*arc.getRadius();
		double angleToStart = arc.getMinAngle();
		double angleToEnd = arc.getMaxAngle();
		double angleDifference = Math.abs(angleToStart-angleToEnd);
		return circumference*(angleDifference/360);
	}
	
	
	@Override
	public double getAbsoluteStartDistance() {
		return startDistance;
	}
	
	@Override
	public double getAbsoluteEndDistance() {
		return startDistance+getSegmentDistance();
	}
	
	@Override
	public double getRemainingDistance(Position intersectionPoint) {
		double circumference = 2*Math.PI*arc.getRadius();
		double angleTo = arc.getCenter().angleTo(intersectionPoint);
		double angleDifference = arc.getMaxAngle()-angleTo;
		return circumference*(angleDifference/360);
	}

	
	public void setStartDistance(double startDistance) {
		this.startDistance = startDistance;
	}
	
}
