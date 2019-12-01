package com.amhsrobotics.libs.util.path;

import com.amhsrobotics.libs.auton.motionprofile.TrapezoidalMotionProfile;
import com.amhsrobotics.libs.util.geometry.*;

public class ArcPathSegment extends PathSegment {
	private Arc arc;
	private Transform startPoint;
	private Transform endPoint;
	private double startDistance;
	
	public ArcPathSegment(Arc arc, Transform startPoint, Transform endPoint){
		this.arc = arc;
		this.startPoint = startPoint;
		this.endPoint = endPoint;
	}
	
	@Override
	public Position getIntersection(Transform robotTransform) {
		Rotation angleTo = new Rotation(arc.getCenter().angleTo(robotTransform.getPosition()));
		
		return  new Position(angleTo.cos()*arc.getRadius(), angleTo.sin()*arc.getRadius()).add(arc.getCenter());
	}
	
	@Override
	public boolean onSegment(Position point) {
		return arc.isOnCircle(point);
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
	
	public Transform getStartPoint() {
		return startPoint;
	}
	
	public void setStartPoint(Transform startPoint) {
		this.startPoint = startPoint;
	}
	
	public Transform getEndPoint() {
		return endPoint;
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
	
	public void setEndPoint(Transform endPoint) {
		this.endPoint = endPoint;
	}
	
	public void setStartDistance(double startDistance) {
		this.startDistance = startDistance;
	}
	
}
