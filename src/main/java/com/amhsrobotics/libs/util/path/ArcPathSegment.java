package com.amhsrobotics.libs.util.path;

import com.amhsrobotics.libs.util.geometry.Arc;
import com.amhsrobotics.libs.util.geometry.Line;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;

public class ArcPathSegment extends PathSegment {
	private Arc arc;
	private Transform startPoint;
	private Transform endPoint;
	
	public ArcPathSegment(Arc arc, Transform startPoint, Transform endPoint){
		this.arc = arc;
		this.startPoint = startPoint;
		this.endPoint = endPoint;
	}
	
	@Override
	public Position getIntersection(Transform robotTransform) {
		return null;
	}
	
	@Override
	public boolean onSegment(Position point) {
		return arc.isOnCircle(point);
	}
	
	@Override
	public double getVelocity() {
		return 0;
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
	
	public void setEndPoint(Transform endPoint) {
		this.endPoint = endPoint;
	}
}
