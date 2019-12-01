package com.amhsrobotics.libs.util.path;


import com.amhsrobotics.libs.util.geometry.Line;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;

public class LinePathSegment extends PathSegment{
	private Line line;
	private Transform startPoint;
	private Transform endPoint;
	

	
	private double startDistance;
	private double startTime;
	private double endTime;
	
	public LinePathSegment(Line line, Transform startPoint, Transform endPoint){
		this.line = line;
		this.startPoint = startPoint;
		this.endPoint = endPoint;
	}
	
	@Override
	public Position getIntersection(Transform robotTransform) {
		return new Transform(robotTransform.getPosition(), line.getTransform().getRotation().rotateBy(90)).findLineIntersectionPoint(line.getTransform()).getPosition();
	}
	
	@Override
	public boolean onSegment(Position point) {
		boolean withinStartAndEnd = Math.abs(point.distance(startPoint.getPosition()) + point.distance(endPoint.getPosition()) - startPoint.getPosition().distance(endPoint.getPosition())) < 2e-16;
		return line.isColinear(point) && withinStartAndEnd;
	}
	
	@Override
	public double getVelocity() {
		return 0;
	}
	
	@Override
	public PathSegmentType getType() {
		return PathSegmentType.LINE;
	}
	
	@Override
	public ArcPathSegment getArcSegment() {
		System.out.println("Tried to get arc segment, but the segment is a line!");
		return null;
	}
	
	@Override
	public LinePathSegment getLineSegment() {
		return this;
	}
	
	public Line getLine() {
		return line;
	}
	
	public void setLine(Line line) {
		this.line = line;
	}
	
	public Transform getStartPoint() {
		return startPoint;
	}
	
	public void setStartPoint(TrajectoryPoint startPoint) {
		this.startPoint = startPoint;
	}
	
	public Transform getEndPoint() {
		return endPoint;
	}
	
	@Override
	public double getSegmentDistance() {
		return startPoint.getPosition().distance(endPoint.getPosition());
	}
	
	@Override
	public double getStartTime() {
		return startTime;
	}
	
	@Override
	public double getEndTime() {
		return endTime;
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
	public double getSegmentTime() {
		return endTime-startTime;
	}
	
	@Override
	public double getRemainingDistance(Position intersectionPoint) {
		return intersectionPoint.distance(endPoint.getPosition());
	}

	
	public void setEndPoint(TrajectoryPoint endPoint) {
		this.endPoint = endPoint;
	}
	
	public void setStartPoint(Transform startPoint) {
		this.startPoint = startPoint;
	}
	
	public void setEndPoint(Transform endPoint) {
		this.endPoint = endPoint;
	}
	
	public void setStartDistance(double startDistance) {
		this.startDistance = startDistance;
	}
	
	public void setStartTime(double startTime) {
		this.startTime = startTime;
	}
	
	public void setEndTime(double endTime) {
		this.endTime = endTime;
	}
}
