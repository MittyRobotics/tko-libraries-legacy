package com.amhsrobotics.datatypes.libs.util.path;


import com.amhsrobotics.datatypes.libs.util.geometry.Arc;
import com.amhsrobotics.datatypes.libs.util.geometry.Line;
import com.amhsrobotics.datatypes.libs.util.geometry.Position;
import com.amhsrobotics.datatypes.libs.util.geometry.Transform;

public class LinePathSegment extends PathSegment{
	private Line line;
	
	private double startDistance;
	
	public LinePathSegment(Line line, Transform startPoint, Transform endPoint){
		this.line = line;
		setStartPoint(startPoint);
		setEndPoint(endPoint);
	}
	
	@Override
	public Position getParallelIntersection(Transform referenceTransform) {
		return new Transform(referenceTransform.getPosition(), line.getTransform().getRotation().rotateBy(90)).findLineIntersectionPoint(line.getTransform()).getPosition();
	}
	
	@Override
	public boolean onSegment(Position point) {
		boolean withinStartAndEnd = Math.abs(point.distance(getStartPoint().getPosition()) + point.distance(getEndPoint().getPosition()) - getStartPoint().getPosition().distance(getEndPoint().getPosition())) < 2e-16;
		return line.isColinear(point) && withinStartAndEnd;
	}
	
	@Override
	public Position getIntersectionPointWithCircle(Arc circle) {
		Position[] positions = line.intersectionPointsWithCircle(circle);

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
	

	@Override
	public double getSegmentDistance() {
		return getStartPoint().getPosition().distance(getEndPoint().getPosition());
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
		return intersectionPoint.distance(getEndPoint().getPosition());
	}
	
	
	public void setStartDistance(double startDistance) {
		this.startDistance = startDistance;
	}
	
}
